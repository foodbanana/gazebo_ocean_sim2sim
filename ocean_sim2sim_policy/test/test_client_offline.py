"""Offline checks of the policy client (no ROS graph, no server): observation pairing incl. the base_imu window mean,
chunk interpolation, safety limits, config checks. Run: python3 -m pytest ocean_sim2sim_policy/test (docs/policy.md)."""
import copy
import math
import os

import numpy as np
import pytest
import yaml

from ocean_sim2sim_policy.client import (build_observation, check_config, chunk_step, imu_window_mean, safe_target,
                                         urdf_limits)

CFG_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'policy_client.yaml')
WRIST, BASE = 'observation/wrist_image', 'observation/base_image'


def cfg(**over):
    with open(CFG_FILE) as f:
        c = yaml.safe_load(f)
    for k, v in over.items():
        c[k] = v
    return c


def stamps(t0, t1, hz):
    n = int(round((t1 - t0) * hz))
    return [round(t0 + i / hz, 6) for i in range(n + 1)]


def img(v):
    return np.full((4, 6, 3), v, dtype=np.uint8)


def snapshot(t_now=12.1, cam_late=1, imu=None, with_base=False):
    """Isaac docs/policy_client.md 4.1: joints 120 Hz, gripper 30 Hz, images 30 Hz arriving cam_late frames late."""
    js = [(t, np.full(6, t)) for t in stamps(11.0, t_now, 120)]
    grip = [(t, 1150.0 * (t - 11.0)) for t in stamps(11.0, t_now, 30)]
    cams = stamps(11.0, t_now, 30)[:-cam_late] if cam_late else stamps(11.0, t_now, 30)
    snap = {'js': js, 'grip': grip, 'img': {WRIST: [(t, img(int(t * 30) % 256)) for t in cams[-4:]]}, 'imu': imu}
    if with_base:
        snap['img'][BASE] = [(t, img(1)) for t in cams[-5:-1]]       # third view one more frame late
    return snap


def test_config_file_ok():
    check_config(cfg())


@pytest.mark.parametrize('change, msg', [
    (lambda c: c.pop('cameras'), 'missing'),
    (lambda c: c['base_imu'].pop('window'), 'base_imu.window'),
    (lambda c: c.update(blank_images={WRIST: [480, 640]}), 'both'),
    (lambda c: c.update(cameras={}), 'empty'),
    (lambda c: c['base_imu'].update(mode='append'), 'mode'),
    (lambda c: c.update(max_lead=0.3), 'max_lead'),
    (lambda c: c.update(blank_images={BASE: [480]}), 'height, width'),
])
def test_config_errors(change, msg):
    c = cfg()
    change(c)
    with pytest.raises(ValueError, match=msg):
        check_config(c)


def test_pairing_at_image_stamp():
    snap = snapshot()
    obs, t_obs = build_observation(snap, cfg())
    assert t_obs == pytest.approx(12.0667, abs=1e-3)              # latest image stamp (1 frame late)
    assert obs['observation/state'].dtype == np.float32 and obs['observation/state'].shape == (7,)
    # joints / gripper at the image stamp, not the newest ones
    assert obs['observation/state'][0] == pytest.approx(t_obs, abs=1e-6)
    assert obs['observation/state'][6] == pytest.approx((t_obs - 11.0), abs=1e-5)
    assert obs['prompt'] == 'pick up the drone'
    assert set(obs) == {'observation/state', WRIST, 'prompt'}


def test_pairing_two_cameras_uses_the_later_one():
    obs, t_obs = build_observation(snapshot(with_base=True), cfg(cameras={WRIST: '/w', BASE: '/b'}))
    assert t_obs == pytest.approx(12.0333, abs=1e-3)              # min of the two latest stamps
    assert obs['observation/state'][0] == pytest.approx(t_obs, abs=1e-6)


def test_pairing_stale_gripper_is_an_error():
    snap = snapshot()
    snap['grip'] = [x for x in snap['grip'] if x[0] < 11.99]      # gripper messages stop at 11.9667
    with pytest.raises(RuntimeError, match='gripper'):
        build_observation(snap, cfg())


def test_pairing_nothing_before_t_obs_is_an_error():
    snap = snapshot()
    snap['js'] = [x for x in snap['js'] if x[0] > 12.08]          # joints start after the image stamp
    with pytest.raises(RuntimeError, match='joint_states: no message'):
        build_observation(snap, cfg())


def test_blank_image():
    obs, _ = build_observation(snapshot(), cfg(blank_images={BASE: [480, 640]}))
    assert obs[BASE].shape == (480, 640, 3) and obs[BASE].dtype == np.uint8 and not obs[BASE].any()


# ---- base_imu: identical to stage1 lerobot_stage1_extract_bag.py --base-imu topic
def stage1_mean(imu_t, imu_v, qt, dt):
    sel = (imu_t > qt - dt) & (imu_t <= qt)
    return imu_v[sel].mean(axis=0)


def imu_buffer(t0, t1, hz=100.0, seed=0):
    rng = np.random.default_rng(seed)
    ts = np.array(stamps(t0, t1, hz))
    vals = rng.normal(size=(len(ts), 6)) + np.array([0, 0, 0, 0, 0, 9.81])
    return [(float(t), v) for t, v in zip(ts, vals)], ts, vals


def test_imu_window_mean_equals_stage1():
    buf, ts, vals = imu_buffer(11.0, 12.2)
    for qt in (12.0667, 12.04, 12.0, 11.5333):
        m, t_used = imu_window_mean(buf, qt, 0.040)
        np.testing.assert_allclose(m, stage1_mean(ts, vals, qt, 0.040), rtol=0, atol=1e-12)
        assert qt - 0.040 < t_used <= qt + 1e-9


def test_imu_window_empty_is_an_error():
    buf, _, _ = imu_buffer(11.0, 11.5)
    with pytest.raises(RuntimeError, match='base_imu'):
        imu_window_mean(buf, 12.0, 0.040)


@pytest.mark.parametrize('mode', ['key', 'state'])
def test_pairing_with_base_imu(mode):
    buf, ts, vals = imu_buffer(11.0, 12.1)
    c = cfg()
    c['base_imu'].update(enabled=True, mode=mode)
    obs, t_obs = build_observation(snapshot(imu=buf), c)
    expect = stage1_mean(ts, vals, t_obs, 0.040).astype(np.float32)
    assert t_obs == pytest.approx(12.0667, abs=1e-3)              # IMU arrives before the images: t_obs = image stamp
    if mode == 'key':
        np.testing.assert_allclose(obs['observation/base_imu'], expect, atol=1e-6)
        assert obs['observation/state'].shape == (7,)
    else:
        assert 'observation/base_imu' not in obs
        assert obs['observation/state'].shape == (13,)
        np.testing.assert_allclose(obs['observation/state'][7:], expect, atol=1e-6)


def test_pairing_waits_for_late_imu():
    """IMU behind the images: t_obs = newest IMU stamp (complete window), image = latest at or before it."""
    buf, ts, vals = imu_buffer(11.0, 12.05)
    c = cfg()
    c['base_imu']['enabled'] = True
    obs, t_obs = build_observation(snapshot(imu=buf), c)
    assert t_obs == pytest.approx(12.05, abs=1e-6)
    np.testing.assert_allclose(obs['observation/base_imu'], stage1_mean(ts, vals, t_obs, 0.040), atol=1e-6)


def test_pairing_imu_far_behind_is_an_error():
    buf, _, _ = imu_buffer(11.0, 11.9)                            # IMU stopped 0.17 s before the images
    c = cfg()
    c['base_imu']['enabled'] = True
    snap = snapshot(imu=buf)
    with pytest.raises(RuntimeError, match=WRIST):                 # no image at or before t_obs (buffer of 4 images)
        build_observation(snap, c)


# ---- chunk execution
def test_chunk_interpolation():
    act = np.zeros((10, 7))
    act[:, 0] = np.arange(10) * 0.1
    act[:, 6] = [0, 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.7]
    a, i0, held = chunk_step(act, 5.42)
    assert a[0] == pytest.approx(0.542) and i0 == 5 and not held
    assert act[i0, 6] > 0.5                                       # gripper: value of step floor(x), not interpolated
    a, i0, held = chunk_step(act, 4.99)
    assert a[0] == pytest.approx(0.499) and i0 == 4 and act[i0, 6] == 0
    a, i0, held = chunk_step(act, -0.3)                            # before the chunk: step 0
    assert a[0] == 0 and i0 == 0 and not held
    a, i0, held = chunk_step(act, 9.0)
    assert a[0] == pytest.approx(0.9) and i0 == 9 and held
    a, i0, held = chunk_step(act, 12.7)                            # late next chunk: hold the last step
    assert a[0] == pytest.approx(0.9) and i0 == 9 and held


def test_chunk_timing_example():
    """Isaac docs/policy_client.md 5.3: t_obs 100.167, 60 Hz ticks -> positions 4.58, 5.00, 5.42 (25 Hz steps)."""
    t_obs, dt_c = 100.1667, 1 / 25
    xs = [(100.1667 + 0.1833 + i / 60 - t_obs) / dt_c for i in range(3)]
    np.testing.assert_allclose(xs, [4.58, 5.0, 5.42], atol=0.01)


def test_safe_target():
    lo, hi = np.full(6, -1.0), np.full(6, 1.0)
    q_cmd = np.zeros(6)
    q_meas = np.zeros(6)
    q = safe_target(np.full(6, 0.5), q_cmd, q_meas, 1 / 60, 1.5, 0.08, lo, hi)
    np.testing.assert_allclose(q, 0.025)                           # 1.5 rad/s / 60 Hz
    q = safe_target(np.full(6, 0.5), np.full(6, 0.07), q_meas, 1 / 60, 1.5, 0.08, lo, hi)
    np.testing.assert_allclose(q, 0.08)                            # lead over the measured joints
    q = safe_target(np.full(6, 2.0), np.full(6, 0.99), np.full(6, 0.99), 1 / 60, 1.5, 0.08, lo, hi)
    np.testing.assert_allclose(q, 1.0)                             # joint limit


def test_urdf_limits():
    urdf = """<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
      <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><limit lower="-6.28" upper="6.28"/></joint>
      <joint name="j2" type="continuous"><parent link="b"/><child link="c"/></joint></robot>"""
    lo, hi = urdf_limits(urdf, ['j2', 'j1'])
    assert lo[0] == -math.inf and hi[1] == 6.28
    with pytest.raises(ValueError, match='j3'):
        urdf_limits(urdf, ['j3'])


def test_config_not_modified_by_build():
    c = cfg()
    c0 = copy.deepcopy(c)
    build_observation(snapshot(), c)
    assert c == c0
