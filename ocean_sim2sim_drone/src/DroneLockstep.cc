// DroneLockstep: Gazebo side of the PX4 drone (docs/drone_px4.md).
//
// Every `period` of sim time (1/120 s: the Isaac Sim physics / PX4 lockstep rate) the plugin sends the drone body state
// to the bridge node (scripts/px4_bridge_node.py, a TCP server on localhost) and WAITS for its reply: the body wrench
// from the rotors (thrust k w^2 at the rotor positions, reaction torque) and the linear drag, computed there with the
// Isaac Sim / Pegasus thrust model from the PX4 motor commands. While waiting the physics does not advance = lockstep
// (the bridge in turn waits for PX4). The wrench is applied at every physics step until the next reply (240 Hz physics:
// 2 steps per exchange).
//
// Prop spin (visual, Isaac Sim drone_flight rule computed by the bridge): the reply also carries the 4 prop joint
// speeds, set as joint velocity commands (JointVelocityCmd) on the rotor joints at every physics step.
//
// No bridge connected: no force, props held still (the drone hangs on its world anchor). Connection lost or no reply
// within `timeout`: error, no force, props stopped (no silent fallback). Reconnects when the bridge comes back.
//
// Wire format (little endian float64):
//   plugin -> bridge  15 doubles: sim time [s], position (3) [m, world], orientation (w x y z, world),
//                                 linear velocity (3) [m/s, world], angular velocity (3) [rad/s, world], step dt [s]
//   bridge -> plugin  10 doubles: force (3) [N, world], torque (3) [N m, world, about the link origin],
//                                 rotor joint velocities (4) [rad/s, rotor_joints order]
//
// SDF:
//   <plugin filename="ocean_sim2sim_drone-DroneLockstep" name="ocean_sim2sim::DroneLockstep">
//     <link_name>body</link_name> <rotor_joints>rotor0_joint rotor1_joint rotor2_joint rotor3_joint</rotor_joints>
//     <port>4570</port> <rate>120</rate> <timeout>10</timeout>
//   </plugin>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

namespace ocean_sim2sim
{
class DroneLockstep : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate
{
public:
  ~DroneLockstep() override { this->Close(); }

  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    const std::string linkName = _sdf->Get<std::string>("link_name", "body").first;
    this->port = _sdf->Get<int>("port", 4570).first;
    this->period = 1.0 / _sdf->Get<double>("rate", 120.0).first;
    this->timeout = _sdf->Get<double>("timeout", 10.0).first;
    const auto linkEntity = this->model.LinkByName(_ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[DroneLockstep] link [" << linkName << "] not found in model [" << this->model.Name(_ecm) << "]\n";
      return;
    }
    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(_ecm, true);
    std::istringstream names(_sdf->Get<std::string>("rotor_joints", "").first);
    std::string jn;
    while (names >> jn)
    {
      const auto j = this->model.JointByName(_ecm, jn);
      if (j == gz::sim::kNullEntity)
      {
        gzerr << "[DroneLockstep] rotor joint [" << jn << "] not found in model [" << this->model.Name(_ecm) << "]\n";
        return;
      }
      this->joints.emplace_back(j);
    }
    if (this->joints.size() != kRotors)
    {
      gzerr << "[DroneLockstep] <rotor_joints> needs " << kRotors << " joint names, got " << this->joints.size() << "\n";
      return;
    }
    this->ok = true;
    gzmsg << "[DroneLockstep] " << this->model.Name(_ecm) << "::" << linkName << " <-> bridge 127.0.0.1:" << this->port
          << ", exchange every " << this->period * 1000.0 << " ms sim time\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ok || _info.paused)
      return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (this->fd < 0 && t >= this->nextConnectTry)
    {
      this->Connect();
      this->nextConnectTry = t + 0.5;
      if (this->fd >= 0)
        this->stepsLeft = 0;        // exchange at once
    }
    // count physics steps (Gazebo time is integer ns: 1/240 s does not add up to exactly 1/120 s, comparing times
    // skipped an exchange now and then -> 12.5 ms instead of 8.33 ms)
    if (this->fd >= 0 && this->stepsLeft <= 0)
    {
      this->stepsLeft = std::max(1L, std::lround(this->period / dt));
      if (!this->Exchange(_ecm, t, dt))
      {
        this->Close();
        this->reply.fill(0.0);
      }
    }
    if (this->fd >= 0)
      --this->stepsLeft;
    // prop joint speeds every step (0 = held still while no bridge / after an error)
    for (size_t i = 0; i < kRotors; ++i)
      this->joints[i].SetVelocity(_ecm, {this->fd >= 0 ? this->reply[6 + i] : 0.0});
    if (this->fd >= 0)
      this->link.AddWorldWrench(_ecm, gz::math::Vector3d(this->reply[0], this->reply[1], this->reply[2]),
                                gz::math::Vector3d(this->reply[3], this->reply[4], this->reply[5]));
  }

private:
  void Connect()
  {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0)
      return;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(this->port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
      ::close(s);
      return;
    }
    int one = 1;
    ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    this->fd = s;
    gzmsg << "[DroneLockstep] connected to the bridge (lockstep on)\n";
  }

  void Close()
  {
    if (this->fd >= 0)
    {
      ::close(this->fd);
      this->fd = -1;
    }
  }

  bool Exchange(const gz::sim::EntityComponentManager &_ecm, double _t, double _dt)
  {
    const auto pose = this->link.WorldPose(_ecm);
    const auto v = this->link.WorldLinearVelocity(_ecm);
    const auto w = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !v || !w)
      return true;                  // components not ready in the first step
    const auto &p = pose->Pos();
    const auto &q = pose->Rot();
    const std::array<double, 15> out{_t, p.X(), p.Y(), p.Z(), q.W(), q.X(), q.Y(), q.Z(),
                                     v->X(), v->Y(), v->Z(), w->X(), w->Y(), w->Z(), _dt};
    if (!this->WriteAll(out.data(), sizeof(out)))
    {
      gzerr << "[DroneLockstep] bridge connection lost (send) at sim " << _t << " s: no rotor force\n";
      return false;
    }
    std::array<double, 6 + kRotors> in{};
    if (!this->ReadAll(in.data(), sizeof(in)))
    {
      gzerr << "[DroneLockstep] no reply from the bridge within " << this->timeout << " s (or connection lost) at sim "
            << _t << " s: no rotor force\n";
      return false;
    }
    for (double x : in)
      if (!std::isfinite(x))
      {
        gzerr << "[DroneLockstep] non-finite reply from the bridge: no rotor force\n";
        return false;
      }
    this->reply = in;
    return true;
  }

  bool WriteAll(const void *_buf, size_t _n)
  {
    const char *b = static_cast<const char *>(_buf);
    while (_n > 0)
    {
      ssize_t k = ::send(this->fd, b, _n, MSG_NOSIGNAL);
      if (k <= 0)
        return false;
      b += k;
      _n -= static_cast<size_t>(k);
    }
    return true;
  }

  bool ReadAll(void *_buf, size_t _n)
  {
    char *b = static_cast<char *>(_buf);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(this->timeout);
    while (_n > 0)
    {
      const int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                        deadline - std::chrono::steady_clock::now()).count());
      if (ms <= 0)
        return false;
      pollfd pfd{this->fd, POLLIN, 0};
      if (::poll(&pfd, 1, ms) <= 0)
        return false;
      ssize_t k = ::recv(this->fd, b, _n, 0);
      if (k <= 0)
        return false;
      b += k;
      _n -= static_cast<size_t>(k);
    }
    return true;
  }

  static constexpr size_t kRotors = 4;
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link link{gz::sim::kNullEntity};
  bool ok{false};
  int port{4570};
  double period{1.0 / 120.0};
  double timeout{10.0};
  int fd{-1};
  double nextConnectTry{0.0};
  long stepsLeft{0};
  std::array<double, 6 + kRotors> reply{};   // wrench (6) + rotor joint velocities (4)
  std::vector<gz::sim::Joint> joints;
};
}  // namespace ocean_sim2sim

GZ_ADD_PLUGIN(ocean_sim2sim::DroneLockstep, gz::sim::System, ocean_sim2sim::DroneLockstep::ISystemConfigure,
              ocean_sim2sim::DroneLockstep::ISystemPreUpdate)
