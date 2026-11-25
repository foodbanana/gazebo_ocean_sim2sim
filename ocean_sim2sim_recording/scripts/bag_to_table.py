#!/usr/bin/env python3
# =============================================================
# bag_to_table.py - recording run directory (bag/ + meta.json) -> 100 Hz table again (docs/recording.md)
#
#   record_run.py does this at the end of a run; use this after --no-convert or to rebuild the table.
#       ros2 run ocean_sim2sim_recording bag_to_table.py ocean_gazebo_recordings/<run>
#   Writes <run>/data_100hz.npz, data_100hz.csv, e1_timing.json and updates meta.json "table".
# =============================================================
import argparse
import json
import os

from ocean_sim2sim_recording import convert, meta


def main():
    p = argparse.ArgumentParser(description='Recording run -> 100 Hz table (npz + csv) and E1 timing.')
    p.add_argument('run_dir')
    a = p.parse_args()
    e1, header = convert.convert(a.run_dir)
    path = os.path.join(a.run_dir, 'meta.json')
    if os.path.exists(path):
        m = json.load(open(path))
        m['table'] = {'file_npz': 'data_100hz.npz', 'file_csv': 'data_100hz.csv', 'columns': header,
                      'e1_summary': {k: e1[k] for k in ('grid', 'rtf') if k in e1}}
        meta.write_json(path, m)
    print(json.dumps(e1['grid']), json.dumps(e1.get('rtf')))


if __name__ == '__main__':
    main()
