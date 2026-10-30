"""对仿真轨迹做已知地图几何核对；不能代替接触传感器和真实仿真观察。"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .planning import clearance,load_world,segment_safe


def main():
    p=argparse.ArgumentParser(); p.add_argument('csv')
    p.add_argument('--world',default='config/world.json')
    p.add_argument('--goal',nargs=2,type=float)
    args=p.parse_args(); world=load_world(args.world)
    rows=list(csv.DictReader(Path(args.csv).open(encoding='utf-8-sig',newline='')))
    if len(rows)<2: raise ValueError('至少需要2个轨迹样本')
    points=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows])
    radius=world['robot_radius']
    margins=[clearance(world,p)-radius for p in points]
    safe=all(segment_safe(world,a,b,radius) for a,b in zip(points,points[1:]))
    goal=world['goal'] if args.goal is None else args.goal
    print(json.dumps({'samples':len(rows),'min_body_clearance_m':min(margins),
                      'sampled_path_geometrically_clear':safe,
                      'final_goal_error_m':float(np.linalg.norm(points[-1]-goal)),
                      'note':'只核对已知静态地图和采样轨迹；不是接触事件证明，也不涵盖未记录时间'},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
