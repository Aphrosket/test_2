"""不依赖ROS2的规划和车体安全几何；单位米、弧度。"""
import heapq
import json
import math
from pathlib import Path

import numpy as np


def load_world(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def clearance(world, point):
    """点到障碍/边界的有符号距离；负数表示位于障碍内部。"""
    x, y = point
    xmin, ymin, xmax, ymax = world['bounds']
    distances = [x-xmin, xmax-x, y-ymin, ymax-y]
    for o in world['obstacles']:
        dx, dy = abs(x-o['x']), abs(y-o['y'])
        if o['kind'] == 'cylinder':
            d = math.hypot(dx, dy)-o['radius']
        else:
            qx, qy = dx-o['sx']/2, dy-o['sy']/2
            d = math.hypot(max(qx,0),max(qy,0)) + min(max(qx,qy),0)
        distances.append(d)
    return min(distances)


def segment_safe(world, a, b, radius):
    # 距离场是1-Lipschitz；采样点额外留半个采样间隔，避免漏掉点间碰撞。
    a, b = np.asarray(a,float), np.asarray(b,float)
    length = float(np.linalg.norm(b-a))
    count = max(1,math.ceil(length/0.015))
    extra = length/count/2
    return all(clearance(world,p) > radius+extra for p in np.linspace(a,b,count+1))


def path_safe(world, path, radius):
    return len(path) > 0 and all(clearance(world,p) > radius for p in path) and all(
        segment_safe(world,a,b,radius) for a,b in zip(path,path[1:]))


def astar(world, start, goal):
    radius = world['robot_radius']+world['planning_margin']
    res = world['resolution']
    xmin,ymin,xmax,ymax = world['bounds']
    width,height = math.ceil((xmax-xmin)/res),math.ceil((ymax-ymin)/res)
    def cell(p): return (int(math.floor((p[0]-xmin)/res)),int(math.floor((p[1]-ymin)/res)))
    def position(c): return np.array([xmin+(c[0]+0.5)*res,ymin+(c[1]+0.5)*res])
    for p in (start,goal):
        if clearance(world,p) <= radius:
            raise ValueError('起点或终点在障碍、边界或车体膨胀区域内')
    source,target = cell(start),cell(goal)
    if not segment_safe(world,start,position(source),radius) or not segment_safe(world,position(target),goal,radius):
        raise ValueError('端点与栅格中心的连接不安全')
    queue = [(0.,source)]
    costs,parents,closed = {source:0.},{},set()
    occupied = {}
    def free(c):
        if c not in occupied:
            occupied[c] = (0 <= c[0] < width and 0 <= c[1] < height and
                           clearance(world,position(c)) > radius+res*0.72)
        return occupied[c]
    while queue:
        _,current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == target:
            cells = [current]
            while cells[-1] != source:
                cells.append(parents[cells[-1]])
            result = [np.asarray(start,float)] + [position(c) for c in reversed(cells)] + [np.asarray(goal,float)]
            if not path_safe(world,result,radius):
                raise ValueError('离散路径连接安全检查失败')
            return np.asarray(result)
        closed.add(current)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt = (current[0]+dx,current[1]+dy)
            if not free(nxt) or (dx and dy and (not free((current[0]+dx,current[1])) or not free((current[0],current[1]+dy)))):
                continue
            cost = costs[current]+math.hypot(dx,dy)*res
            if cost < costs.get(nxt,float('inf')):
                costs[nxt],parents[nxt] = cost,current
                heuristic = np.linalg.norm(position(nxt)-position(target))
                heapq.heappush(queue,(cost+heuristic,nxt))
    raise ValueError('地图中没有可达路径')


def smooth_path(world, path):
    radius = world['robot_radius']+world['planning_margin']
    # 先做可见性简化，再进行Chaikin角点切割；每一轮重新验证整条曲线的折线近似。
    simplified, i = [path[0]],0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not segment_safe(world,path[i],path[j],radius+0.025):
            j -= 1
        simplified.append(path[j])
        i = j
    result = np.asarray(simplified)
    for _ in range(4):
        candidate = [result[0]]
        for a,b in zip(result,result[1:]):
            candidate.extend([0.75*a+0.25*b,0.25*a+0.75*b])
        candidate.append(result[-1])
        candidate = np.asarray(candidate)
        if not path_safe(world,candidate,radius):
            break
        result = candidate
    return result


def plan(world, start=None, goal=None):
    raw = astar(world, world['start'] if start is None else start,
                world['goal'] if goal is None else goal)
    return raw,smooth_path(world,raw)


def body_velocity(world_velocity, yaw):
    x,y = world_velocity
    return (math.cos(yaw)*x+math.sin(yaw)*y,
            -math.sin(yaw)*x+math.cos(yaw)*y)
