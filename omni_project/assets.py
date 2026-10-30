"""从同一地图配置生成SDF世界、已知地图和离线路径预览。"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .planning import clearance, load_world, plan


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent,tag,attrs)
    if text is not None:
        child.text = str(text)
    return child


def geometry(parent, kind, dimensions):
    g = element(parent,'geometry')
    shape = element(g,kind)
    if kind == 'box':
        element(shape,'size',' '.join(map(str,dimensions)))
    elif kind == 'cylinder':
        element(shape,'radius',dimensions[0]); element(shape,'length',dimensions[1])
    else:
        element(shape,'radius',dimensions[0])


def shape(link, name, kind, dimensions, pose, color, collision=True, slide=False):
    if collision:
        c = element(link,'collision',name=name+'_collision')
        element(c,'pose',pose); geometry(c,kind,dimensions)
        if slide:
            ode = element(element(element(c,'surface'),'friction'),'ode')
            element(ode,'mu',0.0); element(ode,'mu2',0.0)
    v = element(link,'visual',name=name+'_visual')
    element(v,'pose',pose); geometry(v,kind,dimensions)
    material = element(v,'material')
    element(material,'ambient',color); element(material,'diffuse',color)


def make_world(world, path):
    sdf = ET.Element('sdf',{'version':'1.8'})
    root = element(sdf,'world',name='recruitment')
    physics = element(root,'physics',name='physics',type='ignored')
    element(physics,'max_step_size',0.001); element(physics,'real_time_factor',1.0)
    for lib, cls in [('physics','Physics'),('user-commands','UserCommands'),
                     ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        p = element(root,'plugin',filename='ignition-gazebo-'+lib+'-system',
                    name='ignition::gazebo::systems::'+cls)
        if cls == 'Sensors': element(p,'render_engine','ogre2')
    light = element(root,'light',name='sun',type='directional')
    element(light,'pose','0 0 8 0 0 0'); element(light,'diffuse','0.8 0.8 0.8 1')
    element(light,'direction','-0.3 0.2 -0.9')
    floor = element(root,'model',name='floor'); element(floor,'static','true')
    link = element(floor,'link',name='link')
    shape(link,'floor','box',[12,10,0.1],'0 0 -0.05 0 0 0','0.8 0.8 0.8 1')
    xmin,ymin,xmax,ymax = world['bounds']
    obstacles = list(world['obstacles']) + [
        {'name':'west','kind':'box','x':xmin-0.075,'y':(ymin+ymax)/2,'sx':0.15,'sy':ymax-ymin+0.3},
        {'name':'east','kind':'box','x':xmax+0.075,'y':(ymin+ymax)/2,'sx':0.15,'sy':ymax-ymin+0.3},
        {'name':'south','kind':'box','x':(xmin+xmax)/2,'y':ymin-0.075,'sx':xmax-xmin,'sy':0.15},
        {'name':'north','kind':'box','x':(xmin+xmax)/2,'y':ymax+0.075,'sx':xmax-xmin,'sy':0.15}]
    for o in obstacles:
        model = element(root,'model',name=o['name']); element(model,'static','true')
        element(model,'pose',f"{o['x']} {o['y']} 0.4 0 0 0")
        link = element(model,'link',name='link')
        dims = [o['radius'],0.8] if o['kind']=='cylinder' else [o['sx'],o['sy'],0.8]
        shape(link,'body',o['kind'],dims,'0 0 0 0 0 0','0.7 0.25 0.15 1')
    robot = element(root,'model',name='omni_car')
    element(robot,'pose',f"{world['start'][0]} {world['start'][1]} 0.15 0 0 0")
    link = element(robot,'link',name='base_link')
    inertial = element(link,'inertial'); element(inertial,'mass',5.0)
    inertia = element(inertial,'inertia')
    for key,value in [('ixx',0.065),('iyy',0.098),('izz',0.135)]: element(inertia,key,value)
    shape(link,'body','box',[0.45,0.35,0.18],'0 0 0 0 0 0','0.1 0.4 0.9 1')
    for i,(x,y) in enumerate([(-0.16,-0.12),(-0.16,0.12),(0.16,-0.12),(0.16,0.12)]):
        # 理想全向模型的球形支撑；不是麦克纳姆轮动力学仿真。
        shape(link,'roller'+str(i),'sphere',[0.06],f'{x} {y} -0.09 0 0 0','0.1 0.1 0.1 1',slide=True)
    shape(link,'laser','cylinder',[0.04,0.05],'0 0 0.18 0 0 0','0.1 0.1 0.1 1',collision=False)
    sensor = element(link,'sensor',name='lidar',type='gpu_lidar')
    element(sensor,'pose','0 0 0.18 0 0 0'); element(sensor,'topic','/lidar')
    element(sensor,'update_rate',10); element(sensor,'always_on','true')
    lidar = element(sensor,'lidar'); horizontal = element(element(lidar,'scan'),'horizontal')
    for k,v in [('samples',360),('resolution',1),('min_angle',-3.14159265),('max_angle',3.14159265)]: element(horizontal,k,v)
    rg = element(lidar,'range')
    for k,v in [('min',0.08),('max',10.0),('resolution',0.01)]: element(rg,k,v)
    p = element(robot,'plugin',filename='ignition-gazebo-velocity-control-system',name='ignition::gazebo::systems::VelocityControl')
    element(p,'topic','/model/omni_car/cmd_vel')
    p = element(robot,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
    for k,v in [('odom_frame','odom'),('robot_base_frame','base_link'),('dimensions',3),
                ('odom_topic','/model/omni_car/odometry'),('odom_publish_frequency',50)]: element(p,k,v)
    ET.indent(sdf)
    Path(path).write_text(ET.tostring(sdf,encoding='unicode'),encoding='utf-8')


def generate(config, output):
    world = load_world(config)
    output = Path(output); output.mkdir(parents=True,exist_ok=True)
    make_world(world,output/'arena.sdf')
    xmin,ymin,xmax,ymax = world['bounds']; res = world['resolution']
    # 地图比墙体外缘稍大，避免边界墙刚好落在图片外。
    origin = [xmin-0.2,ymin-0.2]
    width,height = round((xmax-xmin+0.4)/res),round((ymax-ymin+0.4)/res)
    pixels = np.full((height,width),254,np.uint8)
    for row in range(height):
        for col in range(width):
            p = [origin[0]+(col+0.5)*res,origin[1]+(height-row-0.5)*res]
            if clearance(world,p) <= res*0.71: pixels[row,col] = 0
    with (output/'known_map.pgm').open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode()); f.write(pixels.tobytes())
    (output/'known_map.yaml').write_text(
        f'image: known_map.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',encoding='utf-8')
    raw,smooth = plan(world)
    import json
    (output/'planned_path.json').write_text(json.dumps({'raw':raw.tolist(),'smooth':smooth.tolist()},indent=2),encoding='utf-8')
    # 可用浏览器查看的矢量预览；非实时仿真证据。
    def coords(points): return ' '.join(f'{(p[0]-xmin)*80:.2f},{(ymax-p[1])*80:.2f}' for p in points)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480" viewBox="0 0 640 480"><rect width="640" height="480" fill="white" stroke="black"/>']
    for o in world['obstacles']:
        if o['kind']=='cylinder':
            svg.append(f'<circle cx="{(o["x"]-xmin)*80}" cy="{(ymax-o["y"])*80}" r="{o["radius"]*80}" fill="#d98655"/>')
        else:
            svg.append(f'<rect x="{(o["x"]-o["sx"]/2-xmin)*80}" y="{(ymax-o["y"]-o["sy"]/2)*80}" width="{o["sx"]*80}" height="{o["sy"]*80}" fill="#d98655"/>')
    svg.append(f'<polyline points="{coords(raw)}" fill="none" stroke="#aaa" stroke-width="2"/>')
    svg.append(f'<polyline points="{coords(smooth)}" fill="none" stroke="#2266cc" stroke-width="3"/>')
    for name,p in [('A',world['start']),('B',world['goal'])]:
        x,y=(p[0]-xmin)*80,(ymax-p[1])*80
        svg.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#228844"/><text x="{x+8}" y="{y}">{name}</text>')
    svg.append('</svg>'); (output/'path_preview.svg').write_text(''.join(svg),encoding='utf-8')
    return world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config',default='config/world.json')
    parser.add_argument('--output',default='generated')
    args = parser.parse_args()
    generate(args.config,args.output)
    print('已生成世界、已知地图和离线路径预览；这不是SLAM地图。')


if __name__ == '__main__': main()
