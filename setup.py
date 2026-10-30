from glob import glob
from setuptools import setup

setup(name='omni_project',version='0.1.0',packages=['omni_project'],
      data_files=[('share/ament_index/resource_index/packages',['resource/omni_project']),
                  ('share/omni_project',['package.xml']),
                  ('share/omni_project/config',glob('config/*')),
                  ('share/omni_project/launch',glob('launch/*.py'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='wch',maintainer_email='wch@example.com',
      description='Humble Fortress 全向车、路径规划、SLAM和导航实训',license='Apache-2.0',
      entry_points={'console_scripts':[
          'runtime = omni_project.runtime:main',
          'teleop = omni_project.teleop:main',
          'follow = omni_project.follow:main',
          'navigate = omni_project.navigation_goal:main',
          'generate = omni_project.assets:main']})
