#!/usr/bin/env python3
"""Replace the placeholder person/backpack in patrol_world.sdf with Gazebo Fuel models.

Positions come from ground_truth.csv (next to the world file), so the map stays valid.
Environment: PERSON_NAME, BACKPACK_NAME, PERSON_YAW, BACKPACK_YAW.
"""
import csv
import os
import re
import shutil
import sys

world = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                           else '~/patrol_ws/src/patrol_sim/worlds/patrol_world.sdf')
person = os.environ.get('PERSON_NAME', 'Standing person')
backpack = os.environ.get('BACKPACK_NAME', 'Backpack')
yaw = {'target_person_1': float(os.environ.get('PERSON_YAW', '1.2')),
       'target_backpack_1': float(os.environ.get('BACKPACK_YAW', '0.0'))}
uri = {'target_person_1': person, 'target_backpack_1': backpack}
BASE = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'

pos = {}
with open(os.path.join(os.path.dirname(world), 'ground_truth.csv')) as f:
    for row in csv.DictReader(f):
        pos[row['name']] = (row['x'], row['y'])

text = open(world).read()
bak = world + '.placeholder.bak'
if not os.path.exists(bak):
    shutil.copy(world, bak)

for name in ('target_person_1', 'target_backpack_1'):
    x, y = pos[name]
    block = (f'    <include>\n      <name>{name}</name>\n'
             f'      <uri>{BASE}{uri[name]}</uri>\n'
             f'      <pose>{x} {y} 0 0 0 {yaw[name]}</pose>\n'
             f'      <static>true</static>\n    </include>')
    pattern = (r'<model name="' + name + r'">.*?</model>|<include>\s*<name>' + name
               + r'</name>.*?</include>')
    text, n = re.subn(pattern, block.strip(), text, count=1, flags=re.S)
    if n != 1:
        sys.exit(f'Could not find {name} in {world}')
    print(f'{name}: {uri[name]} at ({x}, {y}), yaw {yaw[name]}')

open(world, 'w').write(text)
print('World updated. Original saved as', bak)
