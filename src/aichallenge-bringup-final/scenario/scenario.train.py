#!/usr/bin/env python3

import os
import lgsvl
import math
import copy
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--host",help='simulator execute pc\'s ip address / --host 192.168.0.1',default='127.0.0.1')
parser.add_argument("--bridge",help='bridge execute pc\'s ip address / --bridge 192.168.0.1',default='127.0.0.1')
args = parser.parse_args()

sim = lgsvl.Simulator(args.host, 8181)

scene_name = "Borregas Avenue"
vehicle_name = "Lexus"

if sim.current_scene == scene_name:
    sim.reset()
else:
    sim.load(scene_name)
spawns = sim.get_spawn()


def project(point):
    # project the point to ground surface
    layer_mask = 0
    layer_mask |= 1 << 0  # 0 is the layer for the road (default)
    hit = sim.raycast(point, lgsvl.Vector(0, -1, 0), layer_mask)
    return copy.deepcopy(hit.point)

def add_ego_car():
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])
    state = lgsvl.AgentState()
    state.transform = spawns[0]
    a = sim.add_agent(vehicle_name,
                      lgsvl.AgentType.EGO, state)

    a.connect_bridge(args.bridge, 9090)
    print("Waiting for connection...")
    while not a.bridge_connected:
        time.sleep(1)
    print("Bridge connected:", a.bridge_connected)

    return a

def get_ground_point(spawn_id, offset):
    forward_offset, right_offset = offset

    forward = lgsvl.utils.transform_to_forward(spawns[spawn_id])
    right = lgsvl.utils.transform_to_right(spawns[spawn_id])
    up = lgsvl.utils.transform_to_up(spawns[spawn_id])

    return project(spawns[spawn_id].position + forward_offset * forward + right_offset * right + 5 * up)

def get_angle(spawn_id, offset):
    rotation = spawns[spawn_id].rotation
    angle = copy.deepcopy(rotation)
    angle.y += offset
    return angle

def add_line_loop_car_with_trigger(car_type, spawn_id, offsets, trigger_distance, delay, angle_offset, speed, no_loop=False):
    """
    offset is (forward, right)
    1. wait at offsets[0] to be triggered.
    2. visit offsets[0] -> offsets[n-1] accordingly
    3. back to the offsets[0] through the underground of offsets[n-1] -> that of offset[0]
    4. play again from 2
    """
    rotation = spawns[spawn_id].rotation
    angle = copy.deepcopy(rotation)
    angle.y += angle_offset

    speed_invisible = 50
    underground_offset = 3
    up = lgsvl.utils.transform_to_up(spawns[spawn_id])
    wait = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[0]), speed, angle, delay, 0, trigger_distance),
    ]
    forward = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offset), speed, angle, 0) for offset in offsets
    ]
    backward = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[-1]) - underground_offset * up, speed_invisible, angle, 0),
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[0]) - underground_offset * up, speed_invisible, angle, 0),
    ]
    if no_loop:
        waypoints = wait + forward + backward * 100
    else:
        loop = forward + backward
        N = 50  # the number of loop iteration
        waypoints = wait + loop * N

    state = lgsvl.AgentState()
    state.transform.position = get_ground_point(spawn_id, offsets[0])
    state.transform.rotation = angle
    npc = sim.add_agent(car_type, lgsvl.AgentType.NPC, state)
    npc.follow(waypoints, loop=False)

def add_line_loop_car_with_underground_trigger(car_type, spawn_id, trigger_offset, offsets, trigger_distance, delay, angle_offset, speed, no_loop=False):
    """
    offset is (forward, right)
    1. wait underground of trigger_offsets to be triggered.
    2. visit offsets[0] -> offsets[n-1] accordingly
    3. back to the offsets[0] through the underground of offsets[n-1] -> that of offset[0]
    4. play again from 2
    """
    rotation = spawns[spawn_id].rotation
    angle = copy.deepcopy(rotation)
    angle.y += angle_offset

    speed_invisible = 100
    underground_offset = 3
    up = lgsvl.utils.transform_to_up(spawns[spawn_id])
    underground = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, trigger_offset) - underground_offset * up, speed_invisible, angle, 0, 0, trigger_distance),
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[0])     - underground_offset * up, speed_invisible, angle, delay),
    ]
    forward = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offset), speed, angle, 0) for offset in offsets
    ]
    backward = [
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[-1]) - underground_offset * up, speed_invisible, angle, 0),
        lgsvl.DriveWaypoint(get_ground_point(spawn_id, offsets[0]) - underground_offset * up, speed_invisible, angle, 0),
    ]
    if no_loop:
        waypoints = underground + forward + backward * 100
    else:
        loop = forward + backward
        N = 50  # the number of loop iteration
        waypoints = underground + loop * N

    state = lgsvl.AgentState()
    state.transform.position = get_ground_point(spawn_id, trigger_offset) - underground_offset * up
    state.transform.rotation = angle
    npc = sim.add_agent(car_type, lgsvl.AgentType.NPC, state)
    npc.follow(waypoints, loop=False)

def add_static_car(i, offset_forward, offset_right, offset_angle, debug=False):
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[i])
    right = lgsvl.utils.transform_to_right(spawns[i])
    up = lgsvl.utils.transform_to_up(spawns[i])

    state = lgsvl.AgentState()
    state.transform.position = project(spawns[i].position + offset_forward * forward + offset_right * right + 5 * up)
    angle = copy.deepcopy(spawns[i].rotation)
    angle.y += offset_angle
    state.transform.rotation = angle
    if debug:
        npc = sim.add_agent("Lincoln2017MKZ (Apollo 5.0)", lgsvl.AgentType.EGO, state)
    else:
        npc = sim.add_agent("Sedan", lgsvl.AgentType.NPC, state)

def add_pedestrian_with_trigger(agent_type, offset, timing):
    i = 0
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[i])
    right = lgsvl.utils.transform_to_right(spawns[i])
    up = lgsvl.utils.transform_to_up(spawns[i])
    wp = [
        lgsvl.WalkWaypoint(project(spawns[0].position + 5 * right + offset * forward + 2 * up), 0, 100),
        lgsvl.WalkWaypoint(project(spawns[0].position + 5 * right + offset * forward + 2 * up), timing, 0),
        lgsvl.WalkWaypoint(project(spawns[0].position - 5 * right + offset * forward + 2 * up), 0),
        lgsvl.WalkWaypoint(project(spawns[0].position - 11 * right + offset * forward + 2 * up), 0),
    ]
    state = lgsvl.AgentState()
    state.transform = copy.deepcopy(spawns[0])
    state.transform.position = copy.deepcopy(wp[0].position)
    p = sim.add_agent(agent_type, lgsvl.AgentType.PEDESTRIAN, state)
    p.follow(wp, False)
    # def on_waypoint(agent, index):
          # print("waypoint {} reached".format(index))
    # p.on_waypoint_reached(on_waypoint)

def change_all_signals_green():
    controllables = sim.get_controllables("signal")
    for c in controllables:
        c.control('green=15;loop')

### main ####
scenario_id = 'train'

# 自車位置、信号機は同じです。
add_ego_car()
change_all_signals_green()
# car1
add_line_loop_car_with_trigger('Sedan', 1, [(165, 8), (250, 12)], 80, 0, angle_offset=0, speed=8)
add_line_loop_car_with_trigger('SUV', 1, [(165, 8), (250, 12)], 80, 3, angle_offset=0, speed=8)
# car2
add_line_loop_car_with_trigger('Hatchback', 1, [(85, 5), (160, 8)], 100, 0, angle_offset=0, speed=8)
add_line_loop_car_with_trigger('Sedan', 1, [(85, 5), (160, 8)], 100, 5, angle_offset=0, speed=8)
# car3
add_line_loop_car_with_underground_trigger('SUV', 0, (328, -25), [(250, -6), (-5, -6)], 5, 0, angle_offset=180, speed=4)
add_line_loop_car_with_underground_trigger('SUV', 0, (328, -25), [(0, 0), (300, 0)], 5, 5, angle_offset=0, speed=10, no_loop=True)
add_line_loop_car_with_underground_trigger('Sedan', 0, (328, -25), [(100, 0), (300, 0)], 5, 5, angle_offset=0, speed=10, no_loop=True)
add_line_loop_car_with_underground_trigger('Hatchback', 0, (328, -25), [(200, 0), (300, 0)], 5, 5, angle_offset=0, speed=10, no_loop=True)
# car4
add_line_loop_car_with_underground_trigger('Sedan', 0, (90, -6), [(52, 40), (52, -60)], 5, 0, angle_offset=270, speed=10)
add_line_loop_car_with_underground_trigger('SUV', 0, (90, -6), [(40, -60), (40, 40)], 5, 1, angle_offset=90, speed=10)
# pedestrians
add_pedestrian_with_trigger('Bob', 115, 10)
add_pedestrian_with_trigger('Howard', 110, 5)
add_pedestrian_with_trigger('Stephen', 105, 0)
# 駐車場敷地内の車
add_static_car(1, 158, 24.5, 90)
add_static_car(1, 180, 35, 270)
add_static_car(1, 196, 50, 180)
# 静止車両
add_static_car(0, 200, -1.5, 0)
add_static_car(0, 220, -1.5, 0)
### main end ####

sim.run(time_limit = 300.0)
sim.remote.command("simulator/push_message", {"message": "publish_timersensor"})
