# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.3
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0]


def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while (1):
            '''if position_estimate[0] > BOX_LIMIT:
                mc.start_back()
            elif position_estimate[0] < -BOX_LIMIT:
                mc.start_forward()
            '''

            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(3)
        mc.stop()

def move_linear_simple_2(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(5)
        mc.forward(50)
        time.sleep(5)
        mc.forward(50)
        time.sleep(5)
        mc.back(50)
        time.sleep(5)
        mc.left(50)
        time.sleep(5)
        mc.right(50)
        time.sleep(5)
        mc.stop()

def move_linear_simple_3(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(10)
        mc.stop()

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def prettyPrint(data):
    for key, value in data.items():
        print(value, end=',')
    print("")

def log_pos_callback(timestamp, data, logconf):
    prettyPrint(data)
    #global position_estimate
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[1] = data['stateEstimate.y']
    #position_estimate[2] = data['stateEstimate.z']


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimateZ.x', 'int16_t')
        logconf.add_variable('stateEstimateZ.y', 'int16_t')
        logconf.add_variable('stateEstimateZ.z', 'int16_t')
        logconf.add_variable('stateEstimateZ.vx', 'int16_t')
        logconf.add_variable('stateEstimateZ.vy', 'int16_t')
        logconf.add_variable('stateEstimateZ.vz', 'int16_t')
        #logconf.add_variable('stateEstimateZ.ax', 'int16_t')
        #logconf.add_variable('stateEstimateZ.ay', 'int16_t')
        #logconf.add_variable('stateEstimateZ.az', 'int16_t')
        logconf.add_variable('stabilizer.thrust', 'float')
        #logconf.add_variable('debug.error', 'float')
        #logconf.add_variable('debug.integral', 'float')
        logconf.add_variable('controller.pitch', 'float')
        logconf.add_variable('controller.roll', 'float')

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        for var in logconf.variables:
            print(var.name, end=',')
        print("")

        logconf.start()

        # take_off_simple(scf)
        # move_linear_simple(scf)
        move_linear_simple_2(scf)
        # move_linear_simple_3(scf)
        # move_box_limit(scf)
        # logconf.stop()
