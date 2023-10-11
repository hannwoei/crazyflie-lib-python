# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
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
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
dancing swarm aero 2023
"""
import math
import time

import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

class Uploader:
    def __init__(self):
        self._is_done = False
        self._sucess = True

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done,
                                  write_failed_cb=self._upload_failed)

        while not self._is_done:
            time.sleep(0.2)

        return self._sucess

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True
        self._sucess = True

    def _upload_failed(self, mem, addr):
        print('Data upload failed')
        self._is_done = True
        self._sucess = False

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

circle =  [
    [1.51717,0,0,0,0,1.32574,-1.98833,1.05213,-0.193077,0,0,0,0,0,0,0,0,0,0,0,0,0.535169,-0.659232,0.301615,-0.0494349,0,0,0,0,0,0,0,0],
    [1.50824,0.3,0.106933,-0.103026,-0.0410602,-0.209442,0.212167,-0.0626071,0.00443994,0,0,0,0,0,0,0,0,0.3,0.33848,-0.00502046,-0.0275498,0.0241283,-0.151489,0.124218,-0.0286872,0,0,0,0,0,0,0,0],
    [1.50824,0,-0.43482,-2.9983e-14,0.029862,-0.254959,0.663639,-0.450116,0.0952838,0,0,0,0,0,0,0,0,0.6,4.96303e-14,-0.0577895,1.276e-14,-0.299076,0.345641,-0.143107,0.0206168,0,0,0,0,0,0,0,0],
    [1.51717,-0.3,0.213865,0.103026,-0.0410602,0.417628,-0.835528,0.519489,-0.105396,0,0,0,0,0,0,0,0,0.3,-0.33848,-0.01005,-0.0275498,0.0730012,0.130773,-0.146042,0.0365326,0,0,0,0,0,0,0,0],
]

small_circle =  [
    [1.27649,0,0,0,0,1.92865,-3.47886,2.20546,-0.483774,0,0,0,0,0,0,0,0,0,0,0,0,0.69662,-1.01417,0.549531,-0.106795,0,0,0,0,0,0,0,0],
    [1.22015,0.2,0.0606813,-0.100431,-0.0484002,-0.127439,0.0464536,0.0845032,-0.0393948,0,0,0,0,0,0,0,0,0.2,0.272864,0.000809798,-0.0324747,0.0658206,-0.360035,0.346088,-0.0967091,0,0,0,0,0,0,0,0],
    [1.22015,0,-0.350529,-9.67329e-15,0.0352001,-0.461004,1.39941,-1.1527,0.299203,0,0,0,0,0,0,0,0,0.4,-1.22796e-15,-0.0563338,5.23403e-15,-0.505617,0.737984,-0.387096,0.0710504,0,0,0,0,0,0,0,0],
    [1.27649,-0.2,0.172407,0.100431,-0.0484002,0.514023,-1.24746,0.930469,-0.225515,0,0,0,0,0,0,0,0,0.2,-0.272864,-0.0016548,-0.0324747,0.0991706,0.196796,-0.265897,0.0794233,0,0,0,0,0,0,0,0],

]

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = Uploader().upload(trajectory_mem)
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration

# _______________________________
# Essential Functions
# _______________________________

# TAKE OFF
DEFAULT_HEIGHT = 0.5
SPACING = 0.3
INIT_SPACING = 0.6
SHIFT_X = -INIT_SPACING
SHIFT_Y = INIT_SPACING
def take_off(scf, params):
    cf = scf.cf
    commander = cf.high_level_commander
    d = params['d']

    # Take off
    commander.takeoff(DEFAULT_HEIGHT, 2.0)
    time.sleep(3.0)

    # Go to start location
    commander.go_to(SHIFT_X + SPACING * (d-1), SHIFT_Y + 0.1 -(SPACING* (d-1)), DEFAULT_HEIGHT , 0.0 ,2.0)
    time.sleep(3.0)
 
#     time.sleep(7.0)
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()
    
# Do a Loop
LOOPS = 3
def run_v_circle_aero(scf, params):
    cf = scf.cf

    d = params['d']

    commander = cf.high_level_commander
    trajectory_id = 1

    # Take off
    duration = upload_trajectory(cf, trajectory_id, small_circle)
    relative = True

    # Delay based on ID
    time.sleep((d-1)*2)

    # Execute circle trajectory
    for t in range(0, LOOPS):
        commander.start_trajectory(trajectory_id, 1.0, relative)
        time.sleep(duration) 
    time.sleep(8-(d-1)*2) # change '8' so that the last ID does not need to wait

    # Go back to initial position and land again
    commander.go_to(SHIFT_X + SPACING * (d-1), SHIFT_Y -(SPACING* (d-1)),DEFAULT_HEIGHT , 0.0 ,2.0)
    time.sleep(2.0)
    if d == 1:
        commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0)
    if d == 2:
        commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) 
    if d == 3:
        commander.go_to(0, 0 ,DEFAULT_HEIGHT , 0.0 ,2.0)
    if d == 4:
        commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0)        
    if d == 5:
        commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0)

    time.sleep(5.0)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

# test
# def run_shared_sequence_test(scf, params):
#     cf = scf.cf
# 
#     d = params['d']
# 
#     commander = cf.high_level_commander
# 
#     time.sleep(5.0)
#     if d == 1:
# #         commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*7 ,2.0) # URI4
#         commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4 ,2.0) # URI1
#     if d == 2:
# #         commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*3 ,2.0) # URI2
#         commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*5 ,2.0) # URI5
# 
#     time.sleep(10.0)
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()    

def run_h_circle_aero(scf, params):
    cf = scf.cf

    d = params['d']
    c = params['c']
    z = params['z']
       
    # Number of setpoints sent per second
    fs = 10
    fsi = 1.0 / fs

    # Compensation for unknown error :-(
    comp = 1.0 # no compensation
    
    commander = cf.high_level_commander

    # 1st circle
    time.sleep(5.0)
    if d == 1:
        commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4 ,2.0) # URI1
    if d == 2:
        commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*3 ,2.0) # URI2
    if d == 3:
        commander.go_to(0.0, 0.0 ,DEFAULT_HEIGHT+0.2 , 0.0 ,2.0) # URI3
    if d == 4:
        commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*7 ,2.0) # URI4
    if d == 5:
        commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*5 ,2.0) # URI5
       
    time.sleep(5.0)
    for _ in range(2):
        # The time for one revolution
        circle_time = 10 # 8
        steps = circle_time * fs
        for _ in range(steps):
            cf.commander.send_hover_setpoint(c * comp * math.pi / circle_time,
                                             0, 360.0 / circle_time, z)
            time.sleep(fsi)
    
    if d == 1:
        commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI1
    if d == 2:
        commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI2
    if d == 3:
        commander.go_to(0.0, 0.0 ,DEFAULT_HEIGHT+0.2 , 0.0 ,2.0) # URI3
    if d == 4:
        commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI4
    if d == 5:
        commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI5
                    
     # 2nd circle
    time.sleep(3.0)
    if d == 1:
        commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*5 ,2.0) # URI1
    if d == 2:
        commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*7 ,2.0) # URI2
    if d == 3:
        commander.go_to(0.0, 0.0 ,DEFAULT_HEIGHT+0.2 , 0.0 ,2.0) # URI3
    if d == 4:
        commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4*3 ,2.0) # URI4
    if d == 5:
        commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , np.pi/4 ,2.0) # URI5
       
    time.sleep(3.0)
    for _ in range(2):
        # The time for one revolution
        circle_time = 10
        steps = circle_time * fs
        for _ in range(steps):
            cf.commander.send_hover_setpoint(c * comp * math.pi / circle_time,
                                             0, -360.0 / circle_time, z)
            time.sleep(fsi)
    
    if d == 1:
        commander.go_to(-INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI1
    if d == 2:
        commander.go_to(-INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI2
    if d == 3:
        commander.go_to(0.0, 0.0 ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI3
    if d == 4:
        commander.go_to(INIT_SPACING, INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI4
    if d == 5:
        commander.go_to(INIT_SPACING, -INIT_SPACING ,DEFAULT_HEIGHT , 0.0 ,2.0) # URI5
                   
    time.sleep(5.0)
       
    commander.go_to(SHIFT_X + SPACING * (d-1), SHIFT_Y -(SPACING* (d-1)),DEFAULT_HEIGHT , 0.0 ,2.0)
    time.sleep(5.0)      
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()     

def run_square_aero(scf):
    activate_mellinger_controller(scf, False)

    box_size = 0.5
    flight_time = 1.2

    commander = scf.cf.high_level_commander

    commander.takeoff(DEFAULT_HEIGHT, 2.0)
    time.sleep(3)

    # 1st square
    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    # 2nd square
#     time.sleep(2)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)
    
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()
  
# URIS of swarm
uris = {
    'radio://0/80/2M/E7E7E7E712',
    'radio://0/80/2M/E7E7E7E713',
    'radio://0/80/2M/E7E7E7E714',
    'radio://0/80/2M/E7E7E7E715',
    'radio://0/80/2M/E7E7E7E716',
    # Add more URIs if you want more copters in the swarm
}

# Parameters of Swarm
params = {
    'radio://0/80/2M/E7E7E7E712': [{'d': 1, 'c': 1.7, 'z': 0.5}],
    'radio://0/80/2M/E7E7E7E713': [{'d': 2, 'c': 1.7, 'z': 0.5}],
    'radio://0/80/2M/E7E7E7E714': [{'d': 3, 'c': 0.0, 'z': 0.7}],
    'radio://0/80/2M/E7E7E7E715': [{'d': 4, 'c': 1.7, 'z': 0.5}],
    'radio://0/80/2M/E7E7E7E716': [{'d': 5, 'c': 1.7, 'z': 0.5}],
}


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        # Activate HL commander and reset estimator
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
#         swarm.parallel_safe(take_off, args_dict=params)
#         input("enter to continue")
        
        # square
        swarm.parallel_safe(run_square_aero)
        
        # circle - horizontal
        swarm.parallel_safe(run_h_circle_aero, args_dict=params)
        
        # circle - vertical
        swarm.parallel_safe(run_v_circle_aero, args_dict=params)
