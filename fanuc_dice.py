import time
from time import sleep
from robot_controller import robot
import json
import random
from datetime import datetime

import numpy as np
import paho.mqtt.client as mqtt


# DJ is the name of my robot
# DJ's IP is 10.8.4.16

drive_path_DJ = '10.8.4.16' #DJ's ip

robotDJ = robot(drive_path_DJ)
# robotDJ.set_speed(300)


def DJ_bulldozer_pull(x, y, z, w, p, r):
    bulldoze1_up = [x-100, y-100, z+100, w, p, r]
    bulldoze1 = [x-100, y-100, z-30, w, p, r]
    bulldoze2_up = [x+100, y+100, z+100, w, p, r]
    bulldoze2 = [x+100, y+100, z-30, w, p, r]

    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(bulldoze2_up)
    robotDJ.write_cartesian_position(bulldoze2)
    robotDJ.write_cartesian_position(bulldoze1)
    robotDJ.write_cartesian_position(bulldoze1_up)

def DJ_bulldozer_push(x, y, z, w, p, r):
    bulldoze1_up = [x-100, y-100, z+100, w, p, r]
    bulldoze1 = [x-100, y-100, z-30, w, p, r]
    bulldoze2_up = [x+100, y+100, z+100, w, p, r]
    bulldoze2 = [x+100, y+100, z-30, w, p, r]

    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(bulldoze1_up)
    robotDJ.write_cartesian_position(bulldoze1)
    robotDJ.write_cartesian_position(bulldoze2)
    robotDJ.write_cartesian_position(bulldoze2_up)

def DJ_home():
    robotDJ = robot(drive_path_DJ)
    home = [540.0, -150.0, 550.0, -179.0, 0.01, 30.0]

    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(home)

def DJ_pickup(z):
    pickup = [625.0, -10.0, z, -179.9, 0.0, 30.0]
    pickup_up = [630.0, 0.0, z+200, -179.9, 0.0, 30.0]

    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(pickup_up)
    robotDJ.write_cartesian_position(pickup)
    robotDJ.schunk_gripper('close')
    robotDJ.write_cartesian_position(pickup_up)

def DJ_putdown(z):
    putdown = [625.0, -10.0, z, -179.9, 0.0, 30.0]
    putdown_up = [630.0, 0.0, z+200, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(putdown_up)
    robotDJ.write_cartesian_position(putdown)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(putdown_up)

def DJ_top_left(x,y):
    top_left_up = [x, y, 330.0, -179.9, 0.0, 30.0]
    top_left = [x, y, 125.0, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(top_left_up)
    robotDJ.write_cartesian_position(top_left)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(top_left_up)

def DJ_top_right(x,y):
    top_right_up = [x, y, 330.0, -179.9, 0.0, 30.0]
    top_right = [x, y, 125.0, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(top_right_up)
    robotDJ.write_cartesian_position(top_right)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(top_right_up)

def DJ_open():
    robotDJ.schunk_gripper('open')

def DJ_bottom_right(x,y):
    bottom_right_up = [x, y, 265.0, -179.9, 0.0, 30.0]
    bottom_right = [x, y, 125.0, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(bottom_right_up)
    robotDJ.write_cartesian_position(bottom_right)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(bottom_right_up)

def DJ_bottom_left(x,y):
    bottom_left_up = [x, y, 330.0, -179.9, 0.0, 30.0]
    bottom_left = [x, y, 125.0, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(bottom_left_up)
    robotDJ.write_cartesian_position(bottom_left)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(bottom_left_up)

def DJ_rando(x,y):
    rando_up = [x, y, 330.0, -179.9, 0.0, 30.0]
    rando = [x, y, 125.0, -179.9, 0.0, 30.0]

    robotDJ.write_cartesian_position(rando_up)
    robotDJ.write_cartesian_position(rando)
    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(rando_up)

def DJ_to_dice(x,y,z,w,p,r):
    to_dice_up = [x, y+8, z+50, w, p, r]
    to_dice = [x,y,z,w,p,r]

    robotDJ.schunk_gripper('open')
    robotDJ.write_cartesian_position(to_dice_up)
    robotDJ.write_cartesian_position(to_dice)
    robotDJ.schunk_gripper('close')
    robotDJ.write_cartesian_position(to_dice_up)

# def main():
#     DJ_home()
#     DJ_pickup(65.0)
#     DJ_top_left()
#     DJ_pickup(65.0)
#     DJ_top_right()
#     DJ_pickup(65.0)
#     DJ_bottom_left()
#     DJ_pickup(65.0)
#     DJ_bottom_right()
#     DJ_home()

# if __name__ == "__main__":
#     main()

