from email.mime import image
from callibration import connect_camera, detect_dice_clumps, process_callibration_image, process_image, image_to_robot_coords, fanuc_to_full_pose, get_calibration_coords, compute_affine_matrix
import cv2 as cv
import numpy as np
from standardbots import StandardBotsRobot, models
import math
from fanuc_dice import  DJ_bulldozer_pull, DJ_bulldozer_push, DJ_home, DJ_to_dice, DJ_putdown

import paho.mqtt.client as mqtt
import json, time

#BROKER = "192.168.1.11"
BROKER = "10.8.4.28"
TOPIC = "robot/chat"
CLIENT_ID = "robot1_master"
STATE = 0
MAX_STATE = 4
FINISHED = False

def on_connect(client, userdata, flags, rc):
    print("Master connected.")
    client.subscribe(TOPIC)
    # Always announce current state
    client.publish(TOPIC, json.dumps({"sender": CLIENT_ID, "state": STATE}))
    print(f"Master state {STATE} announced.")

def on_message(client, userdata, msg):
    global STATE, FINISHED
    data = json.loads(msg.payload.decode())
    sender = data.get("sender")
    s = data.get("state", 0)
    stop = data.get("stop", False)

    if sender == CLIENT_ID or FINISHED:
        return

    if stop:
        print("Master received stop. Disconnecting.")
        FINISHED = True
        client.disconnect()
        return

    # When slave matches, advance
    if s == STATE:
        STATE += 1

        if STATE == 1:
            # input("Robot 1 finished bulldozing, Press Enter to finish state 1...")
            print("Robot 1 finished bulldozing, Press Enter to finish state 1...")
            # put logic to take picture and bulldoze
            DJ_home()

            num_clumps = 1
            push_pull_count = 0

            while num_clumps >= 1:

                image_path = connect_camera()
                clumps = detect_dice_clumps(image_path)
                robot_coords = image_to_robot_coords(clumps)
                fanuc_coords = robot_coords[0]

                num_clumps = len(clumps)
                if num_clumps == 0:
                    print('--------------------------------------------')
                    print('no clumps found, moving on to pick-up')
                    print('--------------------------------------------')
                else:
                    print('--------------------------------------------')
                    print('Clumps found. Commencing bulldoze sequence')
                    print('--------------------------------------------')

                z=125.0
                yaw = -179.9
                pitch = 0.0

                clump_full_pose = fanuc_to_full_pose(fanuc_coords, z, yaw, pitch)

                if push_pull_count%2 == 0:

                    for i in range(len(clumps)):

                        DJ_bulldozer_push(*clump_full_pose[i])

                else:
                    for i in range(len(clumps)):

                        DJ_bulldozer_pull(*clump_full_pose[i])

                DJ_home()

                push_pull_count += 1
        if STATE == 2:
            print("Waiting for robot 2 to finish bulldozing")
            # Do nothing, robot 2 is bulldozing
        if STATE == 3:
            print("Robot 1 take picture------------------Press Enter to finish state 3...")
            # put in picture logic
            image_path = connect_camera()          # Capture image automatically

        if STATE == 4:
            print("Robot 2 Take Picture------------------DO NOTHING")
            # Do Nothing
        # if STATE == 5:
        #     input("Clear side------------------Press Enter to finish state 5...")
        #     # put in clearing logic
        #     # Combine with manual Z, yaw, pitch

        # if STATE == 6:
        #     #print("Finished---------------------GO HOME")
        #     input("Press Enter when home")
       
        if STATE > MAX_STATE:
            print("Master done.")
            client.publish(TOPIC, json.dumps({"sender": CLIENT_ID, "stop": True}))
            FINISHED = True
            client.disconnect()
            return
        print(f"Master -> State {STATE}")
        time.sleep(0.5)
        client.publish(TOPIC, json.dumps({"sender": CLIENT_ID, "state": STATE}))
    elif s < STATE:
        # Slave behind — remind it of current state
        client.publish(TOPIC, json.dumps({"sender": CLIENT_ID, "state": STATE}))

client = mqtt.Client(CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_forever()

print("finish routine and go home...................")


dice_coords = process_image('dice_capture.jpg')  # Detect dice centers
robot_coords = image_to_robot_coords(dice_coords)  # Convert to robot coordinates
print(len(dice_coords))
z = 125.0
yaw = -179.9
pitch = 0.0
fanuc_coords_list = robot_coords[0]  # first element of your tuple
fanuc_full_poses = fanuc_to_full_pose(fanuc_coords_list, z, yaw, pitch)

# print('full poses:')
# print(fanuc_full_poses)

DJ_home()


i=0

while i < len(dice_coords):

    DJ_to_dice(*fanuc_full_poses[i])
    DJ_putdown(65.0+(i*83))

    i+=1

DJ_home()






