import time
from time import sleep
from robot_controller import robot
import json
import random
from datetime import datetime
from fanuc_dice import DJ_pickup, DJ_rando, DJ_home, DJ_bottom_left, DJ_bottom_right, DJ_top_left, DJ_top_right, DJ_to_dice, DJ_putdown
import numpy as np
import paho.mqtt.client as mqtt

# from cv_grab import connect_camera, process_callibration_image
import numpy as np
import cv2
import mvsdk
import time

#fanuc stuff
import time
from time import sleep
from robot_controller import robot
import json
import random
from datetime import datetime


# Connect to the mindvision camera; must be hardwired in, writes an image to file so that other functions can retrieve it
def connect_camera():
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        raise RuntimeError("No camera found!")

    DevInfo = DevList[0]
    print("Selected camera:", DevInfo.GetFriendlyName())

    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        raise RuntimeError(f"CameraInit Failed({e.error_code}): {e.message}")

    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
    mvsdk.CameraPlay(hCamera)

    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if monoCamera else 3))

        filename = "dice_capture.jpg"
        cv2.imwrite(filename, frame)
        print(f"Image captured and saved as {filename}")

    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)

    return filename

#Takes image path as input and returns clump center coordinates as output.
    # These output coordinates are fed to image_to_robot_coords
def detect_dice_clumps(image_path, merge_distance=100, min_area=300, box_scale=1.00, area_threshold=6000):
    """
    Detect yellow dice in an image, merge close boxes, and return center coordinates (cx, cy, c_theta)
    of boxes with area > area_threshold, formatted for image_to_robot_coords().

    Parameters:
        image_path (str): Path to image, or None to capture via connect_camera().
        merge_distance (float): Distance threshold (in pixels) to merge nearby boxes.
        min_area (float): Minimum contour area to consider as a die.
        box_scale (float): Scaling factor for enlarging detected boxes.
        area_threshold (float): Only boxes with area > this value are returned.

    Returns:
        list of tuples: [(cx, cy, c_theta), ...] formatted for image_to_robot_coords()
    """
    # --- LOAD IMAGE ---
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"❌ Could not load image: {image_path}")

    # --- COLOR THRESHOLDING ---
    LOWER_YELLOW = np.array([15, 60, 60])
    UPPER_YELLOW = np.array([45, 255, 255])

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    # --- CLEAN MASK ---
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # --- FIND INITIAL BOXES ---
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in contours:
        if cv2.contourArea(c) > min_area:
            x, y, w, h = cv2.boundingRect(c)
            side = int(max(w, h) * box_scale)
            cx, cy = x + w // 2, y + h // 2
            x1 = max(cx - side // 2, 0)
            y1 = max(cy - side // 2, 0)
            x2 = min(cx + side // 2, image.shape[1])
            y2 = min(cy + side // 2, image.shape[0])
            boxes.append([x1, y1, x2, y2])

    # --- MERGE CLOSE BOXES ---
    def merge_boxes(boxes, dist_thresh):
        merged = True
        while merged:
            merged = False
            new_boxes = []
            skip = set()
            for i in range(len(boxes)):
                if i in skip:
                    continue
                x1a, y1a, x2a, y2a = boxes[i]
                for j in range(i + 1, len(boxes)):
                    if j in skip:
                        continue
                    x1b, y1b, x2b, y2b = boxes[j]
                    # Compute distance between centers
                    cxa, cya = (x1a + x2a) / 2, (y1a + y2a) / 2
                    cxb, cyb = (x1b + x2b) / 2, (y1b + y2b) / 2
                    dist = np.sqrt((cxa - cxb) ** 2 + (cya - cyb) ** 2)
                    if dist < dist_thresh:
                        # Merge
                        x1 = min(x1a, x1b)
                        y1 = min(y1a, y1b)
                        x2 = max(x2a, x2b)
                        y2 = max(y2a, y2b)
                        new_boxes.append([x1, y1, x2, y2])
                        skip.add(j)
                        merged = True
                        break
                else:
                    new_boxes.append(boxes[i])
            boxes = new_boxes
        return boxes

    merged_boxes = merge_boxes(boxes, merge_distance)

    # --- FILTER + COMPUTE CENTER COORDS ---
    clump_coords = []
    for (x1, y1, x2, y2) in merged_boxes:
        area = (x2 - x1) * (y2 - y1)
        if area > area_threshold:
            
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            c_theta = 0.0  # placeholder angle, can be replaced by orientation detection
            if cx<650:
                clump_coords.append((cx, cy, c_theta))
                # draw on image
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)

    # --- DISPLAY RESULT ---
    cv2.imshow("Detected Dice (Merged + Centers)", image)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    # --- RETURN (cx, cy, c_theta) FOR EACH DIE ---
    return clump_coords

# Process image and detect dice ---
def process_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image.")
        return []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([17, 40, 40])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = image.copy()
    dice_data = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300:
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w, h), angle = rect
        if min(w, h) == 0:
            continue

        side = min(w, h)
        square_rect = ((cx, cy), (side, side), angle)
        if w < h:
            angle = 90 + angle
        if angle > 90:
            angle -= 180
        if angle < 0:
            angle += 90

        if cx < 650:
            dice_data.append([cx, cy, angle, square_rect])

    dice_data.sort(key=lambda d: d[0])

    for i, (cx, cy, angle, square_rect) in enumerate(dice_data, start=1):
        box = cv2.boxPoints(square_rect)
        box = np.int32(box)
        cv2.drawContours(output, [box], -1, (0, 0, 255), 2)
        cv2.circle(output, (int(cx), int(cy)), 4, (0, 255, 0), -1)
        cv2.putText(output, f"Die {i}", (int(cx) - 25, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.putText(output, f"{angle:.1f}°", (int(cx) - 25, int(cy) + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    print("\nDetected Dice (Negative Angles Shifted +90°):")
    print(f"{'Die #':<6} {'Center X':<10} {'Center Y':<10} {'Angle (deg)':<10}")
    print("-" * 40)
    for i, (cx, cy, angle, _) in enumerate(dice_data, start=1):
        print(f"{i:<6} {round(cx,1):<10} {round(cy,1):<10} {round(angle,1):<10}")

    cv2.imwrite("dice_labeled_output.jpg", output)
    cv2.imshow("Dice - Angles Adjusted (+90 if Negative)", output)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    # Return dice centers (cx, cy) for further processing
    print([(d[0], d[1], d[2]) for d in dice_data])
    return [(d[0], d[1], d[2]) for d in dice_data]

# Draw bounding boxes and return dice image coordinates
# this output is fed to get_calibration_coordinates 
def process_callibration_image(image_path, num_dice):
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image.")
        return []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([17, 40, 40])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dice_data = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300:
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w, h), angle = rect
        if min(w, h) == 0:
            continue

        side = min(w, h)
        square_rect = ((cx, cy), (side, side), angle)
        if w < h:
            angle = 90 + angle
        if angle > 90:
            angle -= 180
        if angle < 0:
            angle += 90

        dice_data.append([cx, cy])

    # Sort by x-coordinate
    dice_data.sort(key=lambda d: d[0])

    # Take only the first 4 dice
    first_four = dice_data[:num_dice]

    # Swap x and y for output
    output_coords = [(y, x) for x, y in first_four]

    print('callibration image processed')

    return output_coords

# convert image coordinates to fanuc and standardbot coordinates
def image_to_robot_coords(dice_coords):
    """
    Swap X,Y coordinates and apply affine transformations to get robot X,Y.
    """

    # Fanuc affine matrix
    fanuc_affine_matrix = np.array([
        [ 1.48352900e+00, -1.00348000e-01, -5.22128324e+02],
        [ 6.65930000e-02,  1.45055700e+00, 2.40631969e+02]
    ])

    fanuc_coords = []
    for cx, cy, c_theta in dice_coords:
        # Swap X and Y before applying transform
        if cx < 650:
            img_pt = np.array([cy, cx, 1])
            fx, fy = fanuc_affine_matrix @ img_pt

            # Convert image angle to Fanuc angle
            fanuc_theta = 30 + (90 - c_theta)
            fanuc_coords.append((float(fx), float(fy), float(fanuc_theta)))

    print("\nDice coordinates converted to Fanuc X,Y:")
    for i, (fx, fy, fanuc_theta) in enumerate(fanuc_coords, start=1):
        print(f"Die {i}: X={fx:.2f}, Y={fy:.2f}, r={fanuc_theta:.2f}")

    # StandardBot affine matrix
    standard_affine_matrix = np.array([
        [-0.00151949, 0.0000987, 0.4648],
        [-0.0001228, -0.001549, 2.293]
    ])

    standard_coords = []
    for cx, cy, c_theta in dice_coords:
        if cx>650:
            img_pt = np.array([cy, cx, 1])
            sx, sy = standard_affine_matrix @ img_pt
            image_theta = c_theta
            standard_coords.append((float(sx), float(sy), float(image_theta)))

    print("\nDice coordinates converted to Standard X,Y:")
    for i, (sx, sy, image_theta) in enumerate(standard_coords, start=1):
        print(f"Die {i}: X={sx:.2f}, Y={sy:.2f}, Joint 5 = j5 + {image_theta:.2f}")

    return fanuc_coords, standard_coords

# turn dice coordinates into 6-item positions for the fanuc
def fanuc_to_full_pose(fanuc_coords, z, yaw, pitch):
    """
    Convert Fanuc (x, y, roll) coordinates to full robot pose [x, y, z, yaw, pitch, roll].
    
    Args:
        fanuc_coords (list of tuples): List of (x, y, roll) tuples
        z (float): Z coordinate
        yaw (float): yaw angle
        pitch (float): pitch angle
    
    Returns:
        list of lists: Each element is [x, y, z, yaw, pitch, roll]
    """
    fanuc_full_poses = []
    for x, y, roll in fanuc_coords:
        fanuc_full_pose = [float(x), float(y), float(z), float(yaw), float(pitch), float(roll)]
        fanuc_full_poses.append(fanuc_full_pose)
    
    # Print nicely
    print("\nFanuc full poses:")
    for i, pose in enumerate(fanuc_full_poses, start=1):
        print(f"Pose {i}: {pose}")
    
    return fanuc_full_poses

# runs full calibration routine and returns robot and image coords
def get_calibration_coords():
    """
    Moves robot to 4 calibration points, captures an image, detects dice centers,
    and returns robot_coords and image_coords.
    
    Returns:
        robot_coords: list of 4 (x, y) tuples
        image_coords: list of 4 (x, y) tuples (swapped if needed in process_callibration_image)
    """
    # Connect to robot and set speed
    robot_ip='10.8.4.16'
    robotDJ = robot(robot_ip)
    robotDJ.set_speed(300)

    # Move to home position
    DJ_home()

    # Define robot calibration points
    x1, y1 = -30.0, 365.0
    x2, y2 = 825.0, 430.0
    x3, y3 = 320.0, 700
    x4, y4 = 800.0, 850.0
    x5, y5 = -30.0, 1140.0
    

    num_dice = 5

    # Move robot to each point and perform actions
    for i in range(num_dice):
        DJ_pickup((65+(num_dice-1)*80) - i*80)
        if i == 0:
            DJ_top_left(x1, y1)
        elif i == 1:
            DJ_bottom_left(x2, y2)
        elif i==2:
            DJ_rando(x3,y3)
        elif i == 3:
            DJ_bottom_right(x4, y4)
        else:
            DJ_top_right(x5, y5)
            DJ_home()

    # Collect robot coordinates
    robot_coords = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

    # Capture image and detect dice centers
    image_path = connect_camera()  # Capture image automatically
    num_dice = len(robot_coords)
    image_coords = process_callibration_image(image_path, num_dice)  # Detect dice centers

    return robot_coords, image_coords

# takes image and robot coords and returns affine matrix
def compute_affine_matrix(image_coords, robot_coords):
    """
    Compute a 2x3 affine transformation matrix from image and robot coordinates.
    Accepts 3 or more points.
    """
    if len(image_coords) < 3 or len(robot_coords) < 3:
        raise ValueError("At least 3 corresponding points are required for affine transformation.")

    if len(image_coords) != len(robot_coords):
        raise ValueError("image_coords and robot_coords must have the same number of points.")

    img_pts = np.array(image_coords, dtype=np.float32)
    rob_pts = np.array(robot_coords, dtype=np.float32)

    M, _ = cv2.estimateAffine2D(img_pts, rob_pts)

    if M is None:
        raise ValueError("❌ Could not compute affine matrix. Check that points are not collinear.")

    M_rounded = np.round(M, 6)
    print("✅ Affine Transformation Matrix (2x3):")
    print(M_rounded)
    return M_rounded

# robot_coords, image_coords = get_calibration_coords()
# print('robot coords:')
# print(robot_coords)
# print('image coords:')
# print(image_coords)
# affine_matrix = compute_affine_matrix(image_coords, robot_coords)

# DJ_home()
# # DJ_open()
# image_path = connect_camera()          # Capture image automatically
# dice_coords = process_image(image_path)  # Detect dice centers
# robot_coords = image_to_robot_coords(dice_coords)  # Convert to robot coordinates

# fanuc_coords_list = robot_coords[0]  # first element of your tuple

# # Combine with manual Z, yaw, pitch
# z = 125.0
# yaw = -179.9
# pitch = 0.0

# fanuc_full_poses = fanuc_to_full_pose(fanuc_coords_list, z, yaw, pitch)

# print('full poses:')
# print(fanuc_full_poses)


# DJ_home()
# DJ_to_dice(*fanuc_full_poses[0])
# DJ_putdown(65.0)
# DJ_to_dice(*fanuc_full_poses[1])
# DJ_putdown(80+65)
# DJ_home()
