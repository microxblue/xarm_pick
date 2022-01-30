import os
import time
import cv2
import numpy as np
from   robot.robot_arm import RobotArm
from   robot.constants import XARM_CONFIG_FILE, GRIPPER_OPENED, GRIPPER_CLOSED

aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()


def transform_pt (matrix, p, conv_int = True):
  px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
  py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
  if conv_int:
      p_after = (int(px), int(py))
  else:
      p_after = (px, py)
  return p_after

def detect_markers (cap, id = None):
    # 5 markers are used
    # marker ID 1-4 is to mark the pickup rectangle area
    # marker ID 6 is to mark the cube to be picked up
    marker_rect = None
    retry = 0
    while retry < 10:
        _, image = cap.read()  #
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
        if ids is not None:
            ids = list(ids.flatten())
            if id is None:
                marker_ids = [1,2,3,4]
            else:
                marker_ids = [id]
            if set(marker_ids).issubset(set(ids)):
                if id is None:
                    marker_rect = np.array([corners[ids.index(1)][0][3], corners[ids.index(2)][0][2], corners[ids.index(4)][0][0], corners[ids.index(3)][0][1]])
                else:
                    marker_rect = corners[ids.index(id)][0]
                break
        retry += 1
        time.sleep (.02)
    return marker_rect


def main ():

    # check if the XARM needs calibration first
    if not os.path.exists(XARM_CONFIG_FILE):
        print ('XARM Config file could not be found. Calibration should be performed.')
        print ('Please run:\n  python xarm_calibrate.py')
        return

    # connect robot
    robot = RobotArm()

    # set initial pos
    START_ARM_JPOS = [0.0, -0.85, 1.75, 1.85, 0.0]
    robot.open_gripper()
    robot.move_arm_jpos(START_ARM_JPOS)

    # target image shape
    rows, cols = 240, 320

    # open camera
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    _, image = cap.read()

    # detect pickup area markers
    markers_pts = detect_markers (cap)
    if markers_pts is None:
        print ('Failed to detect expected markers from camera frame, please adjust camera posistion !')
        # show camera to allow user to adjust in order to detect markers
        while True:
            _, image = cap.read()
            markers_pts = detect_markers (cap)
            if markers_pts is not None:
                print ('Markers were detected !')
                return
            cv2.imshow('image', image)
            key = cv2.waitKey(10)
            if key == 27:
                return

    # matrix to tranform camera image to 320x240 rect image
    pts1   = markers_pts
    pts2   = np.float32([[0,        0],     [0,      rows],     [cols,      0],     [cols,    rows]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    # matrix to transform picking area 320x240 pixel to XARM picking space XY
    # it might need adjustment to work for your own XARM hardware
    cpt1 = np.float32([[0,        0],    [0,   320],      [240,   0],    [240, 320]])
    cpt2 = np.float32([[0.275 , 0.133],  [0.285, -0.098], [0.12, 0.12],  [0.13, -0.11]])
    pickup_matrix = cv2.getPerspectiveTransform(cpt1, cpt2)

    while True:
        _, image = cap.read()
        picking_area = cv2.warpPerspective(image, matrix, (cols, rows))

        # detect the cube (marker ID 6)
        cube_pts = []
        cube_pt  = detect_markers (cap, 6)
        if cube_pt is not None:
            for x, y in cube_pt:
                pt = transform_pt (matrix, (x,y))
                cv2.circle(picking_area, pt, 4, (255,0,0), 1)
                cube_pts.append(pt)

        # display picking area
        cv2.imshow('image', picking_area)
        key = cv2.waitKey(10)
        if key == 27:
            break

        if len(cube_pts) > 0:
            # calculate cube center point
            x = sum([cube_pts[i][0] for i in range(4)]) // 4
            y = sum([cube_pts[i][1] for i in range(4)]) // 4

            # transform to picking XYZ space
            pt = list(transform_pt (pickup_matrix, (y, x), False))
            z  = 0.04 - (y * 1.0 / rows) * 0.02
            pt.append (0.04 - (y * 1.0 / rows) * 0.02)

            # open gripper
            robot.set_gripper_state(GRIPPER_OPENED, speed = 50)

            # build picking up way points
            arm_jposs = []
            if pt[0] < 0.2:
                # for area nearby ( X < 0.2)
                waypts = [pt, [0.0,0.10,0.1]]
            else:
                # for area far ( X > 0.2)
                waypts = [[0.2,0.0,0.1], pt, [0.2,0.0,0.1], [0.0,0.10,0.1]]

            for waypt in waypts:
                arm_jpos, _ = robot.mp.calculate_ik(waypt)
                arm_jposs.append(arm_jpos)

            # return to initial pos at the end
            current_jpos = np.array (START_ARM_JPOS)
            arm_jposs.append(current_jpos)
            joint_ids    = robot.controller.arm_joint_ids
            speed     = 2.0

            # move arms to pick up
            for idx, arm_jpos in enumerate(arm_jposs):
                delta_jpos  = np.linalg.norm(np.subtract(arm_jpos, current_jpos))
                duration_ms = int(1000 * delta_jpos / speed) + 10
                robot.controller.move_servos(joint_ids, arm_jpos, duration=duration_ms)
                time.sleep(duration_ms / 1000)
                current_jpos = arm_jpos
                if idx == (len(arm_jposs) - 1) // 2 - 1:
                    # drop cube
                    robot.set_gripper_state(GRIPPER_CLOSED, backoff=-0.05, speed = 200)
                elif idx == len(arm_jposs) - 2:
                    # grab cube
                    robot.set_gripper_state(GRIPPER_OPENED, speed = 50)
                    time.sleep (.1)

            # wait before another pick up
            time.sleep (1)

    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
    #arm_pos ()
