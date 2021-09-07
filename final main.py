# Date: 2020 Nov 23
# Authors:
#
#       Elijah Cuico  21233069
#       Viktor Moreno 75388181
#
# Title: FG005 - 6 axis arm
# Description:
# A 6 axis robotic arm run by servos and an arduino nano. Movement based on spherical coordinates
import math
import random
import numpy as np
import pptk
import time
import matplotlib as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

running = 0

# initial angles for the arm. Format: base theta, lower arm phi, upper arm phi, hand phi
arm_phi_initial = [[0, 0, 0, 0],[0,0,-180,180],[0,0,-90,0],[[180,0,-90,0]]]

#elbow_pos = [0,0,0]
#wrist_pos = [0,0,0]
#hand_pos = [0,0,0]

# position testing. Will also have a randomly generated list to test robustness
pos_test = [[1,0,0],[0,1,0],[1,2,3],[4,5,6]]

len_lower = 14
len_upper = 12
len_hand = 7

radial = 0

tolerance = .5#0.009
optimization = 1.2#0.628

servo_min = 0
servo_max = 180

gimmie = [0,0]

# Calculates radial distance of a point
# Input: 3d coordinate
# Output: scalar distance
def calc_radial(position):
    return math.sqrt(math.pow(position[0],2),math.pow(position[1],2))

# returns the angle relative to a fixed cartesian xy plen
# Input: target coord
# Output: angle in degrees
def target_theta(position):

    reference = math.atan2(position[0],position[1])

    if position[0] < 0:

        # quadrant 3
        if position[1] < 0:
            reference += 180

        # quadrant 2
        reference += 90

    # quadrant 4
    if position[1] < 0:
        reference += 360

    return reference

# returns the refernce angle from the current base angle to the target base angle on xy plane
# Input: base angle readout, target coordinates
# Output: reference angle in degrees
def base_angle_difference(base_angle, target_coord):
    return math.atan2(target_coord[1],target_coord[0]) - base_angle


# Calculate joint position as a 3d cartesian coordinate
# Input: Base servo angle, gyro Ry angle, arm length
# Output: 3d coordinate
def joint_pos(base_ang, gyro_ang, arm_length):
    if base_ang == 0:
        x = arm_length * math.sin(math.radians(gyro_ang))
        y = 0
        z = arm_length * math.cos(math.radians(gyro_ang))
    else:
        x = arm_length * math.sin(math.radians(base_ang)) * math.cos(math.radians(gyro_ang))
        y = arm_length * math.sin(math.radians(base_ang)) * math.sin(math.radians(gyro_ang))
        z = arm_length * math.cos(math.radians(gyro_ang))

    #print(x)
    #print(y)
    #print(z)

    position = [x, y, z]

    return position

# calculates the position of a joint in 2d space. Mode determines if the axis is flipped (if top is 90 or 0 degrees). 90 degrees by default.
# Input: angle, arm length, if axis is flipped or not
# Output: 2d coordinate
def planar_joint_pos(ang, arm_len, mode):
    if(mode == 1):
        x = arm_len * math.sin(math.radians(ang))
        y = arm_len * math.cos(math.radians(ang))
    else:
        x = arm_len * math.cos(math.radians(ang))
        y = arm_len * math.sin(math.radians(ang))

    return [x,y]

# Calculates the vector between two 3d points
# Input: 3d point a, 3d point b
# Output: 3d vector
def point_vec(pos_a, pos_b):
    x = pos_b[0] - pos_a[0]
    y = pos_b[1] - pos_a[1]
    z = pos_b[2] - pos_a[2]

    position = [x,y,z]

    return position

# Calculates the vector between two 3d points
# Input: 2d point a, 2d point b
# Output: 2d vector
def planar_point_vec(pos_a, pos_b):
    x = pos_b[0] - pos_a[0]
    y = pos_b[1] - pos_a[1]

    return [x,y]

# Length of a given 3d vector
# Input: 3d vector
# Output: scalar length
def vector_length(vector):
    return math.sqrt(math.pow(math.fabs(vector[0]),2) + math.pow(math.fabs(vector[1]),2) + math.pow(math.fabs(vector[2]),2))

# Length of a given 2d vector
# Input: 2d vector
# Output: scalar length
def planar_vector_length(vector):
    return math.sqrt((vector[0]*vector[0]) + (vector[1]*vector[1]))

# Mode 1 movement: Using an internal 3D cartesian system based on the base position
# Uses difference in theta to move

# Mode 3 movement: Using vectors to calculate positioning of joints in an internally defined cartesian coordinate
# system

class Servo():
    def __init__(self, initial_angle):
        self.angle = initial_angle

    def move(self, degree):
        self.angle += degree

        #servo limits
        if self.angle > servo_max:
            self.angle = servo_max
        if self.angle < servo_min:
            self.angle = servo_min

    def read_angle(self):
        return self.angle

def move_mode_three(base_ang, lower_ang, upper_ang, hand_ang, lower_len, upper_len, hand_len, target_pos):
    dphi = 0.1

    elbow_pos = joint_pos(base_ang, lower_ang, lower_len)
    wrist_pos = joint_pos(base_ang, upper_ang, upper_len)
    hand_pos = joint_pos(base_ang, hand_ang, hand_len)

    x = "Elbow Position:"
    y = "Wrist position:"
    z = "Hand position:"

    print(x)
    print(elbow_pos)
    print(y)
    print(wrist_pos)
    print(z)
    print(hand_pos)
    print("\n\n")

    wrist_set = 1
    elbow_set = 1
    shoulder_set = 1
    base_set = 1

    dbase = dphi
    dshoulder = dphi
    delbow = dphi
    dwrist = dphi

    while(shoulder_set or base_set or elbow_set or wrist_set):
        if vector_length(target_pos) > (len_lower + len_upper + len_hand - 2):
            print("Out of range!")
            break

        base_ang = servo_base.read_angle()

        # aligning the base with the target point
        if base_angle_difference(base_ang, target_pos) < 1 and base_angle_difference(base_ang, target_pos) > (-1):
            servo_base.move(0) # to avoid jitter in the servos
            base_set = 0
            #print("Base set\n")
        elif base_angle_difference(base_ang, target_pos) > 0:
            servo_base.move(-dbase)
        else:
            servo_base.move(dbase)

        # set so distance from elbow position to target point is the length of the upper arm
        elbow_pos = joint_pos(base_ang, lower_ang, lower_len)

        # lower arm control
        lower_ang = servo_shoulder.read_angle()
        if math.fabs(vector_length(point_vec(elbow_pos, target_pos)) - upper_len) <= tolerance or shoulder_set == 0:
            print("\nELBOW FINAL POS")
            print(elbow_pos)
            len_current = vector_length(point_vec(elbow_pos, target_pos))
            print("Elbow distance to point:")
            print(len_current)
            servo_shoulder.move(0) #
            shoulder_set = 0
        else:#if vector_length(point_vec(elbow_pos, target_pos)) != (upper_len*optimization):
            print(round(vector_length(point_vec(elbow_pos, target_pos)),2))
            print("Shoulder angle:")
            print(servo_shoulder.read_angle())

            # compares the change in distance before and after the angle change
            len_previous = vector_length(point_vec(elbow_pos, target_pos))
            print("Previous elbow pos:")
            print(elbow_pos)

            print(len_previous)

            servo_shoulder.move(dshoulder)

            elbow_pos = joint_pos(base_ang, servo_shoulder.read_angle(), lower_len)
            print("Current Elbow pos:")
            print(elbow_pos)

            len_current = vector_length(point_vec(elbow_pos, target_pos))
            print(len_current)

            if servo_shoulder.read_angle() > servo_max or servo_shoulder.read_angle() > servo_max:
                servo_shoulder.move(-dshoulder)
                print("Out of shoulder range!")
                shoulder_set = 0
            elif servo_shoulder.read_angle() < servo_min or servo_shoulder.read_angle() < servo_min:
                servo_shoulder.move(dshoulder)
                print("Out of shoulder range!")
                shoulder_set = 0
            if len_previous < len_current:
                print("PLEASE FLIP")
                dshoulder = dshoulder * (-1)
                print(dshoulder)
            #if len_previous > len_current and math.fabs(len_current-len_previous) < tolerance:
            #    shoulder_set = 0

        # upper arm control
        upper_ang = servo_elbow.read_angle() + servo_shoulder.read_angle()


        wrist_pos = joint_pos(base_ang, upper_ang, upper_len)
        wrist_pos = [wrist_pos[0] + elbow_pos[0], wrist_pos[1] + elbow_pos[1], wrist_pos[2] + elbow_pos[2]]

        if math.fabs(vector_length(point_vec(wrist_pos, target_pos)) - hand_len) <= tolerance and shoulder_set == 0 \
                or elbow_set == 0:
            servo_elbow.move(0) # within tolerance to avoid potential jitter
            elbow_set = 0
            print("WRIST FINAL POS")
            print(wrist_pos)
            len_current = vector_length(point_vec(wrist_pos, target_pos))
            print("Wrist distance to point:")
            print(len_current)

        else:
            print(round(vector_length(point_vec(wrist_pos, target_pos)), 2))
            print("Elbow angle:")
            print(servo_elbow.read_angle())

            # compares the change in distance before and after the angle change
            len_previous = vector_length(point_vec(wrist_pos, target_pos))
            print(len_previous)

            servo_elbow.move(delbow)  # move the elbow

            wrist_pos = joint_pos(base_ang, servo_elbow.read_angle(), upper_len)
            wrist_pos = [wrist_pos[0] + elbow_pos[0], wrist_pos[1] + elbow_pos[1], wrist_pos[2] + elbow_pos[2]]

            print("Current Wrist pos:")
            print(wrist_pos)

            len_current = vector_length(point_vec(wrist_pos, target_pos))
            print(len_current)

            if servo_elbow.read_angle() > servo_max:
                servo_elbow.move(-delbow)
                print("Out of elbow range!")
                elbow_set = 0
            elif servo_elbow.read_angle() < servo_min:
                servo_elbow.move(delbow)
                print("Out of elbow range!")
                elbow_set = 0

            if len_previous < len_current:
                print("PLEASE FLIP")
                delbow = delbow * (-1)
                print(delbow)
            #elif len_previous > len_current and math.fabs(len_current - len_previous) < tolerance:
            #    elbow_set = 0

        # wrist control
        hand_pos = joint_pos(base_ang, hand_ang, hand_len) + wrist_pos
        hand_pos = [hand_pos[0] + wrist_pos[0], hand_pos[1] + wrist_pos[1], hand_pos[2] + wrist_pos[2]]

        hand_ang = servo_wrist.read_angle() + servo_elbow.read_angle()

        if math.fabs(vector_length(point_vec(wrist_pos, target_pos)) - hand_len) <= tolerance\
                and elbow_set == 0 and shoulder_set == 0 or wrist_set == 0:
            servo_wrist.move(0)  # within tolerance to avoid potential jitter
            wrist_set = 0
            print("HAND FINAL POS")
            print(hand_pos)
            len_current = vector_length(point_vec(wrist_pos, target_pos))
            print("Hand distance to point")
            print(len_current)
        else:
            print(round(vector_length(point_vec(wrist_pos, target_pos)), 2))
            print("Wrist angle:")
            print(servo_wrist.read_angle())

            # compares the change in distance before and after the angle change
            len_previous = vector_length(point_vec(wrist_pos, target_pos))
            print(len_previous)
            print("Previous hand pos:")
            print(hand_pos)


            servo_wrist.move(dwrist)  # move the wrist
            print("dwrist:")
            print(dwrist)
            hand_pos = joint_pos(base_ang, servo_wrist.read_angle(), hand_len)
            hand_pos = [hand_pos[0] + wrist_pos[0], hand_pos[1] + wrist_pos[1], hand_pos[2] + wrist_pos[2]]

            print("Current hand pos:")
            print(hand_pos)

            len_current = vector_length(point_vec(wrist_pos, target_pos))
            print(len_current)

            if servo_wrist.read_angle() > servo_max:
                servo_wrist.move(-dwrist)
                print("Out of wrist range!")
                wrist_set = 0
            elif servo_wrist.read_angle() < servo_min:
                servo_wrist.move(dwrist)
                print("Out of wrist range!")
                wrist_set = 0

            if len_previous < len_current:
                print("PLEASE FLIP")
                dwrist = dwrist * (-1)
                print(dwrist)
            #elif len_previous > len_current and math.fabs(len_current - len_previous) < tolerance:
            #    wrist_set = 0

        #time.sleep(.5)
        #print(base_set)
        #print(shoulder_set)
        #print(elbow_set)
        #print(wrist_set)


        #time.sleep(1)
        #print(x)
        #print(elbow_pos)
        #print(y)
        #print(wrist_pos)
        #print(z)
        #print(hand_pos)
        #print("\n")


# physical parameters
# the entire arm is vertical

servo_base = Servo(0)
servo_shoulder = Servo(90)
servo_elbow = Servo(0)
servo_wrist = Servo(0)

# Remake of mode 3
# arm is the longest one attached to the base, forearm is the middle one, hand is the shortest one on the end.
def simple_mode_three(base_servo, arm_servo, arm_len, forearm_servo, forearm_len, hand_servo, hand_len, target_pos):
    # from base to length of arm
    elbow_pos = planar_joint_pos(arm_servo.read_angle(), arm_len, 0)

    # from base to length of forearm
    wrist_pos = planar_joint_pos(forearm_servo.read_angle(), forearm_len, 1)
    # true wrist position
    # wrist_pos = [wrist_pos[0] + elbow_pos[0], wrist_pos[1] + elbow_pos[1]]

    # from base to length of hand
    hand_pos = planar_joint_pos(hand_servo.read_angle(), hand_len, 1)
    # true hand position
    # hand_pos = [hand_pos[0] + wrist_pos[0], hand_pos[1] + wrist_pos[1]]

    hand_set = 1
    wrist_set = 1
    elbow_set = 1
    base_set = 1

    x = "Elbow Position:"
    y = "Wrist position:"
    z = "Hand position:"

    # movement rates
    dbase = 0.1
    dshoulder = 0.1
    delbow = 0.1
    dwrist = 0.1


    # planar_target_pos = [target_pos[0], target_pos[2]]

    while elbow_set or wrist_set or hand_set:
        # since the base is rotating, the coordinate values will always be in the first quadrant
        planar_target_pos = [math.fabs(target_pos[0]), target_pos[2]]

        if planar_vector_length(planar_target_pos) > (len_lower + len_upper + len_hand):
           # print(planar_vector_length(target_pos))
            print("ERROR: Out of range!")
            elbow_set = 0
            wrist_set = 0
            hand_set = 0

        # Move base to correspond with target
        if base_angle_difference(base_servo.read_angle(), target_pos) < 1 and base_angle_difference(base_servo.read_angle(), target_pos) > (-1):
            servo_base.move(0)  # to avoid jitter in the servos
            base_set = 0
            # print("Base set\n")
        elif base_angle_difference(base_servo.read_angle(), target_pos) > 0:
            servo_base.move(-dbase)
        else:
            servo_base.move(dbase)

        #  arm control
        # close to the origin, less than radius 12
        if planar_vector_length(planar_target_pos) < forearm_len:
            len_expected = (forearm_len + hand_len) * (1 - (1/planar_vector_length(planar_target_pos)))
            #len_expected = forearm_len + ((hand_len) * (1 - (1 / planar_vector_length(planar_target_pos))))
            print("CONDITION 1")
            # print("Target length")
            # print(len_expected)
        # really close to the origin, less than radius 7
            if planar_vector_length(planar_target_pos) < hand_len:
                #len_expected = forearm_len + (planar_vector_length(planar_target_pos))
                len_expected = forearm_len + ((hand_len) * (1 - (1 / planar_vector_length(planar_target_pos))))
                print("CONDITION 2")
        else:
            # len_expected = (forearm_len + hand_len) / (arm_len + forearm_len + hand_len) * planar_vector_length(
            #         planar_target_pos)
            len_expected = (forearm_len + hand_len) * (1 - (1 / planar_vector_length(planar_target_pos)))
            print("CONDITION 3")

        # subtracting len expected so that it checks tha actual distance from the joint to the point
        len_current = math.fabs(planar_vector_length(planar_point_vec(elbow_pos, planar_target_pos)))

        # print("\n\nTOLERANCE 1")
        # print(len_current - len_expected)
        # print(elbow_set)

        if tolerance > (len_current-len_expected) > -tolerance:
            elbow_set = 0
            print("ELBOW SET")
            # print(len_expected)
            # print(len_current)
            # print(elbow_pos)
            # print(planar_target_pos)
            # print(planar_vector_length(planar_point_vec(elbow_pos, planar_target_pos)))
            # old_ref_ang = math.atan((planar_target_pos[1]-elbow_pos[1])/(planar_target_pos[0]-elbow_pos[0]))
            # new_ref_ang = old_ref_ang + (90 - arm_servo.read_angle())

            #planar_target_pos = [(planar_target_pos[0] - elbow_pos[0])*math.cos(math.radians(arm_servo.read_angle())),
            #                     (planar_target_pos[1] - elbow_pos[1])*math.sin(math.radians(arm_servo.read_angle()))]
            # THIS ONE planar_target_pos = [(forearm_len * math.cos(math.radians(new_ref_ang))), (forearm_len * math.sin(math.radians(new_ref_ang)))]

            # print("ELBOW FINAL POS")
            # prints the actual coordinates of the elbow
            # print(elbow_pos)
            # print("ELBOW EXPECTED DISTANCE")
            # print(len_expected)
            # print("ELBOW FINAL DISTANCE")
            # print(len_current + len_expected)
        elif elbow_set == 1:
            # The goal is to get the distance between the elbow and the point as close as possible
            # print("\n\nTOLERANCE 2")
            # print(len_current-len_expected)
            # print(elbow_set)

            # print(planar_target_pos)
            print("Elbow Expected Length:")
            print(len_expected)
            len_prev = len_current
            print("Elbow Previous Length:")
            print(len_prev)

            arm_servo.move(dshoulder)
            print("Shoulder Servo Angle:")
            print(arm_servo.read_angle())

            elbow_pos = planar_joint_pos(arm_servo.read_angle(), arm_len, 0)

            len_current = math.fabs(planar_vector_length(planar_point_vec(elbow_pos, planar_target_pos)))
            print("Elbow Current Length:")
            print(len_current)
            # print("Elbow position:")
            # print(elbow_pos)
            # print("\n")

            # max servo limits
            if servo_shoulder.read_angle() == servo_max:
                servo_shoulder.move(-dshoulder)
                # print("Out of shoulder range!")
                elbow_set = 0
            elif servo_shoulder.read_angle() == servo_min:
                servo_shoulder.move(dshoulder)
                # print("Out of shoulder range!")
                elbow_set = 0

            # if the current position after the angle was changed is too far, go the other way.
            # if target_pos[0] <= 0:
            #     dshoulder = dshoulder
            # if it has gone past
            if math.fabs(len_current) > len_expected and math.fabs(len_current) - math.fabs(len_prev) > 0:
                #if target_pos[0] > 0:
                print("SHOULDER FLIP")
                print(len_current - len_expected)
                dshoulder = dshoulder * (-1)

                if tolerance > len_current - len_expected > -tolerance:
                    print("SHOULDER STOP")
                    arm_servo.move(-dshoulder)
                    elbow_set = 0
            #    print(dshoulder)

        # Forearm control
        # old_ref_ang = math.atan((planar_target_pos[1] - elbow_pos[1]) / (planar_target_pos[0] - elbow_pos[0]))
        # new_ref_ang = old_ref_ang + (90-arm_servo.read_angle())
        #
        # planar_target_pos = [(forearm_len * math.cos(math.radians(new_ref_ang))),
        #                      (forearm_len * math.sin(math.radians(new_ref_ang)))]

        planar_target_pos = [planar_target_pos[0] - elbow_pos[0], planar_target_pos[1] - elbow_pos[1]]

        len_expected = hand_len
        len_current = math.fabs(planar_vector_length(planar_point_vec(wrist_pos, planar_target_pos)))
        # print(len_current)

        # print("ERROR 3")
        # print(len_current - len_expected)
        if tolerance > math.fabs(len_current - len_expected) and elbow_set == 0:
            wrist_set = 0
            print("WRIST SET")
            print(len_current - len_expected)
            # Coordinate shift for hand
            # remember to use the z axis for target pos
            # planar_target_pos = [(planar_target_pos[0] - wrist_pos[0]) * math.cos(math.radians(arm_servo.read_angle())),
            #                      (planar_target_pos[1] - wrist_pos[1]) * math.sin(math.radians(arm_servo.read_angle()))]

            # old_ref_ang = math.atan((planar_target_pos[1] - wrist_pos[1]) / (planar_target_pos[0] - wrist_pos[0]))
            # new_ref_ang = old_ref_ang + (90 - forearm_servo.read_angle())

            # planar_target_pos = [(planar_target_pos[0] - elbow_pos[0])*math.cos(math.radians(arm_servo.read_angle())),
            #                     (planar_target_pos[1] - elbow_pos[1])*math.sin(math.radians(arm_servo.read_angle()))]
            #THIS ONE planar_target_pos = [(hand_len * math.cos(math.radians(new_ref_ang))),
            #                      (hand_len * math.sin(math.radians(new_ref_ang)))]

            # Shifts to real coordinates for elbow
            # wrist_pos = [wrist_pos[0] + elbow_pos[0], wrist_pos[1] + elbow_pos[1]]

            # print ("WRIST FINAL POS")
            # #  actual coordinates of wrist
            # print(wrist_pos)
            # print("WRIST EXPECTED DISTANCE")
            # print(len_expected)
            # print("WRIST FINAL DISTANCE")
            # print(len_current + len_expected)
        elif wrist_set == 1:
            # Coordinate shift for the forearm
            print("Wrist expected length")
            print(len_expected)

            len_prev = len_current
            print("Wrist previous length:")
            print(len_prev)

            forearm_servo.move(delbow)
            # print(forearm_servo.read_angle())

            wrist_pos = planar_joint_pos(forearm_servo.read_angle(), forearm_len, 1)

            # uses shifted coordinates from arm at this point
            len_current = planar_vector_length(planar_point_vec(wrist_pos, planar_target_pos))
            print("Wrist current length:")
            print(len_current)
            print("\n")

            # max servo limits
            if servo_elbow.read_angle() == servo_max:
                servo_elbow.move(-delbow)
                # print("Out of elbow range!")
                elbow_set = 0
            elif servo_elbow.read_angle() == servo_min:
                servo_elbow.move(delbow)
                # print("Out of elbow range!")
                elbow_set = 0

            # if the current position after the angle was changed is too far, go the other way.
            if target_pos[0] <= 0:
                delbow = delbow
            if math.fabs(len_current) > len_expected and math.fabs(len_current) - math.fabs(len_prev) > 0:
                # if target_pos[0] > 0:
                # print("ELBOW FLIP")
                delbow = delbow * (-1)
                if tolerance > len_current-len_expected > -tolerance and elbow_set == 0:
                    # print("ELBOW STOP")
                    forearm_servo.move(-delbow)
                    wrist_set = 0
            #    print(delbow)

        # Wrist control
        # planar_target_pos = [target_pos[0], target_pos[2]]
        #
        # old_ref_ang = math.atan((planar_target_pos[1] - wrist_pos[1] - elbow_pos[1]) /
        #                         (planar_target_pos[0] - wrist_pos[0] - elbow_pos[0]))
        # new_ref_ang = old_ref_ang + (90 - forearm_servo.read_angle())
        #
        # planar_target_pos = [(hand_len * math.sin(math.radians(new_ref_ang))),
        #                      (hand_len * math.cos(math.radians(new_ref_ang)))]
        planar_target_pos = [planar_target_pos[0] - wrist_pos[0], planar_target_pos[1] - wrist_pos[1]]

        len_expected = 0
        len_current = planar_vector_length(planar_point_vec(hand_pos, planar_target_pos))

        # print("Hand Pos")
        # print(hand_pos)
        # print("Target pos")
        # print(planar_target_pos)
        # print("Hand previous length:")
        # print(len_prev)
        # print(hand_set)
        # print(elbow_set)
        # print(len_current)

        if tolerance > len_current > -tolerance and wrist_set == 0 and elbow_set == 0:
            hand_set = 0

            # Coordinate shift for hand
            # remember to use the z axis for target pos
            # Shifts to real coordinates for elbow

            # print("HAND FINAL POS")
            # #  actual coordinates of wrist
            # print(hand_pos)
            # print("HAND EXPECTED DISTANCE")
            # print(len_expected)
            # print("HAND FINAL DISTANCE")
            # print(len_current)
        elif hand_set == 1:
            # Coordinate shift for the forearm
            print("Hand Expeted Length")
            print(len_expected)

            len_prev = len_current
            print("Wrist Previous Length")
            print(len_prev)

            hand_servo.move(dwrist)
            print(hand_servo.read_angle())

            hand_pos = planar_joint_pos(hand_servo.read_angle(), hand_len, 1)
            # uses shifted coordinates from arm at this point
            len_current = planar_vector_length(planar_point_vec(hand_pos, planar_target_pos))
            # print("Target coord on hand plane")
            # print(planar_target_pos)
            print("Hand current length:")
            print(len_current)
            print("\n")

            # max servo limits
            if hand_servo.read_angle() == servo_max:
                hand_servo.move(-dwrist)
                # print("Out of elbow range!")
                hand_set = 0
            elif hand_servo.read_angle() == servo_min:
                hand_servo.move(dwrist)
                # print("Out of elbow range!")
                hand_set = 0

            # if the current position after the angle was changed is too far, go the other way.
            if target_pos[0] <= 0:
                dwrist = dwrist
            if math.fabs(len_current) > len_expected and math.fabs(len_current) - math.fabs(len_prev) > 0:
                print("WRIST FLIP")
                dwrist = dwrist * (-1)

                #if tolerance > len_current - len_expected > -tolerance and elbow_set == 0 and wrist_set == 0:
                if tolerance > len_current > -tolerance:
                    # print("ELBOW STOP")
                    hand_servo.move(-dwrist)
                    hand_set = 0
            #    print(dwrist)

        #time.sleep(.1)
        print("\nJoint position")
        print(elbow_pos)
        print(wrist_pos)
        print(hand_pos)
        print("Joint set:")
        print(elbow_set)
        print(wrist_set)
        print(hand_set)
        # print("Joint Angle")
        # print(arm_servo.read_angle())
        # print(forearm_servo.read_angle())
        # print(hand_servo.read_angle())

    # print("Joint position")
    # print(elbow_pos)
    # print(wrist_pos)
    # print(hand_pos)

    elbow_pos = [elbow_pos[0], base_servo.read_angle(), elbow_pos[1]]
    wrist_pos = [wrist_pos[0] + elbow_pos[0], base_servo.read_angle(), wrist_pos[1] + elbow_pos[2]]
    hand_pos = [hand_pos[0] + wrist_pos[0], base_servo.read_angle(), hand_pos[1] + wrist_pos[2]]


    print("\n\nFINAL POSITIONS (Elbow, Wrist, Hand):")
    print(elbow_pos)
    print(wrist_pos)
    print(hand_pos)

    #
    return [elbow_pos, wrist_pos, hand_pos]

# generates a cloud of coordinates for the robot arm
# input: Amount of coords, a seed, max distance on an axis, minimum distance on an axis
# Output: array of coords
def rand_coords(amount, seed, max_bound, min_bound):
    x = 0
    y = 0
    z = 0

    coord_list = [[10,10,10]]

    random.seed(seed)

    for r in range(amount):
        # excludes one half of the sphere, and a volume near the center of the arm's base
        if r % 2 == 0:
            x = random.randint(min_bound, max_bound)
            y = random.randint(-max_bound,-min_bound)
        else:
            x = random.randint(-max_bound, -min_bound)
            y = random.randint(min_bound, max_bound)

        z = random.randint(0,max_bound)

        coord_list.append([x,y,z])

    return coord_list

# Checks the output of the movement against a list of coordinates, passes if within some acceptable range
# Input: Array of 3d coords, the type of movement with input values ready, tolderance for passing
def arm_grader(coord_list, tolerance, movement_mode):
    array_length = len(coord_list)
    dimensions = 3
    passed = 0
    failed = 0

    start_time = time.time()

    for x in range(array_length):
        if math.fabs(simple_mode_three(base_servo, arm_servo, arm_len, forearm_servo, forearm_len, hand_servo, hand_len, coord_list[x])[2][0] - coord_list[x][0]) > tolerance or \
                math.fabs(simple_mode_three(base_servo, arm_servo, arm_len, forearm_servo, forearm_len, hand_servo, hand_len, coord_list[x])[2][1] - coord_list[x][1]) > tolerance or \
                math.fabs(simple_mode_three(base_servo, arm_servo, arm_len, forearm_servo, forearm_len, hand_servo, hand_len, coord_list[x])[2][2] - coord_list[x][2]) > tolerance:
            failed += 1
            print("\n\nFailed on:")
            print(coord_list[x])
            print("\n\n")

        else:
            passed += 1
        #print(x)

    current_time = time.time()

    print("Passed:")
    print(passed)
    print("Failed")
    print(failed)
    print("Operation took: %s seconds" % (current_time - start_time))
    print("Average time of %s seconds" % ((current_time-start_time)/array_length))






# simulation



#arm_grader(rand_coords(2, 8465, 20, 20), 30,  servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand)


# initial_elbow = joint_pos(servo_base.read_angle(), servo_shoulder.read_angle(), len_lower)
# initial_wrist = joint_pos(servo_base.read_angle(), servo_elbow.read_angle(), len_upper)
# initial_wrist = [initial_wrist[0]+initial_elbow[0],initial_wrist[1]+initial_elbow[1],initial_wrist[2]+initial_elbow[2]]
# initial_hand = joint_pos(servo_base.read_angle(), servo_wrist.read_angle(), len_hand)
# initial_hand = [initial_hand[0] + initial_wrist[0], initial_hand[1] + initial_wrist[1], initial_hand[2] + initial_wrist[2]]

# initial_pos = [initial_elbow, initial_wrist, initial_hand, test_pos]

# move_mode_three(servo_base.read_angle(), servo_shoulder.read_angle(), servo_elbow.read_angle(), servo_wrist.read_angle()
#                , len_lower, len_upper, len_hand, test_pos)



# final_elbow = joint_pos(servo_base.read_angle(), servo_shoulder.read_angle(), len_lower)
# final_wrist = joint_pos(servo_base.read_angle(), servo_elbow.read_angle(), len_upper)
# final_wrist = [final_wrist[0]+final_elbow[0],final_wrist[1]+final_elbow[1],final_wrist[2]+final_elbow[2]]
# final_hand = joint_pos(servo_base.read_angle(), servo_wrist.read_angle(), len_hand)
# final_hand = [final_hand[0] + final_wrist[0], final_hand[1] + final_wrist[1], final_hand[2] + final_wrist[2]]

# position = [final_elbow, final_wrist, final_hand]

#P = numpy.random.rand(100,3)
# P = np.asarray(simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos))
# v = pptk.viewer(P)
# v.attributes(P) # have to provide the colour attributes to the points
# v.set(point_size =0.09)

# fig = plt.figure()
# ax = plt.axes(projection="3d")
#
# plt.show()

#mpl.rcParams['legend.fontsize'] = 10
plt.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')

test_pos_one = [20,-7,5]
test_pos_two = [-5,15,17]
test_pos_three = [12,-5, 23]
test_pos_four = [13, 24, 3]

simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos_one)

arr_one = simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos_one)
arr_two = simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos_two)
arr_three = simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos_three)
arr_four = simple_mode_three(servo_base, servo_shoulder, len_lower, servo_elbow, len_upper, servo_wrist, len_hand, test_pos_four)

# for x in arr:
#     x = arr[x][0]
#     y = arr[x][1]
#     z = arr[x][2]
#
#     ax.plot(x,y,z, label='arm thing')

x = [0, arr_one[0][0], arr_one[1][0], arr_one[2][0]]
y = [0, arr_one[0][1], arr_one[1][1], arr_one[2][1]]
z = [0, arr_one[0][2], arr_one[1][2], arr_one[2][2]]

ax.plot(x, y, z, label='Arm 1')

x = [0, arr_two[0][0], arr_two[1][0], arr_two[2][0]]
y = [0, arr_two[0][1], arr_two[1][1], arr_two[2][1]]
z = [0, arr_two[0][2], arr_two[1][2], arr_two[2][2]]

ax.plot(x, y, z, label='Arm 1')

x = [0, arr_three[0][0], arr_three[1][0], arr_three[2][0]]
y = [0, arr_three[0][1], arr_three[1][1], arr_three[2][1]]
z = [0, arr_three[0][2], arr_three[1][2], arr_three[2][2]]

ax.plot(x, y, z, label='Arm 1')

x = [0, arr_four[0][0], arr_four[1][0], arr_four[2][0]]
y = [0, arr_four[0][1], arr_four[1][1], arr_four[2][1]]
z = [0, arr_four[0][2], arr_four[1][2], arr_four[2][2]]

ax.plot(x, y, z, label='Arm 1')

ax.legend()
plt.show()