import pybullet as pb
import numpy as np
import ArmInfo
import time
from math import *

import rospy
from std_msgs.msg import Int16MultiArray
import threading

# np.set_printoptions(precision= 2)

physicsClient = pb.connect(pb.GUI,  options='--background_color_red=0.0 --background_color_green=0.66 --background_color_blue=0.66')
joe = pb.loadURDF("Joe_URDF/robot_2arm/urdf/robot244.urdf", useFixedBase= 1)
pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)

fx = 0
fy = 0
fz = 0

left_arm_DH_table = np.array([[0, pi/2, 0, 0],
                              [-pi/2, pi/2, 0, -120],
                              [pi/2, -pi/2, 0, 0],
                              [pi/2, pi/2, 0, -250],
                              [0, pi/2, 0, 0],
                              [0, pi/2, 0, 250],
                              [pi/2, pi/2, 170, 0]])

right_arm_DH_table = np.array([[0, pi/2, 0, 0],
                              [-pi/2, pi/2, 0, 120],
                              [pi/2, -pi/2, 0, 0],
                              [pi/2, pi/2, 0, -250],
                              [0, pi/2, 0, 0],
                              [0, pi/2, 0, 250],
                              [pi/2, pi/2, 170, 0]])

left_arm = ArmInfo.ArmInfo(left_arm_DH_table)
right_arm = ArmInfo.ArmInfo(right_arm_DH_table)

def initialize():
    left_arm.delta_angle = np.array([0, 0, 0, 0, pi/2, 0, 0])
    right_arm.delta_angle = np.array([0.0, 0, 0, 0, 1e-10, 0, 0])
    left_arm.update()
    right_arm.update()

    print("Current Position = {}, {}".format(np.around(left_arm.current_position, 0), np.around(right_arm.current_position, 0)))
    print("Current Orientation = {}".format(np.around(left_arm.get_current_orientation(), 0)))

    for i in range(7):
        pb.setJointMotorControl2(joe, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = right_arm.delta_angle[i],
                                    force = 500)

    for i in range(8, 15):
        pb.setJointMotorControl2(joe, jointIndex=i,
                                controlMode=pb.POSITION_CONTROL,
                                targetPosition = left_arm.delta_angle[i-8],
                                force = 500)

def trajectory_planning(j0, ox, oy, oz, px, py, pz):
    velocity_factor = 1.0
    angular_threshold = pi/180
    linear_threshold = 1.0
    zeroth_joint_threshold = pi/180

    angular_error = sqrt(pow(ox * pi / 180 - left_arm.current_orientation[0], 2) +
                         pow(oy * pi / 180 - left_arm.current_orientation[1], 2) +
                         pow(oz * pi / 180 - left_arm.current_orientation[2], 2)) / 3
    
    linear_error = sqrt(pow(px - left_arm.current_position[0], 2) +
                        pow(py - left_arm.current_position[1], 2) +
                        pow(pz - left_arm.current_position[2], 2)) / 3

    zeroth_joint_error = (j0 * pi / 180 - pb.getJointState(joe, 8)[0])

    while angular_error > angular_threshold or linear_error > linear_threshold or abs(zeroth_joint_error) > zeroth_joint_threshold:
        # update arm state
        left_arm.delta_angle = np.array(pb.getJointStates(joe, [8, 9, 10, 11, 12, 13, 14]))[0:7, 0]
        left_arm.update()

        # update error
        angular_error = sqrt(pow(ox - left_arm.current_orientation[0], 2) +
                             pow(oy - left_arm.current_orientation[1], 2) +
                             pow(oz - left_arm.current_orientation[2], 2)) / 3
    
        linear_error = sqrt(pow(px - left_arm.current_position[0], 2) +
                            pow(py - left_arm.current_position[1], 2) +
                            pow(pz - left_arm.current_position[2], 2)) / 3

        zeroth_joint_error = (j0 * pi / 180 - pb.getJointState(joe, 8)[0])

        # calculate linear velocity
        vel_ox = (ox - left_arm.current_orientation[0]) * velocity_factor
        vel_oy = (ox - left_arm.current_orientation[1]) * velocity_factor
        vel_oz = (ox - left_arm.current_orientation[2]) * velocity_factor

        vel_px = (px - left_arm.current_position[0]) * velocity_factor
        vel_py = (py - left_arm.current_position[1]) * velocity_factor
        vel_pz = (pz - left_arm.current_position[2]) * velocity_factor

        vel_zeroth_joint = (j0 * pi / 180 - pb.getJointState(joe, 8)[0])

        linear_vel = np.array((vel_ox, vel_oy, vel_oz, vel_px, vel_py, vel_pz))
        
        angular_vel = np.matmul(left_arm.inverse_jacobian, linear_vel)

        # check velocity limit
        max_index = np.argmax(angular_vel)
        max_value = np.max(angular_vel)

        if abs(max_value) > pi:
            print("\tDangerous ! Stop moving.")
            break
        elif abs(max_value) > pi/6:
            angular_vel *= (pi / 6 / abs(max_value))
        
        # render in simulation
        for i in range(16):
            if i == 0 or i == 7 or i == 15:
                continue

            # elif i < 9:
            #     pb.setJointMotorControl2(joe, jointIndex=i,
            #                                   controlMode = pb.VELOCITY_CONTROL,
            #                                   targetVelocity = right_arm.angular_velocity[i-1],
            #                                   force = 50000)
            elif i == 8:
                pb.setJointMotorControl2(joe, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = vel_zeroth_joint,
                                            force = 50000)

            elif i >= 9:
                pb.setJointMotorControl2(joe, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = angular_vel[i-9],
                                            force = 50000)

        time.sleep(0.1)

    for i in range(16):
        pb.setJointMotorControl2(joe, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = 0,
                                        force = 500)

def adaptive():
    while True:
        # print("{}\t{}\t{}".format(fx, fy, fz))
        # update arm state
        left_arm.delta_angle = np.array(pb.getJointStates(joe, [8, 9, 10, 11, 12, 13, 14]))[0:7, 0]
        left_arm.update()

        vx = 0
        vy = 0
        vz = 0
        
        if abs(fx) < 50:
            vz = 0
        else:
            vz = -fx / 5

        if abs(fy) < 50:
            vy = 0
        else:
            vy = fy / 5

        if abs(fz) < 50:
            vx = 0
        else:
            vx = fz / 5


        linear_vel = np.array((0, 0, 0, vx, vy, vz))
        
        angular_vel = np.matmul(left_arm.inverse_jacobian, linear_vel)

        print(np.around(angular_vel, 2))

        # check velocity limit
        max_index = np.argmax(angular_vel)
        max_value = np.max(angular_vel)

        if abs(max_value) > pi:
            print("\tDangerous ! Stop moving.")
            break
        elif abs(max_value) > pi/6:
            angular_vel *= (pi / 6 / abs(max_value))
        
        # render in simulation
        for i in range(16):
            if i == 0 or i == 7 or i == 15:
                continue

            elif i >= 9:
                pb.setJointMotorControl2(joe, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = angular_vel[i-9],
                                            force = 50000)

        # time.sleep(0.8)

    for i in range(16):
        pb.setJointMotorControl2(joe, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = 0,
                                        force = 500)

def callback(data):
    global fx
    global fy
    global fz

    fx = data.data[0]
    fy = data.data[1]
    fz = data.data[2]
        
def listener():    
    rospy.Subscriber("ForseSensor_C2Python", Int16MultiArray, callback)
    rospy.spin()

rospy.init_node("aiRobots_Python", anonymous=True)
sub_td = threading.Thread(target=listener)
sub_td.start()

initialize()
time.sleep(3)
trajectory_planning(0, 0, 0, 0, 350, 120, -300)
adaptive()