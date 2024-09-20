from math import *
import numpy as np

class ArmInfo(object):
    def __init__(self, DH_table):
        # in terms of theta, alpha, a, d
        self.DH_table = DH_table

        # these two is currently based on 1st coordination which looks like spider man
        self.current_position = np.zeros(shape=(3))

        #
        self.current_orientation = np.zeros(shape=(3))

        # used to adjust the scale of linear velocity
        self.velocity_factor = 1.0

        # current delta angles of 7 motors
        self.delta_angle = np.array([0, 0, 0, 1e-6, pi/2, 1e-6, pi/9])

        self.acceleration_factor = 0.1

        self.update()

    def transform_matrix(self, theta, alpha, a ,d):
        matrix = np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                           [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                           [0, sin(alpha), cos(alpha), d],
                           [0, 0, 0, 1]])
        return matrix


    def rotation_matrix(self, angle_type, theta):
        if angle_type == 0: # x
            return np.array([[1,    0,              0,              0],
                             [0,    cos(theta),     -sin(theta),    0], 
                             [0,    sin(theta),     cos(theta),     0],
                             [0,    0,              0,              1]])

        elif angle_type == 1: # y
            return np.array([[cos(theta),   0,      sin(theta),         0],
                             [0,            1,      0,                  0], 
                             [-sin(theta),  0,      cos(theta),         0],
                             [0,            0,      0,                  1]])

        elif angle_type == 2:
            return np.array([[cos(theta),   -sin(theta),    0,         0],
                             [sin(theta),   cos(theta),     0,         0], 
                             [0,            0,              1,         0],
                             [0,            0,              0,         1]])


    def forward_kinematics(self):
        self.T01 = self.transform_matrix(self.DH_table[0, 0], self.DH_table[0, 1], self.DH_table[0, 2], self.DH_table[0, 3])
        self.T12 = self.transform_matrix(self.DH_table[1, 0] + self.delta_angle[0], self.DH_table[1, 1], self.DH_table[1, 2], self.DH_table[1, 3])
        self.T23 = self.transform_matrix(self.DH_table[2, 0] + self.delta_angle[1], self.DH_table[2, 1], self.DH_table[2, 2], self.DH_table[2, 3])
        self.T34 = self.transform_matrix(self.DH_table[3, 0] + self.delta_angle[2], self.DH_table[3, 1], self.DH_table[3, 2], self.DH_table[3, 3])
        self.T45 = self.transform_matrix(self.DH_table[4, 0] + self.delta_angle[3], self.DH_table[4, 1], self.DH_table[4, 2], self.DH_table[4, 3])
        self.T56 = self.transform_matrix(self.DH_table[5, 0] + self.delta_angle[4], self.DH_table[5, 1], self.DH_table[5, 2], self.DH_table[5, 3])
        self.T67 = self.transform_matrix(self.DH_table[6, 0] + self.delta_angle[5], self.DH_table[6, 1], self.DH_table[6, 2], self.DH_table[6, 3])
        self.T78 = self.transform_matrix(self.DH_table[7, 0] + self.delta_angle[6], self.DH_table[7, 1], self.DH_table[7, 2], self.DH_table[7, 3])

        self.T02 = np.matmul(self.T01, self.T12)
        self.T03 = np.matmul(self.T02, self.T23)
        self.T04 = np.matmul(self.T03, self.T34)
        self.T05 = np.matmul(self.T04, self.T45)
        self.T06 = np.matmul(self.T05, self.T56)
        self.T07 = np.matmul(self.T06, self.T67)
        self.T08 = np.matmul(self.T07, self.T78)

        self.P00 = np.array([0, 0, 0])
        self.P01 = np.array(self.T01[0:3, 3])
        self.P02 = np.array(self.T02[0:3, 3])
        self.P03 = np.array(self.T03[0:3, 3])
        self.P04 = np.array(self.T04[0:3, 3])
        self.P05 = np.array(self.T05[0:3, 3])
        self.P06 = np.array(self.T06[0:3, 3])
        self.P07 = np.array(self.T07[0:3, 3])
        self.P08 = np.array(self.T08[0:3, 3]) 

        # End effector orientation
        self.oz = atan2(self.T08[1, 0], self.T08[0, 0])
        self.oy = atan2(-self.T08[2, 0], (self.T08[0, 0] * cos(self.oz) + self.T08[1, 0] * sin(self.oz)))
        self.ox = atan2((self.T08[0, 2] * sin(self.oz) - self.T08[1, 2] * cos(self.oz)), (self.T08[1, 1] * cos(self.oz) - self.T08[0, 1] * sin(self.oz)))

        self.current_orientation = np.array((self.ox, self.oy, self.oz))

        # End effector position
        self.current_position = np.array((self.P08[0], self.P08[1], self.P08[2]))

    def set_target_orientation(self, ox, oy, oz):
        self.target_orientation = np.array((ox, oy, oz)) * pi / 180

    def get_current_orientation(self):
        return self.current_orientation * 180 / pi

    def jacobian(self):
        Z00 = np.array([0, 0, 1])
        Z01 = np.array(self.T01[0:3, 2])
        Z02 = np.array(self.T02[0:3, 2])
        Z03 = np.array(self.T03[0:3, 2])
        Z04 = np.array(self.T04[0:3, 2])
        Z05 = np.array(self.T05[0:3, 2])
        Z06 = np.array(self.T06[0:3, 2])
        Z07 = np.array(self.T07[0:3, 2])
        Z08 = np.array(self.T08[0:3, 2])

        Jv00 = np.cross(Z00, (self.P08 - self.P00))
        Jv01 = np.cross(Z01, (self.P08 - self.P01))
        Jv02 = np.cross(Z02, (self.P08 - self.P02))
        Jv03 = np.cross(Z03, (self.P08 - self.P03))
        Jv04 = np.cross(Z04, (self.P08 - self.P04))
        Jv05 = np.cross(Z05, (self.P08 - self.P05))
        Jv06 = np.cross(Z06, (self.P08 - self.P06))
        Jv07 = np.cross(Z07, (self.P08 - self.P07))

        row0 = np.append(Z00, Jv00, axis=0)
        row1 = np.append(Z01, Jv01, axis=0)
        row2 = np.append(Z02, Jv02, axis=0)
        row3 = np.append(Z03, Jv03, axis=0)
        row4 = np.append(Z04, Jv04, axis=0)
        row5 = np.append(Z05, Jv05, axis=0)
        row6 = np.append(Z06, Jv06, axis=0)
        row7 = np.append(Z07, Jv07, axis=0)

        transpose_jacobian = np.array((row2, row3, row4, row5, row6, row7))

        jacobian_matrix = np.transpose(transpose_jacobian)

        self.inverse_jacobian = np.linalg.inv(jacobian_matrix)

    def move(self):
        for i in range(1, 7):
            self.delta_angle[i] += self.angular_velocity[i-1]*0.5

    def update(self):
        self.forward_kinematics()
        self.jacobian()