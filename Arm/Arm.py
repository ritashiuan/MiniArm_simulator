import numpy as np
import cv2
import math
from math import *
import global_var as g
# 9/11
ArmMotionEnable=False
Angle2Rad = np.pi / 180.0
Rad2Angle= 180.0 / np.pi
# DH parameters
# a = [0, -0.015, 30.96, -8.84, 0, 0] #cm
# d = [0, 0, 0, 25.473, 0, 0]
# alpha = [0, -90, 0, -90, 90, -90]
# a = [0.0, 0.0, 29.425, 0.0, 0.0, 0.0] #cm
# d = [0.0, 0.0, 0.0, 30.2, 0.0, 0.0]

# alpha = [0.0, -90, 0.0, -90, 90, -90]
# a = [0.0, 0.0, 29.425, 8.9, 0.0, 9.4] # cm
# d = [-5.2, 0.0, 0.0, 0.0, 0.0, -6.185] # cm




total_solutions = []

def rad_to_deg(radian):
    return np.degrees(radian)

def deg_to_rad(degree):
    return np.radians(degree)
# Compute transformation matrix
def transformation_matrix(theta, alpha, a, d):
    alpha_rad = deg_to_rad(alpha)
    return np.array([[np.cos(theta), -np.sin(theta) , 0.0, a],
            [np.sin(theta) * np.cos(alpha_rad), np.cos(theta) * np.cos(alpha_rad), -np.sin(alpha_rad), -d * np.sin(alpha_rad)],
            [np.sin(theta) * np.sin(alpha_rad), np.cos(theta) * np.sin(alpha_rad),  np.cos(alpha_rad),  d * np.cos(alpha_rad)],
            [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
# Jo~J5 :rad /T : cm
def calculate_arm_forward_kinematics(J1, J2, J3, J4, J5, J6):
    # 9/11
    print(f"J0, J1, J2, J3, J4, J5, J6 :{0.0, J1, J2, J3, J4, J5, J6}")

    T01 = transformation_matrix(J1, g.alpha[0], g.a[0], g.d[0])
    T12 = transformation_matrix(J2, g.alpha[1], g.a[1], g.d[1])
    T23 = transformation_matrix(J3, g.alpha[2], g.a[2], g.d[2])
    T34 = transformation_matrix(J4, g.alpha[3], g.a[3], g.d[3])
    T45 = transformation_matrix(J5, g.alpha[4], g.a[4], g.d[4])
    T56 = transformation_matrix(J6, g.alpha[5], g.a[5], g.d[5])
    # T6E = np.array([[1.0, 0.0, 0.0, 0.0],
    #                 [0.0, 1.0, 0.0, 0.0],
    #                 [0.0, 0.0, 1.0, 8.8],
    #                 [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    T = np.eye(4)
    # Compute the final transformation matrix
    T =    g.T_B0 @ T01 @ T12 @ T23 @ T34 @ T45 @ T56 
    # for reality
    # T =  TB0 @ T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T6E
    return T

def limitation(sol,solTF):
    #print(sol)
    anglelist = [(angle - math.pi*2) if angle > math.pi else (angle + math.pi*2) if angle < -math.pi else angle for angle in sol]
    # print(anglelist)      
    # print(anglelist[1])
    return solTF
    if (anglelist[4] > 1.72) or (anglelist[4] < -1.72):
                print("angle5:limit\n")
                return False
            
    else:
        print("angle5:ok\n")
        
        if anglelist[1]>(45 / 180 * math.pi) and anglelist[1]<(135 / 180 * math.pi):
        
            print("angle2:limit\n")
            return False
        else:
            print("angle2:ok\n")
            # print(anglelist[2])
            # if (anglelist[2] > 1.124) or (anglelist[2] < -1.918):
            #     print("angle3:limit\n")
            #     return False
            # elif (anglelist[2]< 1.124) and (anglelist[2] > 0.661):
            #     print("Danger Zone\n")
            #     if (anglelist[4] >= 0):
            #         if (anglelist[2]> 1.025):
            #             print("angle5:limit, base is in a collision with end effector \n")
            #             return False
            #         else:
            #             return solTF
            #     else:
            #         print("angle3:ok\n")
            #         return solTF
                    
            # else:
            #     print("angle3:ok\n")
            return solTF
               
    

def arm_InverseKinematics(T):
    global ArmMotionEnable, total_solutions, sol_TF
    
    total_solutions = []
    # T_6E = np.array([[1.0, 0.0, 0.0, 0.0],
    #                 [0.0, 1.0, 0.0, 0.0],
    #                 [0.0, 0.0, 1.0, 8.8],
    #                 [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    T_O6 = np.dot(np.linalg.inv(g.T_B0), T)
    # T_O6 = np.dot(T_OE, np.linalg.inv(T_6E))
    nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
    ox, oy, oz = T[0, 1], T[1, 1], T[2, 1]
    ax, ay, az = T[0, 2], T[1, 2], T[2, 2]
    px, py, pz = T_O6[0, 3], T_O6[1, 3], T_O6[2, 3]
    print(f"px, py, pz: {px, py, pz}")
    theta1 = np.zeros(2)
    theta2 = -999
    theta3 = np.zeros(2)
    theta4 = np.zeros(2)
    theta5 = np.zeros(2)
    theta6 = np.zeros(2)
    
     # J1 解
    for i in range(2):
        theta1[i] = np.arctan2(py, px) - np.arctan2(0, math.pow(-1, i) * np.sqrt(math.pow(px,2) +  math.pow(py,2)))#theta1:rad
    
    for i in range(2):
        # J3 solution
        k1 = np.cos(theta1[i]) * px + np.sin(theta1[i]) * py - g.a[1]
        k2 = -pz + g.d[0]
        k = (math.pow(k1, 2) + math.pow(k2, 2) - (math.pow(g.a[2], 2) + math.pow(g.a[3], 2) + math.pow(g.d[3], 2))) / (2 * g.a[2])
        print(f"k1{k1} k2{k2} k{k}\n")
        
        for j in range(2):
            # J3 solution
            if not math.isnan((np.arctan2(g.a[3], g.d[3]) - np.arctan2(k, math.pow(-1, j) * np.sqrt(math.pow(g.a[3], 2) + math.pow(g.d[3], 2) - math.pow(k, 2))))):
                theta3[j] = np.arctan2(g.a[3], g.d[3]) - np.arctan2(k, math.pow(-1, j) * np.sqrt(math.pow(g.a[3], 2) + math.pow(g.d[3], 2) - math.pow(k, 2)))
            else:
                print("theta3 Nan ~~")
            if theta3[j] > (np.pi / 2):
                theta3[j] -= 2 * np.pi

            o1 = g.d[3] - np.sin(theta3[j]) * g.a[2]
            o2 = np.sin(theta1[i]) * py - g.a[1] + np.cos(theta1[i]) * px
            o3 = np.cos(theta3[j]) * g.a[2] + g.a[3]
            o4 = g.d[0] - pz
            j1 = g.a[1] - np.cos(theta1[i]) * px - np.sin(theta1[i]) * py
            j2 = np.sin(theta1[i]) * py - g.a[1] + np.cos(theta1[i]) * px
            j3 = g.d[0] - pz
            k1 = (o1 * o2 - o3 * o4) / (j1 * j2 - math.pow(j3, 2))

            o1 = g.d[3] - np.sin(theta3[j]) * g.a[2]
            o2 = g.d[0] - pz
            o3 = np.cos(theta3[j]) * g.a[2] + g.a[3]
            o4 = g.a[1] - np.cos(theta1[i]) * px - np.sin(theta1[i]) * py
            j1 = g.d[0] - pz
            j2 = np.sin(theta1[i]) * py - g.a[1] + np.cos(theta1[i]) * px
            j3 = g.a[1] - np.cos(theta1[i]) * px - np.sin(theta1[i]) * py
            k2 = (o1 * o2 - o3 * o4) / (math.pow(j1, 2) - j2 * j3)

            theta2 = np.arctan2(k1, k2) - theta3[j]
           
            if theta2 >( np.pi / 2):
                theta2 -= 2 * np.pi
            if theta2 < (-3 * np.pi / 2):
                theta2 += 2 * np.pi

            # angle 4~6
            # 建立 4x4 的變換矩陣
            t_23 = transformation_matrix(theta3[j], g.alpha[2], g.a[2], g.d[2])

            t_12 = transformation_matrix(theta2, g.alpha[1], g.a[1], g.d[1])

            t_01 = transformation_matrix(theta1[i], g.alpha[0], g.a[0], g.d[0])

            t_03 = t_01 @ t_12 @ t_23
            r_03 = t_03[:3, :3]
            r_x3 = np.array([
                [1.0, 0.0, 0.0],
                [0.0, np.cos(-90 * Angle2Rad), -np.sin(-90 * Angle2Rad)],
                [0.0, np.sin(-90 * Angle2Rad), np.cos(-90 * Angle2Rad)]
            ])
                
            r_06 = np.array([
                [nx, ox, ax],
                [ny, oy, ay],
                [nz, oz, az]
            ])
            inverse_matrix = np.linalg.inv(np.dot(r_03, r_x3))
            rzyz = np.dot(inverse_matrix, r_06)

            for l in range(2):
                theta5 = math.pow(-1, l) * np.arctan2(np.sqrt(math.pow(rzyz[2, 0], 2) + math.pow(rzyz[2, 1], 2)), rzyz[2, 2])
                theta4 = np.arctan2(rzyz[1, 2] / np.sin(theta5), rzyz[0, 2] / np.sin(theta5))
                theta6 = np.arctan2(rzyz[2, 1] / np.sin(theta5), -rzyz[2, 0] / np.sin(theta5))
     

            total_solutions.append([theta1[i], theta2, theta3[j], theta4, theta5, theta6]) 

    print("Total solution => \n")
    for idx, sol in enumerate(total_solutions):
        print(f"Solution {idx + 1} : {' '.join([f'{rad_to_deg(ang) :.3f}' for ang in sol])}")
            
  
    check = True
    sol_TF = [False for _ in range(len(total_solutions))]
    for choose in range(len(total_solutions)):
        
        J1, J2, J3, J4, J5, J6 = total_solutions[choose]
        print(f"IF SOL: {rad_to_deg(J1) :.3f} {rad_to_deg(J2) :.3f} {rad_to_deg(J3) :.3f} {rad_to_deg(J4 ):.3f} {rad_to_deg(J5):.3f} {rad_to_deg(J6):.3f}")
        
        tmpT = calculate_arm_forward_kinematics( J1, J2, J3, J4, J5, J6)
        check = True
        for o in range(4):
            for p in range(4):
                if abs(round(tmpT[o, p]) - round(T[o, p])) > 1:
                    check = False
                    print(f"{choose}: forward kinematics is different from inverse kinematics")
                    print(f'tmpT: {tmpT}')
                    print(f'rotation_matrix: {T}')
                    break
            if not check:
                break
            print(f'tmpT: {tmpT}')
        if check:
            sol_TF[choose]=True
            
            print(f"{choose}: ok!")
    
    J1 = J2 = J3 = J4 = J5 = J6 = 0.0
    output = 0        
    for output in range(len(total_solutions)):
        print("=========================================\n")
        print(f"Check solution {output} :\n")
        sol_TF[output]=limitation(total_solutions[output],sol_TF[output])
        print("=========================================\n")
        if sol_TF[output]:
            J1, J2, J3, J4, J5, J6 = total_solutions[output]
            print(f"SOL(deg): {rad_to_deg(J1) :.3f} {rad_to_deg(J2) :.3f} {rad_to_deg(J3) :.3f} {rad_to_deg(J4 ):.3f} {rad_to_deg(J5):.3f} {rad_to_deg(J6):.3f}")

            print("\nForward kinematics is same as inverse kinematics")
            temp_theta = [rad_to_deg(J1) , rad_to_deg(J2) , rad_to_deg(J3) , rad_to_deg(J4) , rad_to_deg(J5) , rad_to_deg(J6) ]
        
            temp_theta = [(angle - 360.0) if angle > 180 else (angle + 360.0) if angle < -180 else angle for angle in temp_theta]
            print(f"OUTPUT SOL(deg): {(temp_theta[0]) :.3f} {(temp_theta[1]) :.3f} {(temp_theta[2]) :.3f} {(temp_theta[3] ):.3f} {(temp_theta[4]):.3f} {(temp_theta[5]):.3f}")

            Solution = np.array(temp_theta, dtype=float)
            ArmMotionEnable = True
            break
    if J1 == J2 == J3 == J4 == J5 == J6 == 0.0:
        Solution = np.zeros(6, dtype=float)
        ArmMotionEnable = False
    return Solution    
       
def TransRotate(ox, oy, oz):
    
    print(f"ox {ox} oy{oy} oz{oz}\n")
    x_angle = deg_to_rad(ox)

    y_angle = deg_to_rad(oy)
    z_angle = deg_to_rad(oz)

    temp_nx = np.cos(z_angle) * np.cos(y_angle)
    temp_ny = np.sin(z_angle) * np.cos(y_angle)
    temp_nz = -np.sin(y_angle)
    temp_ox = np.cos(z_angle) * np.sin(y_angle) * np.sin(x_angle) - np.sin(z_angle) * np.cos(x_angle)
    temp_oy = np.sin(z_angle) * np.sin(y_angle) * np.sin(x_angle) + np.cos(z_angle) * np.cos(x_angle)
    temp_oz = np.cos(y_angle) * np.sin(x_angle)
    temp_ax = np.cos(z_angle) * np.sin(y_angle) * np.cos(x_angle) + np.sin(z_angle) * np.sin(x_angle)
    temp_ay = np.sin(z_angle) * np.sin(y_angle) * np.cos(x_angle) - np.cos(z_angle) * np.sin(x_angle)
    temp_az = np.cos(y_angle) * np.cos(x_angle)

    # 創建 3x3 的旋轉矩陣
    data = np.array([[temp_nx, temp_ox, temp_ax],
                     [temp_ny, temp_oy, temp_ay],
                     [temp_nz, temp_oz, temp_az]], dtype=np.float32)
    
    print(f'data: {data}')
    return data


def GotoPosition(ox, oy, oz, x, y, z): #cm
    T = TransRotate(ox, oy, oz)
    
    # 創建 4x4 的變換矩陣
    tmp = np.array([[T[0, 0], T[0, 1], T[0, 2], float(x)],
                     [T[1, 0], T[1, 1], T[1, 2], float(y)],
                     [T[2, 0], T[2, 1], T[2, 2], float(z)],
                     [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    
    # print(f"tmp:{tmp}")
    result = arm_InverseKinematics(tmp)
    
    if ArmMotionEnable:
        print("[miniArm] Arm arrival!")
        
        return result
        
    else:
        print("[ScaraArm] Arm fail to arrive!")
        return result