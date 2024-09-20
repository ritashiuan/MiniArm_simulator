import tkinter as tk
import pybullet as p
import pybullet_data
from Arm.Arm import *
from user_gui import *
import numpy as np
import time
import global_var as g
np.set_printoptions(precision= 3)
# 9/11
physicsClient = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=1 --background_color_blue=0.9')#設定物理引擎 # Connect to pybullet GUI
#p.setGravity(0, 0, -9.8) #設定重力加速度 (gravity force along the X world axis,gravity force along the Y world axis,gravity force along the Z world axis)

p.setRealTimeSimulation(0)# 設定關閉及時模擬,允许你控制模擬更新的频率
# 添加資源路徑，內建urdf檔案的路徑添加進來
p.setAdditionalSearchPath(pybullet_data.getDataPath())
g.planeId = p.loadURDF("plane.urdf")


p.resetDebugVisualizerCamera(
                            cameraDistance=2,            # 相機與目標位置之間的距離(m)
                            cameraYaw=0, # 水平旋轉角度,0 度代表正視，90 度表示從右側觀察，-90 度則表示從左側觀察,角度單位(度)
                            cameraPitch=-15, # 垂直旋轉角度 0 度表示水平視角，負值表示向下看，正值表示向上看 角度單位
                            cameraTargetPosition=(0, 0, 0.55) # 相機瞄準的位置
                            )


# RobotId = p.loadURDF(urdfPath, StartPos, StartOrientation,useFixedBase=True)

x_value,y_value,z_value=0.0,0.0,0.0
# endEffectorIndex = 5
# 取得機器人目前的關節位置

def Kinematics(px,py,pz):
    
    ox= math.pi/2
    oy= 0.0
    oz= 0.0
    #9/15
    #J0,J1,J2,J3,J4,J5=GotoPosition(ox, oy, oz, px, py, pz)
    J1,J2,J3,J4,J5, J6=GotoPosition(ox, oy, oz, px, py, pz)
    return [deg_to_rad(J1),deg_to_rad(J2),deg_to_rad(J3),deg_to_rad(J4),deg_to_rad(J5), deg_to_rad(J6)]




# def Angledebug():
#     global RobotId,number_of_joints
#     joint_params = [p.addUserDebugParameter(f'j{i+1}', -math.pi, math.pi, 0) for i in range(g.number_of_joints)]
#     btn_params = [p.addUserDebugParameter(f'btn{i+1}', 1, 0, 0) for i in range(g.number_of_joints)]
#     btn_values = [p.readUserDebugParameter(btn) for btn in btn_params]

#     while True:
#         for i in range(g.number_of_joints):
#             joint_value = p.readUserDebugParameter(joint_params[i])
#             #print(f'joint{i+1} value: {joint_value}\n')
#             set_motor(i, joint_value)
#             for _ in range(3):  # 多次调用仿真步骤
#                 p.stepSimulation()

#             # 检查按钮状态并重置机器人
#             current_btn_value = p.readUserDebugParameter(btn_params[i])
#             if current_btn_value != btn_values[i]: #如果沒有如預期到達一樣數值，按下按鈕會執行回到初始位置與角度
#                 p.resetBasePositionAndOrientation(g.RobotId, StartPos, StartOrientation)
#                 btn_values[i] = current_btn_value  # 更新按钮值

# def start_angle_debug():
#     angle_debug_thread = threading.Thread(target=Angledebug) #創建新線程
#     angle_debug_thread.daemon = True #主程式退出即停止
#     angle_debug_thread.start()

def reset():
    global current_positions,ans,RobotId,number_of_joints
    for i in range(0,g.number_of_joints):
        print(i)
        set_motor(i,0.0)
    current_positions = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]
    ans.set("Reset...")
def EnableMotor(list):
    global RobotId,number_of_joints
    # 获取当前所有关节的位置
    
    for index, value  in enumerate(list):
        #print(f'{index},{value}\n')
        set_motor(index,value)
    for _ in range(20):  # 多次调用仿真步骤
        p.stepSimulation()
    # 获取当前所有关节的位置
   
    current_positions_after = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]
    
    print(f'Positions after setting: {current_positions_after}')
    # # 在运动完成后显示 "arrival"
    # root.after(5000, lambda: ans.set("arrival"))  # 5 秒后显示 "arrival"
    return current_positions_after
def set_motor(jointID,joint_value):
    global answer,x,y,z,ans, RobotId
    print(f'Setting joint {jointID} to {joint_value}\n')
    p.setJointMotorControl2(
        bodyUniqueId=g.RobotId,     # 機器人的 ID
        jointIndex=jointID,             # 要控制的關節索引
        controlMode=p.POSITION_CONTROL, # 控制模式為位置控制
        targetPosition=joint_value,       # 目標位置 (弧度)
        force = 150                 # 最大作用力
    )
    
    x.set('')
    y.set('')
    z.set('')
    
def Run(root):

    print("Run function called")
    global answer,x,y,z,ans,x_value,y_value,z_value, listAngle
    try:
        # 将读取到的值转换为 float 类型
        x_value = float(x.get())
        y_value = float(y.get())
        z_value = float(z.get())
        w_history(x_value , y_value , z_value)
        print(f"goal: {x_value},{y_value},{z_value}")
        x.set('')
        y.set('')
        z.set('')
        goal_text = f'goal: {x_value}, {y_value}, {z_value}\n Running...'
        ans.set(goal_text)  
        if 'answer' not in globals():
            answer = tk.Label(root, 
                            textvariable = ans,
                            font=('Times New Roman',18,'bold'),
                            background='#a0522d'
                            )   # 放入標籤
            answer.place(x=250, y=250)
        else:
            answer.config(textvariable = ans)    
          
        g.listAngle=Kinematics(x_value,y_value,z_value)
        
        current_positions_after=EnableMotor(g.listAngle)
       
    except ValueError:

        print("Invalid input: Please enter a valid number")
        x.set('')
        y.set('')
        z.set('')
        ans.set('Answer:\nInvalid input: Please enter a valid number\n')
        if 'answer' not in globals():
            answer = tk.Label(root, 
                              textvariable= ans,
                              font=('Times New Roman', 18, 'bold'),
                              background='#a0522d')
            answer.place(x=170, y=350)
        else:
            answer.config(textvariable= ans)
        
        
      
# 函數：在指定位置添加坐標軸
def add_coordinate_axes(base_position, axis_length=0.1):
    base_position=[base_position[i]/100 for i in range(3)]
    print(f"base_position:{base_position}")
    p.addUserDebugLine(base_position, [base_position[0] + axis_length, base_position[1], base_position[2]], lineColorRGB=[1, 0, 0], lineWidth=4, lifeTime=0)  # X 軸
    p.addUserDebugLine(base_position, [base_position[0], base_position[1] + axis_length, base_position[2]], lineColorRGB=[0, 1, 0], lineWidth=4, lifeTime=0)  # Y 軸
    p.addUserDebugLine(base_position, [base_position[0], base_position[1], base_position[2] + axis_length], lineColorRGB=[0, 0, 1], lineWidth=4, lifeTime=0)  # Z 軸

# 函數：在指定位置添加文本標籤
def add_coordinate_label(position, label):
    position=[position[i]/100 for i in range(3)]
    p.addUserDebugText(label, position, textColorRGB=[0, 0, 0], textSize=2)



def dis():
    global x_value,y_value,z_valueG
    time.sleep(2. ) 
    g.current_positions_after = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]
    g.current_positions_after = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]
    g.current_positions_after = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]

   
    # 9/11
    # 使用 getLinkState 來獲取末端執行器的正向運動學結果
    end_effector_state = p.getLinkState(g.RobotId, g.endEffectorIndex)
    end_effector_pos = end_effector_state[0]  # 末端點位置 (x, y, z)
    end_effector_orn = end_effector_state[1]  # 末端點方向 (四元數)

    # 獲取位置 (position) 和姿態 (orientation)
    end_effector_position = end_effector_state[4]  # 末端執行器的世界坐標位置
    end_effector_orientation = end_effector_state[5]  # 末端執行器的世界坐標姿態（四元數）
    # end_effector_position_ToBase = end_effector_state[0] # m
    end_effector_position = [end_effector_position[i]*100. for i in range(3)] # cm
    # end_effector_position_ToBase = [end_effector_position_ToBase[i]*100. for i in range(3)] # cm
    print(f"End-Effector Position(cm): {end_effector_position}")
    # print(f"we calculate pos: ")
    print(f"End-Effector Orientation: {end_effector_orientation}")
    # print(f"End-Effector Position relative to base(cm): {end_effector_position_ToBase}") 
    x_dis=(x_value-end_effector_position[0]) 
    y_dis=(y_value-end_effector_position[1])
    z_dis=(z_value-end_effector_position[2])
    # x_dis=(x_value-pos[0])
    # y_dis=(y_value-pos[1])
    # z_dis=(z_value-pos[2])
    # 获取当前所有关节的位置
    # 劃出下面位置
    # add_coordinate_axes(end_effector_position)
    # add_coordinate_label(end_effector_position, "Point C")

    print("==================================================\n")
    print(f'Positions after setting: {g.current_positions_after }')
    # 9/15

    end_effector_rot_matrix = p.getMatrixFromQuaternion(end_effector_orn)
    # 生成轉換矩陣

    t_matrix = np.eye(4)  # 4x4 單位矩陣
    t_matrix[0:3, 0:3] = np.array(end_effector_rot_matrix).reshape(3, 3)  # 旋轉部分
    t_matrix[0:3, 3] = np.array(end_effector_pos)  # 位置部分
    np.set_printoptions(precision= 3)
    print("末端點相對基座的轉換矩陣:")
    print(f"{t_matrix}")
    T=calculate_arm_forward_kinematics(g.current_positions_after[0], g.current_positions_after[1], g.current_positions_after[2],g.current_positions_after[3],g.current_positions_after[4],g.current_positions_after[5])
   
    
    pos=[T[0,3],T[1,3],T[2,3]]
    F_xdis=(pos[0]-end_effector_position[0]) 
    F_ydis=(pos[1]-end_effector_position[1]) 
    F_zdis=(pos[2]-end_effector_position[2]) 
    print(f"T:{T}")
    print(f'angle goal : {g.listAngle}\n')
    print(f'angle goal(deg) : {rad_to_deg(g.listAngle)}\n')
    print(f'Joint State after setting: {g.current_positions_after}\n')
    print("==================================================\n")
    
    #dis_text = f'goal_pos:{x_value:.3f}, {y_value:.3f}, {z_value:.3f}\n now_pos:{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}\ndis_diff_pos: {x_dis:.3f}, {y_dis:.3f}, {z_dis:.3f}\n '
    dis_text = f'goal_pos(cm):{x_value:.3f}, {y_value:.3f}, {z_value:.3f}\n we calculate pos: {pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}\n now_pos(cm):{end_effector_position[0]:.3f}, {end_effector_position[1]:.3f}, {end_effector_position[2]:.3f}\n 正向誤差(cm): {F_xdis:.3f}, {F_ydis:.3f}, {F_zdis:.3f}\n 逆向誤差(cm): {x_dis:.3f}, {y_dis:.3f}, {z_dis:.3f}\n '
    ans.set(dis_text)
    
    
if __name__ == "__main__":
    
   
    initial(Run, reset, dis)
    
    
       
