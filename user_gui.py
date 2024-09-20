
import tkinter as tk
from tkinter import ttk
import pybullet as p
import global_var as g
import random
import numpy as np
#window

root = tk.Tk()
x = tk.StringVar()
y = tk.StringVar()
z = tk.StringVar()
ans = tk.StringVar()
StartPos = [0, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])

root.title('MiniArm')        # 設定標題
root.configure(background='#ffffe0')   # 設定背景色
root.iconbitmap('robot-arm.ico')  # 設定 icon ( 格式限定 .ico )

window_width = root.winfo_screenwidth()    # 取得螢幕寬度
window_height = root.winfo_screenheight()  # 取得螢幕高度
width = 800
height = 600
left = int((window_width - width)/2)       # 計算左上 x 座標
top = int((window_height - height)/2)      # 計算左上 y 座標
root.geometry(f'{width}x{height}+{left}+{top}')
root.resizable(False, False)   # 設定畫面 x 方向和 y 方向都不能縮放

# RobotId = p.loadURDF('mini_arm_URDF_V12/urdf/mini_arm_URDF_V12.urdf', StartPos, StartOrientation,useFixedBase=True)

def openfile(mode):
    global urdfPath, num_link, RobotId, number_of_joints, current_positions, a, d, alpha
    if mode == 0 :
        g.urdfPath="urdf_files_dataset/urdf_files/robotics-toolbox/puma560_description/urdf/puma560_robot.urdf"
        print("Loading puma560_robot modle...")
        g.RobotId = p.loadURDF(g.urdfPath, StartPos, StartOrientation,useFixedBase=True)
        g.a= [0.0, 0.0, 43.18, 0.0, 0.0, 0.0] #cm
        g.d = [0.0, 0.0, 15.01, 0.0, 43.31, 0.0]
       
        g.alpha = [0.0, -90, 0.0, -90, 90, -90]
        g.T_B0 =  np.array([[1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 67.18],
                    [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    elif mode == 1:
        # 9/15
        g.urdfPath="mini_arm_URDF_V14/urdf/mini_arm_URDF_V14.urdf"
        # g.urdfPath="mini_arm_URDF_V10/urdf/mini_arm_URDF_V10.urdf"
        print("Loading mini_arm_URDF modle...")
        g.RobotId = p.loadURDF(g.urdfPath, StartPos, StartOrientation,useFixedBase=True)
        
        g.a= [0.0, 0.0, 29.425, 0.0, 0.0, 0.0] #cm
        g.d = [0.0, 0.0, 0.0, 30.2, 0.0, 0.0]

        g.alpha = [0.0, -90, 0.0, -90, 90, -90]
        
        g.T_B0 =  np.array([[1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 16.1],
                    [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    g.number_of_joints = p.getNumJoints(g.RobotId)
    g.current_positions = [p.getJointState(g.RobotId, i)[0] for i in range(g.number_of_joints)]
    for i in range(0,g.number_of_joints):
        print(f"{i}:{g.current_positions[0]}")
    for joint_number in range(g.number_of_joints):
        info = p.getJointInfo(g.RobotId, joint_number)
        print(info[0], ": ", info[1])

  
    for joint in range(g.number_of_joints):
        p.setCollisionFilterPair(g.RobotId, g.planeId, joint, -1, enableCollision=0)

def clear():
    p.removeBody(g.RobotId)
 

 #每 10 毫秒調用一次 p.stepSimulation() 来更新 PyBullet   
def update_simulation():
    for _ in range(10):  # 進行多步模擬
        p.stepSimulation()
    root.after(1000, update_simulation)  # 增加延遲时间，使得運動更可見
def r_history():
    try:
        # 使用上下文管理器打开文件
        with open('pos.txt', 'r') as f1:
            pos_str = f1.read().strip()  # 读取文件内容并去掉首尾空白字符
            
            # 分割字符串并转换为整数（假设文件内容格式正确）
            x_str, y_str, z_str = pos_str.split(",")
            x_pre = float(x_str)
            y_pre = float(y_str)
            z_pre = float(z_str)
            x.set(x_pre)
            y.set(y_pre)
            z.set(z_pre)
            print(f"x_pre: {x_pre}, y_pre: {y_pre}, z_pre: {z_pre}")
            
    except FileNotFoundError:
        print("文件未找到。")
    except ValueError:
        print("数据转换错误。请检查文件内容是否符合预期格式。")
    except Exception as e:
        print(f"发生了一个错误: {e}")
def r_random():
    x_ran = round(random.uniform(-40.0, 40.0),2)
    y_ran = round(random.uniform(-40.0, 40.0), 2)
    z_ran = round(random.uniform( 10.0, 40.0), 2)
    
    x.set(x_ran)
    y.set(y_ran)
    z.set(z_ran)
    print(f"x_ran: {x_ran}, y_ran: {y_ran}, z_ran: {z_ran}")
    

def GUI_set_mode(Run, reset, dis):
    global x, y, z, ans, root
    
    mylabel = tk.Label(root, 
                   text='MiniArm GUI',
                   font=('Times New Roman',18,'bold'),
                   background='#ffffe0')
    mylabel.place(x=330, y=0)


    coor_label = tk.Label(root, 
                    text='x coor.(cm)\n\ny coor.(cm)\n\nz coor.(cm)\n',
                    font=('Times New Roman',14),
                    background='#ffffe0'

                    )
    coor_label.place(x=230, y=80)

    entry_x=tk.Entry(root,width=20,textvariable=x)
    entry_x.place(x=330, y=85)  # 放入單行輸入框

    entry_y=tk.Entry(root,width=20,textvariable=y)
    entry_y.place(x=330, y=125)  # 放入單行輸入框
    entry_z=tk.Entry(root,width=20,textvariable=z)
    entry_z.place(x=330, y=165)  # 放入單行輸入框
    # 创建按钮并绑定到读取文件的函数
    read_button = tk.Button(root, text="History", background='#deb887', command=r_history)
    read_button.place(x=330, y=205)

    # 创建按钮并绑定到读取文件的函数
    ran_button = tk.Button(root, text="Random", background='#deb887', command=r_random)
    ran_button.place(x=380, y=205)
    btn = tk.Button(root,
                    font=('Times New Roman',14,'bold'),
                    padx=20,
                    pady=5,
                    text='Run',
                    activeforeground='#8b4513',
                    background='#deb887',
                    command=lambda:Run(root))     # 建立 Button 按鈕
    btn.place(x=340, y=500)  

    btn_reset=tk.Button(root,
                    font=('Times New Roman',14,'bold'),
                    padx=20,
                    pady=5,
                    text='Reset',
                    activeforeground='#8b4513',
                    background='#deb887',
                    command=lambda:reset())     # 建立 Button 按鈕
    btn_reset.place(x=200, y=500)  
    btn_show=tk.Button(root,
                    font=('Times New Roman',14,'bold'),
                    padx=20,
                    pady=5,
                    text='Show',
                    activeforeground='#8b4513',
                    background='#deb887',
                    command=lambda:dis())     # 建立 Button 按鈕
    btn_show.place(x=480, y=500)  
   
    root.after(10, update_simulation)  # Start the simulation loop
    # 在运动完成后清除显示
    # 在运动完成后隐藏 Label
    
    #root.after(2000, lambda: answer.place_forget())  # 等待 2 秒后隐藏文本
    root.mainloop()
def w_history(x_value , y_value , z_value):
    f1 = open('pos.txt','w')
    f1.write(f"{str(x_value)},{str(y_value)},{str(z_value)}")
    f1.close()
def Record():
    print("Recordinng...")
    f1 = open('note.txt','w')
    f1.write('goal angles:')     # 寫入 good morning
    f1.write(str(g.listAngle))
    f1.write('\ncurrent angles:')     # 寫入 good morning
    f1.write(str(g.current_positions_after))
    f1.close()


def initial(Run, reset, dis):
    global root
    menu = tk.Menu(root)
    
    menubar_1 = tk.Menu(menu, tearoff=False)                        # 建立第一個選單的子選單，有三個選項
                 # 子選單第一個選項

    menubar_1.add_command(label="Clear", command=clear)              # 子選單第二個選項
    # menubar_1.add_command(label="Exit")              # 子選單第三個選項
    

    menubar_1more = tk.Menu(menubar_1,tearoff=False)              # 建立子選單內的子選單，有三個選項
    menubar_1more.add_command(label="Puma560",command= lambda:openfile(0))            # 子選單的子選單的第一個選項
    menubar_1more.add_command(label="Mini Arm",command= lambda:openfile(1))            # 子選單的子選單的第二個選項
    menubar_1.add_command(label="Record", command=Record) 
    menubar_1.add_cascade(label='Open', menu=menubar_1more)
    menu.add_cascade(label='File', menu=menubar_1)   # 建立第一個選單 File，綁定子選單
    # menubar_2 = tk.Menu(menu)                        # 建立第一個選單的子選單，有三個選項
    # # menubar_2.add_command(label="mode_R", command= lambda:set_mode())              # 子選單第一個選項
    # menubar_2.add_command(label="Save",command=save)              # 子選單第二個選項
    # # menubar_2.add_command(label="Exit")              # 子選單第三個選項
    # menu.add_cascade(label='Setting', menu=menubar_2)   # 建立第一個選單 File，綁定子選單
    root.config(menu=menu)                           # 綁定選單 
    GUI_set_mode(Run, reset, dis)
    root.mainloop()
