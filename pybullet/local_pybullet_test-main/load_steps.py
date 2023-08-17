import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
from time import time,sleep
import math
import random
random.seed(10)
p.connect(p.GUI) # 连接到仿真服务器
p.setGravity(0, 0, -9.8) # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath()) # 设置pybullet_data的文件路径
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
# 加载地面
floor = p.loadURDF("plane.urdf")
Pos=[-0.1,0,0.2]
box=p.loadURDF("box_80_60_30/box2.urdf",Pos)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                             cameraPitch=-40, cameraTargetPosition=[0.2, -0.2, 1])

# mini cheetah的初始位置
startPos = [0, 0, 0.3]


# 加载urdf文件

l=[0,0]
for i in range(10):
    f=np.random.choice([2,3,5,8,10],p=[0.34,0.35,0.25,0.04,0.02])
    print("f",f)


startPos = [0, 0, 0.1]


# 加载urdf文件
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos)


if 1:
    if 1:
        if 1:
                 
            height=0.2 
            length=0.3
            step_width=0.1
            random_seed=10
            
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            
            hal_geo=[step_width, step_width, height]
            colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=hal_geo)
            visualID=p.createVisualShape(p.GEOM_BOX,
                                            halfExtents=hal_geo) 
            
            np.random.seed(random_seed)
            x_range=np.arange(-step_width*(2*10-1),step_width*(2*10+1),step_width*2) 
            y_range=np.arange(step_width*(2*2-1),step_width*(2*22-1),step_width*2)
            step_start_place=[x_range[0]-step_width,y_range[0]-step_width]
            height_map=np.zeros((20,20)) 
            for i in range(20):
                for j in range(20):
                    if j<10:
                        height_cm=np.random.choice([2,3,5,8,10],p=[0.3,0.3,0.25,0.1,0.05])
                        hei_b=height_cm*0.01-height
                    else:
                        height_cm=np.random.choice([2,3,5,8,10],p=[0.3,0.25,0.2,0.15,0.1])
                        hei_b=height_cm*0.01-height
                    base_position=[x_range[i],y_range[j],hei_b] 
                    height_map[i,j]=hei_b +height  
                    box=p.createMultiBody(0, colBoxId,visualID,base_position)
                    #box.append(box)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

            a=1



