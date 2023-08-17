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
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos)





textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = useProgrammatic
import random
random.seed(10)

heightPerturbationRange = 0.025


if heightfieldSource==useDeepLocoCSV:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.2],fileName = "heightmaps/ground2.txt", heightfieldTextureScaling=128)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])

if heightfieldSource==useTerrainFromPNG:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.1],fileName = "heightmaps/Maze.png")
  textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
  terrain  = p.createMultiBody(0, terrainShape)
  p.changeVisualShape(terrain, -1, textureUniqueId = textureId)
 
 
#p.changeVisualShape(terrain, -1, rgbaColor=[0.75,0.75,0.75,1],textureUniqueId = textureId)
#p.changeVisualShape(terrain, -1, textureUniqueId = textureId)
def rpy2quaternion(roll, pitch, yaw):
    x=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    y=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    z=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    w=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    ori=np.array([x,y,z,w])
    return ori

height=0.05
length=0.2
width=0.1
base_position=[0,0.5,0.025]
hal_geo=[width, width, height]
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=hal_geo)
visualID=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=hal_geo) 
ori=  rpy2quaternion(np.pi/18,0,0)                             
box=p.createMultiBody(0, colBoxId,visualID,base_position,ori)

height=0.025
length=5
width=0.025
base_position=[0,0,0]
hal_geo=[length, width, height]
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=hal_geo)
visualID=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=hal_geo) 
ori=  rpy2quaternion(0,0,0)                             
box=p.createMultiBody(0, colBoxId,visualID,base_position,ori)

for i in range(10):
  base_position=[0,0.2*i,0.025]
  #box=p.createMultiBody(0, colBoxId,visualID,base_position,ori)
'''
np.random.seed(1)
x_range=np.arange(-width*2*10,width*2*10,width*2) 
y_range=np.arange(0,width*2*20,width*2) 
for i in range(20):
  for j in range(20):
    if j<10:
      hei_b=np.random.random()*0.02*(2)-height
    else:
      hei_b=np.random.random()*0.02*(2)-height+0.02
    base_position=[x_range[i],y_range[j],hei_b]    
    box=p.createMultiBody(0, colBoxId,visualID,base_position)                            
box=p.createMultiBody(0, colBoxId,visualID,base_position)
box=p.createMultiBody(0, colBoxId,-1,base_position,)
'''
#box=p.createMultiBody(0, colBoxId,-1,[0,5,0.075])
#p.changeVisualShape(box, -1, rgbaColor=[1,1,1,1],textureUniqueId = textureId)

height=0.2 
length=0.3
step_width=0.05
step_height=0.03
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            
hal_geo=[step_width, step_width, height]
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=hal_geo)
visualID=p.createVisualShape(p.GEOM_BOX,halfExtents=hal_geo) 
box =[]        
np.random.seed(4)
x_range=np.arange(-step_width*(2*10-1),step_width*(2*10+1),step_width*2) 
y_range=np.arange(step_width*(2*2-1),step_width*(2*22-1),step_width*2)
height_map=np.zeros((20,20)) 
for i in range(20):
  for j in range(20):
    if j<10:
      hei_b=np.random.random()*step_height*(2)-height
    else:
      hei_b=np.random.random()*step_height*(2)-height+step_height
    base_position=[x_range[i],y_range[j],hei_b] 
    height_map[i,j]=hei_b+height
    box1=p.createMultiBody(0, colBoxId,visualID,base_position)
    box.append(box1)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)



while 1:
    p.stepSimulation()
    
    #GEOM_CONCAVE_INTERNAL_EDGE may help avoid getting stuck at an internal (shared) edge of the triangle/heightfield.
    #GEOM_CONCAVE_INTERNAL_EDGE is a bit slower to build though.
    #flags = p.GEOM_CONCAVE_INTERNAL_EDGE
    flags = 0
    
