import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = useDeepLocoCSV
import random
random.seed(10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
heightPerturbationRange = 0.025
if heightfieldSource==useProgrammatic:
  numHeightfieldRows = 256
  numHeightfieldColumns = 256
  heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
  for j in range (int(numHeightfieldColumns/4)):
    for i in range (int(numHeightfieldRows/4) ):
      height = random.uniform(0,heightPerturbationRange)
      heightfieldData[4*i+4*j*numHeightfieldRows]=height
      heightfieldData[4*i+1+4*j*numHeightfieldRows]=height
      heightfieldData[4*i+(4*j+1)*numHeightfieldRows]=height
      heightfieldData[4*i+1+(4*j+1)*numHeightfieldRows]=height
      #heightfieldData[4*i+4*j*numHeightfieldRows:4*i+4+4*j*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+1)*numHeightfieldRows:4*i+4+(4*j+1)*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+2)*numHeightfieldRows:4*i+4+(4*j+2)*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+3)*numHeightfieldRows:4*i+4+(4*j+3)*numHeightfieldRows]=[height,height,height,height,]
  for j in range (int(numHeightfieldColumns)):
    for i in range (int(numHeightfieldRows) ):
      height = random.uniform(0,heightPerturbationRange)
      heightfieldData[1*i+1*j*numHeightfieldRows]=height
      #heightfieldData[4*i+1+4*j*numHeightfieldRows]=height
      #heightfieldData[4*i+(4*j+1)*numHeightfieldRows]=height
      #heightfieldData[4*i+1+(4*j+1)*numHeightfieldRows]=height
      #heightfieldData[4*i+4*j*numHeightfieldRows:4*i+4+4*j*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+1)*numHeightfieldRows:4*i+4+(4*j+1)*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+2)*numHeightfieldRows:4*i+4+(4*j+2)*numHeightfieldRows]=[height,height,height,height,]
      #heightfieldData[4*i+(4*j+3)*numHeightfieldRows:4*i+4+(4*j+3)*numHeightfieldRows]=[height,height,height,height,]
          
         
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])

if heightfieldSource==useDeepLocoCSV:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.2],fileName = "heightmaps/ground0.txt", heightfieldTextureScaling=256)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])

if heightfieldSource==useTerrainFromPNG:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.1],fileName = "heightmaps/Maze.png")
  textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
  terrain  = p.createMultiBody(0, terrainShape)
  p.changeVisualShape(terrain, -1, textureUniqueId = textureId)
 
 
p.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])

mass = 1
visualShapeId = -1


startPos = [0, 0, 0.3]


# 加载urdf文件
robot = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos)


mass = 1
visualShapeId = -1





p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)



while (p.isConnected()):
  keys = p.getKeyboardEvents()
  
  if updateHeightfield and heightfieldSource==useProgrammatic:
    for j in range (int(numHeightfieldColumns/2)):
      for i in range (int(numHeightfieldRows/2) ):
        height = random.uniform(0,heightPerturbationRange)#+math.sin(time.time())
        heightfieldData[2*i+2*j*numHeightfieldRows]=height
        heightfieldData[2*i+1+2*j*numHeightfieldRows]=height
        heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
        heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows]=height
    #GEOM_CONCAVE_INTERNAL_EDGE may help avoid getting stuck at an internal (shared) edge of the triangle/heightfield.
    #GEOM_CONCAVE_INTERNAL_EDGE is a bit slower to build though.
    #flags = p.GEOM_CONCAVE_INTERNAL_EDGE
    flags = 0
    #terrainShape2 = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, flags = flags, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns, replaceHeightfieldIndex = terrainShape)
    

  #print(keys)
  #getCameraImage note: software/TinyRenderer doesn't render/support heightfields!
  #p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL)
  time.sleep(0.01)