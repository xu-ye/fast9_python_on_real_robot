import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#door = p.loadURDF("door.urdf")
startPos = [0, 0, 0.2]
#door =p.loadURDF("actuators/actuators.urdf", startPos,useFixedBase=True)
door=p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos,useFixedBase=True)
#door = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos)
#linear/angular damping for base and all children=0
p.changeDynamics(door, -1, linearDamping=0, angularDamping=0)
for j in range(p.getNumJoints(door)):
  p.changeDynamics(door, j, linearDamping=0, angularDamping=0)
  print(p.getJointInfo(door, j))

frictionId = p.addUserDebugParameter("jointFriction", 0, 1, 0.1)
torqueId = p.addUserDebugParameter("joint torque", 0, 1, 0.05)
positinId = p.addUserDebugParameter("position", 0, 1, 0.05)
positinId1 = p.addUserDebugParameter("position", -2, 0, -0.33)
positinId2 = p.addUserDebugParameter("position", 0, 2, 0.3)

while (1):
  frictionForce = p.readUserDebugParameter(frictionId)
  jointTorque = p.readUserDebugParameter(torqueId)
  jointPosition = p.readUserDebugParameter(positinId)
  jointPosition1 = p.readUserDebugParameter(positinId1)
  jointPosition2 = p.readUserDebugParameter(positinId2)
  #set the joint friction
  #p.setJointMotorControl2(door, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
  #apply a joint torque
  #p.setJointMotorControl2(door, 0, p.TORQUE_CONTROL, force=jointTorque)
  p.setJointMotorControl2(door, 10, p.POSITION_CONTROL, targetPosition=jointPosition2)
  p.setJointMotorControl2(door, 11, p.POSITION_CONTROL, targetPosition=jointPosition1)
  p.stepSimulation()
  time.sleep(0.01)