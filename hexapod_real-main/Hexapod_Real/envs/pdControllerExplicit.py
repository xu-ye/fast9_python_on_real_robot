import numpy as np


class PDControllerExplicitMultiDof(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep):

    numJoints = len(jointIndices)  #self._pb.getNumJoints(bodyUniqueId)
    curPos, curOrn = self._pb.getBasePositionAndOrientation(bodyUniqueId)
    q1 = [curPos[0], curPos[1], curPos[2], curOrn[0], curOrn[1], curOrn[2], curOrn[3]]
    baseLinVel, baseAngVel = self._pb.getBaseVelocity(bodyUniqueId)
    qdot1 = [
        baseLinVel[0], baseLinVel[1], baseLinVel[2], baseAngVel[0], baseAngVel[1], baseAngVel[2], 0
    ]
    qError = [0, 0, 0, 0, 0, 0, 0]
    qIndex = 7
    qdotIndex = 7
    zeroAccelerations = [0, 0, 0, 0, 0, 0, 0]
    for i in range(numJoints):
      js = self._pb.getJointStateMultiDof(bodyUniqueId, jointIndices[i])

      jointPos = js[0]
      jointVel = js[1]
      q1 += jointPos

      if len(js[0]) == 1:
        desiredPos = desiredPositions[qIndex-7]

        qdiff = desiredPos - jointPos[0]
        qError.append(qdiff)
        zeroAccelerations.append(0.)
        qdot1 += jointVel
        qIndex += 1
        qdotIndex += 1
      if len(js[0]) == 4:
        desiredPos = [
            desiredPositions[qIndex], desiredPositions[qIndex + 1], desiredPositions[qIndex + 2],
            desiredPositions[qIndex + 3]
        ]
        axis = self._pb.getAxisDifferenceQuaternion(desiredPos, jointPos)
        jointVelNew = [jointVel[0], jointVel[1], jointVel[2], 0]
        qdot1 += jointVelNew
        qError.append(axis[0])
        qError.append(axis[1])
        qError.append(axis[2])
        qError.append(0)
        desiredVel = [
            desiredVelocities[qdotIndex], desiredVelocities[qdotIndex + 1],
            desiredVelocities[qdotIndex + 2]
        ]
        zeroAccelerations += [0., 0., 0., 0.]
        qIndex += 4
        qdotIndex += 4

    q = np.array(q1)
    qdot = np.array(qdot1)
    qdotdesired = np.array(desiredVelocities)
    qdoterr = qdotdesired - qdot
    Kp = np.diagflat(kps)
    Kd = np.diagflat(kds)
    p = Kp.dot(qError)
    d = Kd.dot(qdoterr)
    forces = p + d
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces


class PDControllerExplicit(object):

  def __init__(self, pb):
    self._pb = pb
    self.vel_history=[]

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep,first_one=False):
    numJoints = self._pb.getNumJoints(bodyUniqueId)
    jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
    q1 = []
    qdot1 = []
    for i in range(numJoints):


      q1.append(jointStates[i][0])
      qdot1.append(jointStates[i][1])
    

    if first_one:
      self.vel_history=[]
      self.vel_history.append(qdot1[0])
    else:
      self.vel_history.append(qdot1[0])
    if len(self.vel_history)==1:
      qdot = np.array(qdot1)
    elif len(self.vel_history)==2:
      qdot = np.array([sum(self.vel_history)/2])
    else:
      qdot = np.array([sum(self.vel_history[-3:])/3])
    
      
    q = np.array(q1)
    qdot = np.array(qdot1)
    qdes = np.array(desiredPositions)
    qdotdes = np.array(desiredVelocities)
    qError = qdes - q
    qdotError = qdotdes - qdot
    flag=0
    flag=np.ones_like(qdotError)*-1
    flag[qdotError>0]=1
    #flag[qError>0]=1
    #if qdotError>0:
    #  flag=1
    #else:
    #  flag=-1
    kps=np.array(kps)
    kds=np.array(kds)
    kps[abs(qdotError)>5]=kps[abs(qdotError)>5]*0.6
    kds[abs(qdotError)>5]=kds[abs(qdotError)>5]*0.6

    Kp = np.diagflat([kps])
    Kd = np.diagflat([kds])
    #if abs(qdotError[0])>5:
    #  Kd=Kd*0.6
    #  Kp=Kp*0.6
    same_direction=np.zeros_like(qdotError)
    same_direction[qError*qdotError.transpose()<0]=1
    #if qError[0]*qdotError[0]<0:
    #  same_direction=1
    #else:
    #  same_direction=0
    forces = Kp.dot(qError) + Kd.dot(qdotError)
    forces=np.array(forces)
    limit_speed_indice=(abs(qdotError)>9 )*(same_direction>0)
    if limit_speed_indice.any():
      forces[limit_speed_indice]=np.clip((abs(qdotError[limit_speed_indice])-9.2)*flag[limit_speed_indice].transpose()*0.45,-3,3)
    #if abs(qdotError[0])>9 and same_direction:
    #       forces=np.clip([(abs(qdotError[0])-9.2)*flag*0.45],-3,3)
    #forces[same_direction<0.5]=abs(forces[same_direction<0.5])*flag[same_direction<0.5]*(-1)
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces
  


  def computePD0(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep,first_one=False):
    numJoints = self._pb.getNumJoints(bodyUniqueId)
    jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
    q1 = []
    qdot1 = []
    for i in range(numJoints):


      q1.append(jointStates[i][0])
      qdot1.append(jointStates[i][1])
    

    if first_one:
      self.vel_history=[]
      self.vel_history.append(qdot1[0])
    else:
      self.vel_history.append(qdot1[0])
    if len(self.vel_history)==1:
      qdot = np.array(qdot1)
    elif len(self.vel_history)==2:
      qdot = np.array([sum(self.vel_history)/2])
    else:
      qdot = np.array([sum(self.vel_history[-3:])/3])
    
      
    q = np.array(q1)
    qdot = np.array(qdot1)
    qdes = np.array(desiredPositions)
    qdotdes = np.array(desiredVelocities)
    qError = qdes - q
    qdotError = qdotdes - qdot
    flag=0
    
    if qdotError>0:
      flag=1
    else:
      flag=-1
    
    
    Kp = np.diagflat([kps])
    Kd = np.diagflat([kds])
    if abs(qdotError[0])>5:
      Kd=Kd*0.6
      Kp=Kp*0.6
    
    if qError[0]*qdotError[0]<0:
      same_direction=1
    else:
      same_direction=0
    forces = Kp.dot(qError) + Kd.dot(qdotError)
    forces=np.array(forces)
    
    if abs(qdotError[0])>9 and same_direction:
           forces=np.clip([(abs(qdotError[0])-9.2)*flag*0.45],-3,3)
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces
  


  def computePD_single(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep,first_one=False):
    numJoints = len(jointIndices)
    jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
    q1 = []
    qdot1 = []
    for i in range(numJoints):


      q1.append(jointStates[i][0])
      qdot1.append(jointStates[i][1])
    

    if first_one:
      self.vel_history=[]
      self.vel_history.append(qdot1[0])
    else:
      self.vel_history.append(qdot1[0])
    if len(self.vel_history)==1:
      qdot = np.array(qdot1)
    elif len(self.vel_history)==2:
      qdot = np.array([sum(self.vel_history)/2])
    else:
      qdot = np.array([sum(self.vel_history[-3:])/3])
    
      
    q = np.array(q1)
    qdot = np.array(qdot1)
    qdes = np.array(desiredPositions)
    qdotdes = np.array(desiredVelocities)
    qError = qdes - q
    qdotError = qdotdes - qdot
    flag=0
    
    if qdotError>0:
      flag=1
    else:
      flag=-1
    
    
    Kp = np.diagflat([kps])
    Kd = np.diagflat([kds])
    if abs(qdotError[0])>5:
      Kd=Kd*0.6
      Kp=Kp*0.6
    
    if qError[0]*qdotError[0]<0:
      same_direction=1
    else:
      same_direction=0
    forces = Kp.dot(qError) + Kd.dot(qdotError)
    forces=np.array(forces)
    
    if abs(qdotError[0])>9 and same_direction:
           forces=np.clip([(abs(qdotError[0])-9.2)*flag*0.45],-3,3)
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces