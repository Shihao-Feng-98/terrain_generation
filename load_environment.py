import pybullet as p
import pybullet_data as pd
import time
import random

def get_joints_id(robotId) -> list:
    joints_id = []
    print('All joints infomation:')
    for j in range(p.getNumJoints(robotId)):
        info = p.getJointInfo(robotId,j)
        print(info)
        joint_type = info[2]
        if (joint_type==p.JOINT_PRISMATIC or joint_type==p.JOINT_REVOLUTE):
            joints_id.append(j)
            # 取消电机闭环控制,执行力矩控制
            p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, force=0)
    print('Enable joint ids:', joints_id)
    return joints_id


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pd.getDataPath()) #optionally
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=135,
                                cameraPitch=-10, cameraTargetPosition=[0.5, 0, 0.3])
p.setGravity(0,0,-9.81)
p.setTimeStep(1./240.) # default time step 240Hz

planeId = p.loadURDF("plane.urdf")

heightPerturbationRange = 0.04 # 高度扰动范围
numHeightfieldRows = 64 # 高度场行数
numHeightfieldColumns = 64 # 高度场列数
heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
for j in range (int(numHeightfieldColumns/2)):
    for i in range (int(numHeightfieldRows/2)):
        height = random.uniform(0,heightPerturbationRange)
        heightfieldData[2*i+2*j*numHeightfieldRows]=height
        heightfieldData[(2*i+1)+2*j*numHeightfieldRows]=height
        heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
        heightfieldData[(2*i+1)+(2*j+1)*numHeightfieldRows]=height
terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, 
                                        meshScale=[.05,.05,1], 
                                        heightfieldTextureScaling=(numHeightfieldRows-1)/2, # 纹理
                                        heightfieldData=heightfieldData, 
                                        numHeightfieldRows=numHeightfieldRows, 
                                        numHeightfieldColumns=numHeightfieldColumns)
terrainId  = p.createMultiBody(0, terrainShape) # 重量0，碰撞实体Id
p.resetBasePositionAndOrientation(terrainId, [3,0,0], [0,0,0,1])
p.changeVisualShape(terrainId, -1, rgbaColor=[1,0.757,0.145,1]) # 255 193 37

stairId_1 = p.loadURDF("stairs_5cm_18deg.urdf",
                    [0,3,0.025], 
                    [0,0,0,1], 
                    useFixedBase=True, 
                    globalScaling=1)

# slopeId_1 = p.loadURDF("slope_25deg.urdf",
#                     [0,-3,0], 
#                     [0,0,0,1], 
#                     useFixedBase=True, 
#                     globalScaling=1)

# pendulumId = p.loadURDF("pendulum.urdf",
#                     [0,0,1], 
#                     [0,0,0,1], 
#                     useFixedBase=True, 
#                     globalScaling=1)
# p.resetJointState(pendulumId, 0, 0.5)
# p.setJointMotorControl2(pendulumId, 0, p.VELOCITY_CONTROL, force=0)

# robotId = p.loadURDF("/OcQ.urdf",
#                     [0,0,0.4], 
#                     p.getQuaternionFromEuler([0,0,0]), 
#                     useFixedBase=True, 
#                     # flags=p.URDF_USE_INERTIA_FROM_FILE,
#                     globalScaling=1)
# get_joints_id(robotId)

p.setRealTimeSimulation(0) # disable real-time simulation
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
