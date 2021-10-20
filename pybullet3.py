import pybullet as p
import pybullet_data
import time 
from pprint import pprint
import random

# 连接物理引擎
use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 设置重力，加载模型
p.setGravity(0, 0, -50)
p.loadURDF("plane.urdf", useMaximalCoordinates=True)
startPos = [2, 0, 3]
robot_id = p.loadURDF("r2d2.urdf",startPos, useMaximalCoordinates=True)
startPos = [0, 0, 3]
robot_id_2 = p.loadURDF("r2d2.urdf", startPos, useMaximalCoordinates=True)
print('robot_id ==================',robot_id)
print('robot_id_2 ==================',robot_id_2)

# 找到可動關節,不可動的關節是用 4 表示 p.JOINT_FIXED
available_joints_indexes = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != 4] #列出特定物件可動關節 index
pprint([p.getJointInfo(robot_id, i)[1] for i in available_joints_indexes])  #列出可動關節的名稱
wheel_joints_indexes = [i for i in available_joints_indexes if "wheel" in str(p.getJointInfo(robot_id, i)[1])]
print(available_joints_indexes)
print(wheel_joints_indexes)

available_joints_indexes_2 = [i for i in range(p.getNumJoints(robot_id_2)) if p.getJointInfo(robot_id_2, i)[2] != 4] #列出特定物件可動關節 index
pprint([p.getJointInfo(robot_id, i)[1] for i in available_joints_indexes_2])  #列出可動關節的名稱
wheel_joints_indexes_2 = [i for i in available_joints_indexes_2 if "wheel" in str(p.getJointInfo(robot_id_2, i)[1])]
print(available_joints_indexes_2)
print(wheel_joints_indexes_2)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

target_v = 100*random.random()          # 电机达到的预定角速度（rad/s）
max_force = 10*random.random()          # 电机能够提供的力，这个值决定了机器人运动时的加速度，太快会翻车哟，单位N
target_v_2 = 100*random.random()
max_force_2 = 10*random.random()  
# 透過p.setJointMotorControlArray 執行可動關節移動
for i in range(1000):
    p.stepSimulation()

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id_2,
        jointIndices=wheel_joints_indexes_2,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[target_v_2 for _ in wheel_joints_indexes_2],
        forces=[max_force_2 for _ in wheel_joints_indexes_2]
    )
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=wheel_joints_indexes,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[target_v for _ in wheel_joints_indexes],
        forces=[max_force for _ in wheel_joints_indexes]
    )

    location, _ = p.getBasePositionAndOrientation(robot_id_2)
    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=110,
        cameraPitch=-30,
        cameraTargetPosition=location
    )
    time.sleep(1 / 240)         # 模拟器一秒模拟迭代240步

# 透過p.setJointMotorControl2 執行可動關節移動 <兩個其實差不多只是一次執行一個關節還是多個>
# for i in range(10000):
#     p.stepSimulation()
#     p.setJointMotorControl2(
#         robot_id,
#         2,
#         p.VELOCITY_CONTROL,
#         target_v,
#         max_force
#     )
#     p.setJointMotorControl2(
#         robot_id,
#         3,
#         p.VELOCITY_CONTROL,
#         target_v,
#         max_force
#     )
#     p.setJointMotorControl2(
#         robot_id,
#         6,
#         p.VELOCITY_CONTROL,
#         target_v,
#         max_force
#     )
#     p.setJointMotorControl2(
#         robot_id,
#         7,
#         p.VELOCITY_CONTROL,
#         target_v,
#         max_force
#     )
#     location, _ = p.getBasePositionAndOrientation(robot_id)
#     p.resetDebugVisualizerCamera(
#         cameraDistance=3,
#         cameraYaw=110,
#         cameraPitch=-30,
#         cameraTargetPosition=location
#     )
#     time.sleep(1 / 240) 
# input()


# 断开连接

p.disconnect(serve_id)