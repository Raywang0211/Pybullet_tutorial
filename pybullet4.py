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
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf", useMaximalCoordinates=True)
startPos = [2, 0, 3]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("husky/husky.urdf",startPos )
startPos = [0, 0, 3]
robot_id_2 = p.loadURDF("r2d2.urdf", startPos)
print('robot_id ==================',robot_id)
print('robot_id_2 ==================',robot_id_2)

print('crash ============= ',)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
for i in range(100000):
    p.stepSimulation()
    P_min, P_max = p.getAABB(robot_id)
    id_tuple = p.getOverlappingObjects(P_min, P_max)
    if len(id_tuple)>1:
        time.sleep(1 / 240) 

input()
p.disconnect(serve_id)