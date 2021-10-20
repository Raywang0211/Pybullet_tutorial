import pybullet as p
import pybullet_data
import time

# 连接引擎
_ = p.connect(p.GUI)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)

# 载入地面模型，useMaximalCoordinates加大坐标刻度可以加快加载
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
startPos = [10, 10, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.loadURDF("soccerball.urdf", startPos,startOrientation)

# 创建过程中不渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# 禁用 tinyrenderer 
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.setGravity(0, 0, -9.8)

# 创建视觉模型和碰撞箱模型时共用的两个参数
shift = [0, -0.02, 0]
scale = [0.8, 0.8, 0.8]

# 创建视觉形状
# shape_type 可以調整成以下 ： GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER, GEOM_PLANE, GEOM_MESH
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="duck.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[10, 10, 0.1],
    visualFramePosition=shift,
    meshScale=scale
)

#創建碰撞箱形狀
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="duck_vhacd.obj",
    collisionFramePosition=shift,
    meshScale=scale
)

# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
for i in range(3):
    for j in range(3):
        p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[((-3/2)+i)*scale[0]*2, ((-3/2)+j)*scale[1]*2, 3],
            useMaximalCoordinates=False
        )

# 创建结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(1)


input()