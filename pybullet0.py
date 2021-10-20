import pybullet as p
import time
import pybullet_data

# 连接物理引擎
physicsCilent = p.connect(p.GUI) #分成有GUI模式跟沒有GUI模式，後者執行比較快  p.DIRECT
print('physicsCilent = ',p.getConnectionInfo(physicsCilent))

# pybullet.GUI {'isConnected': 1, 'connectionMethod': 1} or pybullet.DIRECT  {'isConnected': 1, 'connectionMethod': 2}  
# 'isConnected' = ID of client

# 添加將要使用的URDF檔案的路徑
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置环境重力加速度
p.setGravity(0, 0, -10)

# 加载URDF模型，此处是加载蓝白相间的陸地
planeId = p.loadURDF("plane.urdf")

# # 加载机器人，并设置加载的机器人的位姿
startPos = [0, 0, 5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# 按照位置和朝向重置机器人的位姿，由于我们之前已经初始化了机器人，所以此处加不加这句话没什么影响
# p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# 讓CPU不會參與影像渲染工作
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

# # # 开始一千次迭代，也就是一千次交互，每次交互后停顿1/240 以步為單位進行摹擬
# for i in range(10000):
#     p.stepSimulation() #進行正向動力學模擬
#     print('i = ',i)
#     time.sleep(1 / 500) #模擬的時間長短

#讓模擬器即時的摹擬運動不需要以每一步為單位顯示
p.setRealTimeSimulation(1)


# # 輸入物件ID來獲取位置與方向四元数
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print("-" * 20)
print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
print("-" * 20)

input()
# 断开连接
p.disconnect()