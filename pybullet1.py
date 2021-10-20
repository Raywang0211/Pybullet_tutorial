import pybullet as p
import pybullet_data
from time import sleep

# 链接物理引擎
p.connect(p.GUI)

# 设置模型加载路径
datapath = pybullet_data.getDataPath()
print('datapath = ',datapath)
p.setAdditionalSearchPath(datapath)

p.setGravity(0, 0, -10)

# 加载模型
# 加载URDF模型，此处是加载蓝白相间的陸地
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("husky/husky.urdf", startPos,startOrientation)
p.setRealTimeSimulation(1)
# 输出基本信息
joint_num = p.getNumJoints(robot_id)  #獲得對應物件的結點數量
print("r2d2的节点数量为：", joint_num)

print("r2d2的信息：")
for joint_index in range(joint_num):
    info_tuple = p.getJointInfo(robot_id, joint_index) #獲得該物件指定結點詳細資料
    print(f"關節序號：{info_tuple[0]}\n\
            關節名稱：{info_tuple[1]}\n\
            關節類型：{info_tuple[2]}\n\
            機器人第一個位置的變量索引：{info_tuple[3]}\n\
            機器人第一個速度的變量索引：{info_tuple[4]}\n\
            保留参数：{info_tuple[5]}\n\
            關節的阻尼大小：{info_tuple[6]}\n\
            關節的摩擦系数：{info_tuple[7]}\n\
            slider和revolute(hinge)类型的位移最小值：{info_tuple[8]}\n\
            slider和revolute(hinge)类型的位移最大值：{info_tuple[9]}\n\
            關節驅動的最大值：{info_tuple[10]}\n\
            關節的最大速度：{info_tuple[11]}\n\
            結點名稱：{info_tuple[12]}\n\
            局部框架中的關節軸系：{info_tuple[13]}\n\
            父節點frame的關節位置：{info_tuple[14]}\n\
            父節點frame的關節方向：{info_tuple[15]}\n\
            父節點的索引，若是基座返回-1：{info_tuple[16]}\n\n")


input()