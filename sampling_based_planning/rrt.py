import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x # 节点的x坐标
        self.y = y # 节点的y坐标
        self.parent = None # 节点的父节点

class RRT:
    def __init__(self, start, goal, obstacle_list, search_range=50, step_size=5, max_iterations=5000):
        self.start = Node(start[0], start[1]) # 起点
        self.goal = Node(goal[0], goal[1]) # 终点
        self.obstacle_list = obstacle_list # 障碍物列表，每个障碍物由其坐标、宽度和高度表示
        self.search_range = search_range # 搜索半径
        self.step_size = step_size # 步长
        self.max_iterations = max_iterations # 最大迭代次数
        self.node_list = [self.start] # 节点列表，包含所有已生成的节点
        self.width = max([o[0]+o[2] for o in obstacle_list]) # 地图宽度
        self.height = max([o[1]+o[3] for o in obstacle_list]) # 地图高度

    def planning(self):
        for i in range(self.max_iterations):
            rand_node = self.get_random_node() # 获取随机节点
            nearest_node = self.get_nearest_node(rand_node) # 获取距离随机节点最近的已生成节点
            new_node = self.steer(nearest_node, rand_node) # 沿着连接最近节点和随机节点的直线，前进一定距离，得到新节点
            if self.check_collision(new_node): # 检查新节点是否与障碍物碰撞
                self.node_list.append(new_node) # 如果未碰撞，则将新节点添加到节点列表中
                if self.is_goal(new_node): # 检查新节点是否为目标节点
                    return self.generate_path(new_node) # 如果是目标节点，则返回从起点到目标节点的路径
        return None # 如果达到最大迭代次数仍未找到路径，则返回空

    def get_random_node(self):
        if random.randint(0, 100) > 5: # 95%的概率，随机选择地图范围内的一个点
            x = random.uniform(0, self.width)
            y = random.uniform(0, self.height)
            return Node(x, y)
        else: # 5%的概率，随机选择目标点
            return self.goal

    def get_nearest_node(self, node):
        min_dist = float('inf') # 初始距离为无穷大
        nearest_node = None
        for n in self.node_list: # 遍历所有已生成节点，找到距离随机节点最近的节点
            dist = self.calculate_distance(n, node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = n
        return nearest_node

    def steer(self, from_node, to_node):
        dist = self.calculate_distance(from_node, to_node)  # 计算连接最近节点和随机节点的直线长度
        if dist <= self.step_size:  # 如果直线长度小于等于步长，则直接到达随机节点
            return to_node
        else:  # 否则，沿着直线前进一步，得到新节点
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            x = from_node.x + self.step_size * math.cos(theta)
            y = from_node.y + self.step_size * math.sin(theta)
            return Node(x, y)

    def check_collision(self, node):
        for o in self.obstacle_list:  # 遍历所有障碍物
            if node.x >= o[0] and node.x <= o[0] + o[2] and node.y >= o[1] and node.y <= o[1] + o[
                3]:  # 如果新节点与当前障碍物相交，则表示发生碰撞
                return False
        return True  # 如果新节点不与任何障碍物相交，则表示未发生碰撞

    def is_goal(self, node):
        dist = self.calculate_distance(node, self.goal)  # 计算新节点与目标节点的距离
        if dist < self.step_size:  # 如果距离小于等于步长，则认为已到达目标节点
            return True
        else:
            return False

    def calculate_distance(self, from_node, to_node):
        return math.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)

    def generate_path(self, node):
        path = []
        while node.parent is not None:  # 从目标节点开始，一直追溯到起点，得到路径
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        path.reverse()  # 将路径反转，从起点到目标节点
        return path
"""

这个实现基于一个Node类和一个RRT类。其中，Node类表示一个节点，包含x、y坐标和父节点；RRT类表示RRT算法，
包含起点、终点、障碍物列表、搜索半径、步长和最大迭代次数等参数，以及一系列函数，包括规划函数planning()、
获取随机节点的函数get_random_node()、获取距离随机节点最近的节点的函数get_nearest_node()、
沿着连接最近节点和随机节点的直线前进一定距离得到新节点的函数steer()
、检查新节点是否与障碍物碰撞的函数check_collision()、
检查新节点是否为目标节点的函数is_goal()、
计算两个节点之间距离的函数calculate_distance()
和从目标节点追溯到起点得到路径的函数generate_path()。

"""