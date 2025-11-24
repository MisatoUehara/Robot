import networkx as nx
from CVRP_2index_multi_demand import *
# 二阶段数据
b=['1.0','1.1','1.2','1.3','2.0','2.1','2.2','2.3','2.4','2.5','3.0','3.1'] #1.0,2.0,3.0为楼层入口,其余为房间号
a=[('1.0','1.1'),('1.1','1.2'),('1.2','1.3'),('1.0','2.0'),('2.0','2.1'),('2.1','2.2'),('2.2','2.3'),('2.0','2.4'),('2.4','2.5'),('2.0','3.0'),('3.0','3.1')]#弧
c={(i,j):1 for (i,j) in a}                  #路网距离,先设定单位距离

# # 物理网络变成仅考虑需求点的全向图
# d={'1.1':1,'1.2':2,'2.1':6,'2.2':1,'3.1':1}  #各房间需求

# d={'1.1':1,'2.1':8}  #各房间需求
d={'1.1':1}  #各房间需求



b_ = {0: '1.0'}  # depot保持原样
d_ = {0: 0}  # depot需求为0
idx = 1
for i in d:
    remaining_demand = d[i]
    while remaining_demand > 0:
        # 每个新节点的需求最多为4,超过4则在原位置复制一个节点
        current_demand = min(remaining_demand, 4)
        b_[idx] = i
        d_[idx] = current_demand
        remaining_demand -= current_demand
        idx += 1



# 创建物理网络图
G = nx.Graph()
G.add_nodes_from(b)
G.add_edges_from(a)
# 为图添加边的权重
for (i, j) in a:
    G[i][j]['weight'] = c[(i, j)]
# 获得全向图字典
c_={}
for i in b_:
    for j in b_:
        if i != j:
            distance = nx.shortest_path_length(G, b_[i],b_[j], weight='weight')
            c_[(i,j)] = distance
print(b_)
print(c_)
print(d_)

cycles=CVRP(b_,c_,d_)

#解的解析
# 将cycle中的B_路径转换为原始的B路径
original_cycles = []
for i in cycles:
    original_cycle = []
    for node in i:
        original_cycle.append(b_[node])  # 转换为原始楼栋编号
    original_cycles.append(original_cycle+['1.0'])

print("楼内全向图配送方案:", original_cycles)

# 将全向图路径转换为物理网络路径
physical_path = []
for cycle in original_cycles:
    detailed_path = []
    for i in range(len(cycle) - 1):
        start = cycle[i]
        end = cycle[i + 1]
        segment_path = nx.shortest_path(G, start, end, weight='weight')
        # 避免重复添加节点（除了第一个段的起点）
        if i == 0:
            detailed_path.extend(segment_path)
        else:
            detailed_path.extend(segment_path[1:])  # 跳过起点避免重复
    physical_path.append(detailed_path)

print("楼内物理网络配送方案:", physical_path)
