from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from data_inputs import generate_networks_from_delivery_plan

random.seed(0)

def reform_data(B,C,D,level):

    #input重构，拆分需求大于4的节点
    if level == 1:
        B_ = {0: 0}  # depot保持原样
    elif level == 2:
        B_ = {0: '1.0'}  # depot保持原样
    else:
        print("请输入正确level参数,用于数据转换")
    D_ = {0: 0}  # depot需求为0
    idx = 1
    for i in D:
        remaining_demand = D[i]
        while remaining_demand > 0:
            # 每个新节点的需求最多为4,超过4则在原位置复制一个节点
            current_demand = min(remaining_demand, 4)
            B_[idx] = i
            D_[idx] = current_demand
            remaining_demand -= current_demand
            idx += 1

    # 计算距离矩阵
    if level == 1:
        C_ = {}
        for i in B_:
            for j in B_:
                if i != j:
                    C_[(i,j)] = C[(B_[i], B_[j])]
    # 将物理网络转换为距离矩阵
    elif level == 2:
        # 创建物理网络图
        G = nx.Graph()
        G.add_nodes_from(b)
        G.add_edges_from(a)
        # 为物理网络图添加边的权重
        for (i, j) in a:
            G[i][j]['weight'] = c[(i, j)]
        # 获得全向图距离矩阵
        C_={}
        for i in B_:
            for j in B_:
                if i != j:
                    distance = nx.shortest_path_length(G, B_[i],B_[j], weight='weight')
                    C_[(i,j)] = distance


    return B_,C_,D_

#CVRP模型,一阶段处理楼栋配送,二阶段处理楼内配送
def CVRP(B,C,D,Q=4):
    V = [i for i in range(math.ceil(sum(D.values())/Q))]
    #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in C if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.DiGraph()
            G.add_edges_from(edges)

            cycles = list(networkx.simple_cycles(G))
            # print("Cycles (lazy):", cycles)
            for S in cycles:
                S = [i for i in S if i != 0]
                if len(S) > 1: #理论上可以不加这句,但是会导致在出现类似[0,1]的回路gurobi的数值问题
                    #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入,消除子回路和超载回路
                    model.cbLazy(quicksum(x[i,j] for i in B if i not in S for j in S if i!=j) >= math.ceil(sum(D[i] for i in S) / Q))
    MD = Model()

    x  = MD.addVars([(i,j) for (i,j) in C], vtype=GRB.BINARY)

    #约束1,每个客户节点必须有且仅有一条出边（除仓库外）
    MD.addConstrs(quicksum(x[i,j] for j in B if j!=i)==1 for i in B if i!=0)
    #约束2,每个客户节点必须有且仅有一条入边（除仓库外）
    MD.addConstrs(quicksum(x[i,j] for i in B if j!=i)==1 for j in B if j!=0)
    #约束3,从仓库出发的车辆数等于返回仓库的车辆数（流平衡约束）
    MD.addConstr(quicksum(x[0,j] for j in B if j!=0)==quicksum(x[i,0] for i in B if i!=0))
    # #约束4,从仓库出发的车辆数为所需最少车辆数
    MD.addConstr(quicksum(x[0, j] for j in B if j != 0) == len(V))

    #目标函数
    MD.setObjective(quicksum(C[i,j]*x[i,j] for (i,j) in C), GRB.MINIMIZE)
    MD.Params.lazyConstraints = 1
    MD.Params.OutputFlag=0 #不输出求解过程
    MD.optimize(Cut)

    #构造回路
    edges = [(i,j) for (i,j) in C if x[i,j].X > 0.5]
    G = networkx.DiGraph()
    G.add_edges_from(edges)
    cycles = list(networkx.simple_cycles(G))
    # print("Cycles:", cycles)
    # print(MD1.ObjVal)
    # # 画出网络图
    # pos = nx.spring_layout(G)
    # plt.figure(figsize=(12, 8))
    # nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=500)
    # nx.draw_networkx_edges(G, pos, edge_color='black', arrows=True, arrowsize=20)
    # nx.draw_networkx_labels(G, pos)

    # # 添加边的权重标签
    # edge_labels = {(i,j): C[i,j] for (i,j) in edges}
    # nx.draw_networkx_edge_labels(G, pos, edge_labels)

    # plt.title("CVRP Solution Network")
    # plt.axis('off')
    # plt.show()

    return cycles,MD.ObjVal


if __name__ == "__main__":
    sum_obj=0

    # input,一阶段都用大写字母,二阶段都用小写字母,主要参数B,C,D,a,b,c,d
    # 一阶段数据
    B=[i for i in range(10)]                    #仓库(0)和楼栋编号
    C={}                                        #距离
    for i in B:
        for j in B:
            if i != j:
                distance = random.randint(20, 50)
                C[(i,j)] = distance
                C[(j,i)] = distance
            else:
                C[i,j]=0
    D={i:random.randint(0,10) for i in B}       #随机需求
    D[0]=0                                      #仓库(0)需求为0


    #一阶段求解
    B_,C_,D_ = reform_data(B,C,D,level=1)
    CYCCLES,obj=CVRP(B_,C_,D_)
    sum_obj+=obj

    # 将cycle中的B_路径转换为原始的B路径
    ORIGINAL_CYCCLES = []
    for i in CYCCLES:
        original_cycle = []
        for node in i:
            original_cycle.append(B_[node])  # 转换为原始楼栋编号
        ORIGINAL_CYCCLES.append(original_cycle+[0])

    # 结果输出,全向图路径
    print("需求:",D)
    print("大楼配送方案:", ORIGINAL_CYCCLES,"共%d次"%(len(ORIGINAL_CYCCLES)))


    # 二阶段：为每个配送路线生成物理网络并求解
    print("\n=== 二阶段：楼内配送优化 ===")
    
    # 配置：所有楼层使用相同的房间布局
    num_floors = 30  # 每栋楼30层
    rooms_per_floor = 5  # 每层5个房间，单链结构；如需分支可改为 [3,2] 等
    
    # 为每栋楼生成随机房间需求分布（将建筑需求随机分配到房间）
    def generate_room_demands_for_building(building_id, total_demand, num_floors, rooms_per_floor):
        """随机将建筑需求分配到各房间"""
        if total_demand <= 0:
            return {}
        
        # 计算总房间数
        if isinstance(rooms_per_floor, int):
            total_rooms = num_floors * rooms_per_floor
        else:
            # 如果是列表，计算每层房间总数
            total_rooms = num_floors * sum(rooms_per_floor)
        
        # 生成所有可能的房间列表 (floor.room格式)
        all_rooms = []
        for floor in range(1, num_floors + 1):
            if isinstance(rooms_per_floor, int):
                num_rooms = rooms_per_floor
            else:
                num_rooms = sum(rooms_per_floor)
            for room in range(1, num_rooms + 1):
                all_rooms.append(f"{building_id}.{floor}.{room}")
        
        # 随机选择房间并分配需求（每个包裹最多4个单位）
        room_demands = {}
        remaining = total_demand
        
        # 随机打乱房间顺序
        import random
        random.shuffle(all_rooms)
        
        # 分配需求到随机房间
        for room in all_rooms:
            if remaining <= 0:
                break
            # 每个房间随机分配1-4个包裹（不超过剩余需求）
            packages = min(random.randint(1, 4), remaining)
            room_demands[room] = packages
            remaining -= packages
        
        return room_demands
    
    # 生成每栋楼的房间需求
    building_room_demands = {}
    for building_id, demand in D.items():
        if building_id != 0 and demand > 0:
            building_room_demands[building_id] = generate_room_demands_for_building(
                building_id, demand, num_floors, rooms_per_floor
            )
            print(f"楼栋 {building_id} 房间需求分布: {building_room_demands[building_id]}")
    
    # 为每条配送路线生成物理网络（每栋楼单独优化）
    networks = generate_networks_from_delivery_plan(
        delivery_plan=ORIGINAL_CYCCLES,
        building_demands=D,
        rooms_per_floor=rooms_per_floor,
        Q=4,                    # 机器人载重量
        k=5,                    # 楼层间转移成本系数
        intra_floor_weight=1,   # 楼层内边权重
        close_loop=True,        # 闭环：最后房间连回入口
        separate_buildings=True, # 每栋楼单独生成网络和优化
        building_room_demands=building_room_demands  # 传入详细房间需求
    )
    
    # 对每栋楼进行二阶段优化
    stage2_total = 0
    for route_idx, route, building_id, a, b, c, d in networks:
        print(f"\n--- 配送 #{route_idx+1}: 路线 {route}, 楼栋 {building_id} ---")
        print(f"房间需求: {d}")
        
        if not d:
            print("无房间需求，跳过")
            continue
        
        # 可选：修改特定节点的边权重（示例：增加某些节点的穿行成本）
        # nodes_to_modify = ['3.2', '3.3']
        # for node in nodes_to_modify:
        #     for (u, v) in list(c.keys()):
        #         if u == node or v == node:
        #             c[(u, v)] = 10
        
        # 二阶段求解
        b_, c_, d_ = reform_data(b, c, d, level=2)
        cycles, obj = CVRP(b_, c_, d_)
        stage2_total += obj
        
        # 结果输出：全向图路径
        original_cycles = []
        for i in cycles:
            original_cycle = []
            for node in i:
                original_cycle.append(b_[node])
            original_cycles.append(original_cycle + ['1.0'])
        
        print(f"楼内全向图配送方案: {original_cycles}")
        
        # 结果输出：将全向图路径转换为物理网络路径
        physical_path = []
        G = nx.Graph()
        G.add_nodes_from(b)
        G.add_edges_from(a)
        for (i, j) in a:
            G[i][j]['weight'] = c[(i, j)]
        
        for cycle in original_cycles:
            detailed_path = []
            for i in range(len(cycle) - 1):
                start = cycle[i]
                end = cycle[i + 1]
                segment_path = nx.shortest_path(G, start, end, weight='weight')
                if i == 0:
                    detailed_path.extend(segment_path)
                else:
                    detailed_path.extend(segment_path[1:])
            physical_path.append(detailed_path)
        
        print(f"楼内物理网络配送方案: {physical_path}")
        print(f"本次配送楼内距离: {obj}")
    
    sum_obj += stage2_total
    print(f"\n=== 总配送距离: {sum_obj} (一阶段: {obj}, 二阶段: {stage2_total}) ===")