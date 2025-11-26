from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from data_inputs import generate_networks_from_delivery_plan

random.seed(42)  # Change this number to get different random results

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
    D={i:random.randint(0,20) for i in B}       #随机需求
    D[0]=0                                      #仓库(0)需求为0


    #一阶段求解
    B_,C_,D_ = reform_data(B,C,D,level=1)
    CYCCLES,stage1_obj=CVRP(B_,C_,D_)
    sum_obj+=stage1_obj

    # 将cycle中的B_路径转换为原始的B路径，同时保留拆分后的需求信息
    ORIGINAL_CYCCLES = []
    SPLIT_DEMANDS = []  # 保存每条路线中各个节点的拆分需求
    for cycle in CYCCLES:
        original_cycle = []
        split_demand_map = {}  # {building_id: [demand1, demand2, ...]}
        for node in cycle:
            building_id = B_[node]
            original_cycle.append(building_id)
            demand = D_[node]
            if building_id not in split_demand_map:
                split_demand_map[building_id] = []
            split_demand_map[building_id].append(demand)
        ORIGINAL_CYCCLES.append(original_cycle+[0])
        SPLIT_DEMANDS.append(split_demand_map)

    # 结果输出,全向图路径
    print("原始需求:",D)
    print("拆分后需求:", D_)
    print("大楼配送方案:", ORIGINAL_CYCCLES,"共%d次"%(len(ORIGINAL_CYCCLES)))

    # 验证每条路线的总需求（使用拆分后的需求）
    print("\n验证每条路线的总需求:")
    for idx, (route, split_demand_map) in enumerate(zip(ORIGINAL_CYCCLES, SPLIT_DEMANDS)):
        buildings_in_route = [b for b in route if b != 0]
        total_demand = sum(sum(demands) for demands in split_demand_map.values())
        print(f"路线 #{idx+1}: {route}")
        print(f"  楼栋拆分需求: {split_demand_map}")
        print(f"  路线总需求: {total_demand}")
        if total_demand > 4:
            print(f"  ⚠️ 警告: 本路线总需求 {total_demand} 超过容量 Q=4! (此路线包含多个楼栋)")
        print()

    # 二阶段：为每个配送路线生成物理网络并求解
    print("\n=== 二阶段：楼内配送优化 ===")
    
    # 配置：所有楼层使用相同的房间布局
    num_floors = 30  # 每栋楼30层
    rooms_per_floor = 5  # 每层5个房间，单链结构；如需分支可改为 [3,2] 等
    
    # 为每栋楼生成房间需求分布：先随机生成，再按楼层排序，相近楼层分配到同一路线
    print("\n=== 基于一阶段路线生成房间需求（随机生成后按楼层分组到相近路线）===")
    building_room_demands = {}
    
    # 第一步：为每栋楼生成所有房间的随机需求分布
    building_total_demands = {}
    for route_idx, (route, split_demand_map) in enumerate(zip(ORIGINAL_CYCCLES, SPLIT_DEMANDS)):
        for building_id in route:
            if building_id == 0:
                continue
            if building_id not in split_demand_map:
                continue
            if building_id not in building_total_demands:
                building_total_demands[building_id] = 0
            for demand in split_demand_map[building_id]:
                building_total_demands[building_id] += demand
    
    # 为每栋楼随机生成房间需求
    building_all_room_demands = {}
    for building_id, total_demand in building_total_demands.items():
        if total_demand <= 0:
            continue
        
        print(f"\n为楼栋 {building_id} 随机生成总需求 {total_demand} 的房间分配")
        
        # 生成所有可能的房间列表
        all_rooms = []
        for floor in range(1, num_floors + 1):
            for room in range(1, rooms_per_floor + 1):
                room_id = f"{building_id}.{floor}.{room}"
                all_rooms.append(room_id)
        
        # 随机打乱房间顺序
        random.shuffle(all_rooms)
        
        # 随机分配需求到房间
        remaining = total_demand
        room_demands = {}
        
        for room_id in all_rooms:
            if remaining <= 0:
                break
            # 每个房间随机分配1-4个包裹，大部分房间只有1个包裹
            # 概率分布：1包裹(70%), 2包裹(20%), 3包裹(7%), 4包裹(3%)
            rand_val = random.random()
            if rand_val < 0.70:
                packages = 1
            elif rand_val < 0.90:
                packages = 2
            elif rand_val < 0.97:
                packages = 3
            else:
                packages = 4
            packages = min(packages, remaining)  # 不超过剩余需求
            room_demands[room_id] = packages
            remaining -= packages
        
        # 如果还有剩余需求（由于概率分配可能导致），分配到最后一个房间
        if remaining > 0 and room_demands:
            last_room = list(room_demands.keys())[-1]
            room_demands[last_room] += remaining
            remaining = 0
        
        # 按楼层排序房间
        sorted_room_demands = sorted(room_demands.items(), key=lambda x: tuple(map(int, x[0].split('.'))))
        building_all_room_demands[building_id] = sorted_room_demands
        total_allocated = sum(demand for _, demand in sorted_room_demands)
        print(f"  生成 {len(sorted_room_demands)} 个房间，按楼层排序完成（总需求: {total_allocated}）")
    
    # 第二步：根据路线将相近楼层的房间分配到同一路线
    print("\n=== 将相近楼层的房间分配到各路线 ===")
    building_room_index = {bid: 0 for bid in building_total_demands.keys()}  # 跟踪每栋楼的房间分配进度
    building_room_remaining = {}  # 跟踪每栋楼每个房间的剩余包裹数
    
    # 初始化剩余包裹数
    for building_id, sorted_demands in building_all_room_demands.items():
        building_room_remaining[building_id] = {room_id: packages for room_id, packages in sorted_demands}
    
    # 创建per-route的房间需求结构
    route_building_room_demands = []  # List of dicts: [{building_id: {room: demand}, ...}, ...]
    
    for route_idx, (route, split_demand_map) in enumerate(zip(ORIGINAL_CYCCLES, SPLIT_DEMANDS)):
        print(f"\n为路线 #{route_idx+1}: {route} 分配房间需求")
        print(f"  楼栋拆分需求: {split_demand_map}")
        
        route_room_demands = {}  # 这条路线的房间需求: {building_id: {room: demand}}
        
        if route_idx not in [item for item in range(len(ORIGINAL_CYCCLES))]:
            route_building_room_demands.append(route_room_demands)
            continue
        
        # 遍历路线中的每个楼栋
        for building_id in route:
            if building_id == 0:  # 跳过仓库
                continue
            
            # 获取该楼栋在本次访问中的需求
            if building_id not in split_demand_map:
                continue
            
            # 如果楼栋还没有初始化房间需求，先初始化
            if building_id not in building_room_demands:
                building_room_demands[building_id] = {}
            
            if building_id not in route_room_demands:
                route_room_demands[building_id] = {}
            
            # split_demand_map[building_id] 是一个列表，包含该楼栋的所有拆分需求
            demands_for_building = split_demand_map[building_id]
            
            # 为该楼栋的每次访问分配需求（从已排序的房间列表中按顺序取）
            for visit_demand in demands_for_building:
                if visit_demand <= 0:
                    continue
                
                # 从已排序的房间列表中取出相近楼层的房间
                # 重要：每个房间的包裹必须在同一次访问中全部配送完，不能拆分到多次访问
                allocated = 0
                rooms_in_this_visit = []
                
                # 第一遍：从当前索引开始，尽量分配连续的房间
                i = building_room_index[building_id]
                while i < len(building_all_room_demands[building_id]) and allocated < visit_demand:
                    room_id, original_packages = building_all_room_demands[building_id][i]
                    remaining_in_room = building_room_remaining[building_id].get(room_id, 0)
                    
                    if remaining_in_room <= 0:
                        i += 1
                        continue
                    
                    # 只有当这个房间的所有包裹都能放入本次访问时，才分配这个房间
                    if remaining_in_room <= (visit_demand - allocated):
                        # 可以完整分配这个房间的所有包裹
                        can_take = remaining_in_room
                        
                        # 更新全局building_room_demands（用于显示）
                        if room_id not in building_room_demands[building_id]:
                            building_room_demands[building_id][room_id] = can_take
                        else:
                            building_room_demands[building_id][room_id] += can_take
                        
                        # 更新本路线的房间需求
                        if room_id not in route_room_demands[building_id]:
                            route_room_demands[building_id][room_id] = can_take
                            rooms_in_this_visit.append(room_id)
                        else:
                            route_room_demands[building_id][room_id] += can_take
                        
                        allocated += can_take
                        building_room_remaining[building_id][room_id] = 0
                        
                        # 更新索引：移动到下一个未分配的房间
                        building_room_index[building_id] = i + 1
                    
                    i += 1
                
                # 第二遍：如果还没凑够，回到开头寻找之前跳过的房间
                if allocated < visit_demand:
                    for i in range(len(building_all_room_demands[building_id])):
                        if allocated >= visit_demand:
                            break
                        
                        room_id, original_packages = building_all_room_demands[building_id][i]
                        remaining_in_room = building_room_remaining[building_id].get(room_id, 0)
                        
                        if remaining_in_room <= 0:
                            continue
                        
                        # 只分配能完整放入的房间
                        if remaining_in_room <= (visit_demand - allocated):
                            can_take = remaining_in_room
                            
                            # 更新全局building_room_demands（用于显示）
                            if room_id not in building_room_demands[building_id]:
                                building_room_demands[building_id][room_id] = can_take
                            else:
                                building_room_demands[building_id][room_id] += can_take
                            
                            # 更新本路线的房间需求
                            if room_id not in route_room_demands[building_id]:
                                route_room_demands[building_id][room_id] = can_take
                                rooms_in_this_visit.append(room_id)
                            else:
                                route_room_demands[building_id][room_id] += can_take
                            
                            allocated += can_take
                            building_room_remaining[building_id][room_id] = 0
                
                # 第三遍：如果仍然没凑够（极端情况），允许部分分配房间
                if allocated < visit_demand:
                    for i in range(len(building_all_room_demands[building_id])):
                        if allocated >= visit_demand:
                            break
                        
                        room_id, original_packages = building_all_room_demands[building_id][i]
                        remaining_in_room = building_room_remaining[building_id].get(room_id, 0)
                        
                        if remaining_in_room <= 0:
                            continue
                        
                        # 允许部分分配：取尽可能多的包裹
                        can_take = min(remaining_in_room, visit_demand - allocated)
                        
                        # 更新全局building_room_demands（用于显示）
                        if room_id not in building_room_demands[building_id]:
                            building_room_demands[building_id][room_id] = can_take
                        else:
                            building_room_demands[building_id][room_id] += can_take
                        
                        # 更新本路线的房间需求
                        if room_id not in route_room_demands[building_id]:
                            route_room_demands[building_id][room_id] = can_take
                            rooms_in_this_visit.append(room_id)
                        else:
                            route_room_demands[building_id][room_id] += can_take
                        
                        allocated += can_take
                        building_room_remaining[building_id][room_id] -= can_take
                
                # 提取楼层范围用于显示
                if rooms_in_this_visit:
                    floors = [int(r.split('.')[1]) for r in rooms_in_this_visit]
                    floor_range = f"{min(floors)}-{max(floors)}" if len(set(floors)) > 1 else f"{floors[0]}"
                    print(f"  楼栋 {building_id} 本次访问需求 {visit_demand}，分配到楼层 {floor_range} 的 {len(rooms_in_this_visit)} 个房间")
        
        route_building_room_demands.append(route_room_demands)
    
    print("\n最终房间需求分配（按楼层排序）:")
    for building_id, room_demands in building_room_demands.items():
        print(f"  楼栋 {building_id}: {len(room_demands)} 个房间，总需求 {sum(room_demands.values())}")
        # 按楼层排序显示前几个房间
        sorted_rooms = sorted(room_demands.items(), key=lambda x: tuple(map(int, x[0].split('.'))))
        print(f"    示例房间: {sorted_rooms[:5]}")
    
    # 为每条配送路线生成物理网络
    print("\n=== 生成物理网络（使用预分配的房间需求）===")
    
    # 为每条路线单独生成网络，使用对应的房间需求
    stage2_total = 0
    network_count = 0
    
    for route_idx, (route, route_room_demands_dict) in enumerate(zip(ORIGINAL_CYCCLES, route_building_room_demands)):
        # 为这条路线生成网络
        networks = generate_networks_from_delivery_plan(
            delivery_plan=[route],  # 只处理当前这一条路线
            building_demands=D,  # 原始楼栋需求
            rooms_per_floor=rooms_per_floor,
            Q=4,                    # 机器人载重量
            k=5,                    # 楼层间转移成本系数
            intra_floor_weight=1,   # 楼层内边权重
            close_loop=True,        # 闭环：最后房间连回入口
            separate_buildings=True, # 每栋楼单独生成网络和优化
            building_room_demands=route_room_demands_dict  # 使用这条路线特定的房间需求
        )
        
        # 对这条路线的每栋楼进行二阶段优化
        for net_route_idx, net_route, building_id, a, b, c, d in networks:
            actual_route_idx = route_idx + 1  # 因为索引从0开始，显示时+1
            print(f"\n--- 配送 #{actual_route_idx}: 路线 {route}, 楼栋 {building_id} ---")
            print(f"房间需求: {d}")
            
            if not d:
                print("无房间需求，跳过")
                continue
            
            # 根据building_id修改特定节点权重
            # 节点格式：'floor.room'（如 '5.1' 表示5层1号房间）
            if building_id == 1:
                nodes_to_modify = ['1.1', '2.1', '3.1','4.1', '15.2', '20.1','5.2', '10.2', '25.1']
            elif building_id == 3:
                nodes_to_modify = ['15.2', '25.1']
            elif building_id == 5:
                nodes_to_modify = ['10.3']
            else:
                nodes_to_modify = []  # 其他楼栋不修改
            
            # 应用节点权重修改
            for node in nodes_to_modify:
                for (u, v) in list(c.keys()):
                    if u == node or v == node:
                        c[(u, v)] = 10
            
            # 二阶段求解
            b_, c_, d_ = reform_data(b, c, d, level=2)
            cycles, obj = CVRP(b_, c_, d_)
            stage2_total += obj
            network_count += 1
            
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
    print(f"\n=== 总配送距离: {sum_obj} (一阶段: {stage1_obj}, 二阶段: {stage2_total}) ===")