from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# ============================================================================
# 配置参数 / Configuration Parameters
# ============================================================================
RANDOM_SEED = 200              # 随机种子 / Random seed for reproducibility
Q = 4                       # 机器人载重量 / Robot capacity (packages)

# 一阶段参数 / Stage 1 Parameters (Building-level delivery)
NUM_BUILDINGS = 5              # 楼栋数量（不包括仓库）/ Number of buildings (excluding depot)
MIN_BUILDING_DISTANCE = 20      # 楼栋间最小距离 / Min distance between buildings
MAX_BUILDING_DISTANCE = 50      # 楼栋间最大距离 / Max distance between buildings
MIN_BUILDING_DEMAND = 0         # 楼栋最小需求 / Min demand per building
MAX_BUILDING_DEMAND = 20        # 楼栋最大需求 / Max demand per building

# 二阶段参数 / Stage 2 Parameters (Room-level delivery)
NUM_FLOORS = 5                  # 每栋楼层数 / Number of floors per building
ROOMS_PER_FLOOR = 5             # 每层房间数 / Number of rooms per floor
FLOOR_DISTANCE = 5              # 楼层间距离（垂直移动成本）/ Distance between floors (vertical cost)
ROOM_DISTANCE = 1               # 同层房间间距离（水平移动成本）/ Distance between rooms on same floor (horizontal cost)
CON_COST = 20                  # 楼层连接的固定连接代价 / Inter-floor connector cost added to transfer edges

# 房间需求分配概率 / Room demand allocation probability
PROB_1_PACKAGE = 0.70           # 1个包裹的概率 / Probability of 1 package
PROB_2_PACKAGES = 0.20          # 2个包裹的概率 / Probability of 2 packages (cumulative: 0.90)
PROB_3_PACKAGES = 0.07          # 3个包裹的概率 / Probability of 3 packages (cumulative: 0.97)
# 其余概率为4个包裹 / Remaining probability is for 4 packages

# 可视化参数 / Visualization Parameters
ENABLE_VISUALIZATION = True  # 启用可视化 / Enable visualization
SHOW_STAGE1_VIS = True          # 显示一阶段可视化 / Show stage 1 visualization
SHOW_STAGE2_VIS = True          # 显示二阶段可视化 / Show stage 2 visualization
MAX_STAGE2_BUILDINGS_VIS = 1  # 最多显示几个楼栋的二阶段路径 / Max buildings to visualize in stage 2
# ============================================================================

random.seed(RANDOM_SEED)  # Change this number to get different random results

def reform_data(B,C,D,level,Q=Q):

    #input重构，拆分需求大于Q的节点
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
            # 每个新节点的需求最多为Q,超过Q则在原位置复制一个节点
            current_demand = min(remaining_demand, Q)
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
def CVRP(B,C,D,level,Q=Q):
    V = [i for i in range(math.ceil(sum(D.values())/Q))]
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in C if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.DiGraph()
            G.add_edges_from(edges)

            cycles = list(networkx.simple_cycles(G))
            #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入
            for S in cycles:
                S = [i for i in S if i != 0]
                if len(S) > 1: #理论上可以不加这句,但是会导致在出现类似[0,1]的回路gurobi的数值问题
                    #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入,消除子回路和超载回路
                    model.cbLazy(quicksum(x[i,j] for i in B if i not in S for j in S if i!=j) >= math.ceil(sum(D[i] for i in S) / Q))
            #约束6,非满载回路合并约束,仅对level2模型启用
            if level == 2:
                non_full_cycle=[]
                for S in cycles:
                    load = sum(D[node] for node in S if node != 0)
                    if load < Q:
                        non_full_cycle.append(S)
                if len(non_full_cycle)>=2:
                    a=non_full_cycle[0]
                    b=non_full_cycle[1]
                    #约束6,非满载回路合并约束,仅对level2模型启用,作为lazy constraint动态加入,消除多条非满载回路
                    model.cbLazy(quicksum(x[a[i],a[i+1]] for i in range(len(a)-1))+quicksum(x[b[i],b[i+1]] for i in range(len(b)-1))<=len(a)+len(b)-3)

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

    # 检查求解状态
    if MD.status != GRB.OPTIMAL:
        print(f"\n⚠️ 模型求解失败! 状态码: {MD.status}")
        if MD.status == GRB.INFEASIBLE:
            print("模型不可行 - 约束冲突，无法找到满足所有约束的解")
            print(f"需求总和: {sum(D.values())}, 容量Q: {Q}, 所需车辆数: {len(V)}")
            MD.computeIIS()
            print("不可行约束已写入文件 model.ilp")
            MD.write("model.ilp")
        elif MD.status == GRB.UNBOUNDED:
            print("模型无界 - 目标函数可以无限优化")
        else:
            print(f"其他求解问题，详情请查看Gurobi状态码: {MD.status}")
        raise Exception("CVRP模型求解失败")

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


from visualization import visualize_stage1_routes, visualize_stage2_routes


if __name__ == "__main__":
    sum_obj=0

    # input,一阶段都用大写字母,二阶段都用小写字母,主要参数B,C,D,a,b,c,d
    # 一阶段数据
    B=[i for i in range(NUM_BUILDINGS + 1)]     #仓库(0)和楼栋编号
    C={}                                        #距离
    for i in B:
        for j in B:
            if i != j:
                distance = random.randint(MIN_BUILDING_DISTANCE, MAX_BUILDING_DISTANCE)
                C[(i,j)] = distance
                C[(j,i)] = distance
            else:
                C[i,j]=0
    D={i:random.randint(MIN_BUILDING_DEMAND, MAX_BUILDING_DEMAND) for i in B}  #随机需求
    D[0]=0                                      #仓库(0)需求为0


    #一阶段求解
    B_,C_,D_ = reform_data(B,C,D,level=1,Q=Q)
    CYCCLES,stage1_obj=CVRP(B_,C_,D_,level=1,Q=Q)
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
        if total_demand > Q:
            print(f"  ⚠️ 警告: 本路线总需求 {total_demand} 超过容量 Q={Q}! (此路线包含多个楼栋)")
        print()

    # 可视化一阶段路径
    if ENABLE_VISUALIZATION and SHOW_STAGE1_VIS:
        print("\n=== 生成一阶段可视化 ===")
        visualize_stage1_routes(B, C, D, ORIGINAL_CYCCLES)

    # 二阶段：根据原始需求生成房间需求并优化
    print("\n=== 二阶段：楼内配送优化 ===")
    print(f"配置: Q={Q}, 楼层数={NUM_FLOORS}, 每层房间数={ROOMS_PER_FLOOR}")
    
    stage2_total = 0
    vis_count = 0  # 用于限制可视化的楼栋数量
    
    # 引入 helper
    from data_inputs import generate_physical_network_for_demands

    # 1. 预先为每栋楼生成房间需求 (Pre-generate room demands)
    building_room_demands = {}
    for building_id in range(1, len(B)):
        if D[building_id] <= 0:
            continue
        
        # 生成所有房间列表
        rooms = []
        for f in range(1, NUM_FLOORS + 1):
            for r in range(1, ROOMS_PER_FLOOR + 1):
                rooms.append(f"{f}.{r}")
        
        random.shuffle(rooms)
        
        # 将总需求 D[building_id] 分配到各个房间
        d_rooms = {}
        remaining = D[building_id]
        
        for room_id in rooms:
            if remaining <= 0:
                break
            # 随机分配 1-4 个包裹
            rand_val = random.random()
            if rand_val < PROB_1_PACKAGE:
                packages = 1
            elif rand_val < PROB_1_PACKAGE + PROB_2_PACKAGES:
                packages = 2
            elif rand_val < PROB_1_PACKAGE + PROB_2_PACKAGES + PROB_3_PACKAGES:
                packages = 3
            else:
                packages = 4
            
            packages = min(packages, remaining)
            if packages > 0:
                d_rooms[room_id] = packages
                remaining -= packages
        
        # 如果还有剩余，分配给最后一个房间
        if remaining > 0 and d_rooms:
            last_room = list(d_rooms.keys())[-1]
            d_rooms[last_room] += remaining
        elif remaining > 0:
            d_rooms[rooms[0]] = remaining

        building_room_demands[building_id] = d_rooms
        print(f"楼栋 {building_id} 预生成房间需求 (总: {D[building_id]}): {d_rooms}")

    # 2. 针对每栋楼，使用其完整需求进行二阶段优化 (Optimize Stage 2 for each building using its full demand)
    # 不再根据 Stage 1 的拆分结果进行分批优化，而是对整栋楼的需求一次性优化
    
    for building_id in range(1, len(B)):
        total_demand = D[building_id]
        if total_demand <= 0:
            continue
            
        print(f"\n=== 楼栋 {building_id} (总需求: {total_demand}) ===")
        
        # 获取该楼栋的所有房间需求
        if building_id not in building_room_demands:
            print("无房间需求信息")
            continue
            
        d_rooms = building_room_demands[building_id]
        print(f"房间需求: {d_rooms}")
        
        if not d_rooms:
            print("无房间需求，跳过")
            continue
            
        # 生成物理网络 (针对整栋楼的所有需求点)
        a, b, c, d = generate_physical_network_for_demands(
            room_demands=d_rooms,
            rooms_per_floor=ROOMS_PER_FLOOR,
            k=FLOOR_DISTANCE,
            con_cost=CON_COST,
            intra_floor_weight=ROOM_DISTANCE,
            include_floor1=True,
            close_loop=True
        )
        
        # 二阶段求解
        b_, c_, d_ = reform_data(b, c, d, level=2, Q=Q)
        cycles, obj = CVRP(b_, c_, d_, Q=Q, level=2)
        stage2_total += obj
        
        # 结果输出：全向图路径
        original_cycles = []
        cycle_demands = []
        for i in cycles:
            original_cycle = []
            trip_demand = 0
            for node in i:
                original_cycle.append(b_[node])
                if node != 0:  # 不计算depot
                    trip_demand += d_[node]
            original_cycles.append(original_cycle + ['1.0'])
            cycle_demands.append(trip_demand)
        
        print(f"楼内全向图配送方案: {original_cycles}")
        print(f"每趟载重: {cycle_demands} (Q={Q})")
        
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
            
        # 可视化二阶段路径（限制可视化的楼栋数量）
        if ENABLE_VISUALIZATION and SHOW_STAGE2_VIS and vis_count < MAX_STAGE2_BUILDINGS_VIS:
            visualize_stage2_routes(building_id, b, a, c, d, physical_path, original_cycles, cycle_demands)
            vis_count += 1
    
    sum_obj += stage2_total
    print(f"\n=== 总配送距离: {sum_obj} (一阶段: {stage1_obj}, 二阶段: {stage2_total}) ===")