from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# ============================================================================
# 配置参数 / Configuration Parameters
# ============================================================================
RANDOM_SEED = 42                # 随机种子 / Random seed for reproducibility
Q = 4                           # 机器人载重量 / Robot capacity (packages)

# 一阶段参数 / Stage 1 Parameters (Building-level delivery)
NUM_BUILDINGS = 5              # 楼栋数量（不包括仓库）/ Number of buildings (excluding depot)
MIN_BUILDING_DISTANCE = 20      # 楼栋间最小距离 / Min distance between buildings
MAX_BUILDING_DISTANCE = 50      # 楼栋间最大距离 / Max distance between buildings
MIN_BUILDING_DEMAND = 0         # 楼栋最小需求 / Min demand per building
MAX_BUILDING_DEMAND = 20        # 楼栋最大需求 / Max demand per building

# 二阶段参数 / Stage 2 Parameters (Room-level delivery)
NUM_FLOORS = 5                  # 每栋楼层数 / Number of floors per building
ROOMS_PER_FLOOR = 3             # 每层房间数 / Number of rooms per floor
FLOOR_DISTANCE = 5              # 楼层间距离（垂直移动成本）/ Distance between floors (vertical cost)
ROOM_DISTANCE = 1               # 同层房间间距离（水平移动成本）/ Distance between rooms on same floor (horizontal cost)

# 房间需求分配概率 / Room demand allocation probability
PROB_1_PACKAGE = 0.70           # 1个包裹的概率 / Probability of 1 package
PROB_2_PACKAGES = 0.20          # 2个包裹的概率 / Probability of 2 packages (cumulative: 0.90)
PROB_3_PACKAGES = 0.07          # 3个包裹的概率 / Probability of 3 packages (cumulative: 0.97)
# 其余概率为4个包裹 / Remaining probability is for 4 packages

# 可视化参数 / Visualization Parameters
ENABLE_VISUALIZATION = False     # 启用可视化 / Enable visualization
SHOW_STAGE1_VIS = True          # 显示一阶段可视化 / Show stage 1 visualization
SHOW_STAGE2_VIS = True          # 显示二阶段可视化 / Show stage 2 visualization
MAX_STAGE2_BUILDINGS_VIS = 3    # 最多显示几个楼栋的二阶段路径 / Max buildings to visualize in stage 2
# ============================================================================

random.seed(RANDOM_SEED)  # Change this number to get different random results

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
def CVRP(B,C,D,Q=Q):
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


def visualize_stage1_routes(B, C, D, ORIGINAL_CYCCLES, building_positions=None):
    """可视化一阶段楼栋配送路径"""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE1_VIS:
        return
    
    # 如果没有提供位置，使用随机布局
    if building_positions is None:
        # 使用距离矩阵创建图
        G = nx.Graph()
        G.add_nodes_from(B)
        for i in B:
            for j in B:
                if i < j and i in C and (i, j) in C:
                    G.add_edge(i, j, weight=C[(i, j)])
        building_positions = nx.spring_layout(G, k=2, iterations=50, seed=RANDOM_SEED)
    
    # 为每条路线使用不同颜色
    colors = plt.cm.tab20(np.linspace(0, 1, len(ORIGINAL_CYCCLES)))
    
    plt.figure(figsize=(14, 10))
    
    # 绘制所有节点
    depot_pos = building_positions[0]
    plt.scatter([depot_pos[0]], [depot_pos[1]], s=800, c='red', marker='s', 
                label='Depot', zorder=5, edgecolors='black', linewidths=2)
    
    for building in B[1:]:
        pos = building_positions[building]
        plt.scatter([pos[0]], [pos[1]], s=500, c='lightblue', marker='o', 
                   zorder=4, edgecolors='black', linewidths=1.5)
        plt.text(pos[0], pos[1], f'{building}\n({D[building]})', 
                ha='center', va='center', fontsize=10, fontweight='bold')
    
    # 绘制每条路线
    for idx, (route, color) in enumerate(zip(ORIGINAL_CYCCLES, colors)):
        for i in range(len(route) - 1):
            start, end = route[i], route[i+1]
            start_pos = building_positions[start]
            end_pos = building_positions[end]
            
            # 绘制箭头
            dx = end_pos[0] - start_pos[0]
            dy = end_pos[1] - start_pos[1]
            plt.arrow(start_pos[0], start_pos[1], dx*0.85, dy*0.85,
                     head_width=0.03, head_length=0.02, fc=color, ec=color,
                     alpha=0.6, length_includes_head=True, zorder=3)
    
    plt.text(depot_pos[0], depot_pos[1], 'Depot\n0', 
            ha='center', va='center', fontsize=11, fontweight='bold', color='white')
    
    plt.title(f'Stage 1: Building Delivery Routes (Q={Q}, Total Routes: {len(ORIGINAL_CYCCLES)})', 
              fontsize=14, fontweight='bold')
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('stage1_building_routes.png', dpi=150, bbox_inches='tight')
    print("  → Stage 1 visualization saved as 'stage1_building_routes.png'")
    plt.show()


def visualize_stage2_routes(building_id, b, a, c, d, physical_paths, original_cycles, cycle_demands):
    """可视化二阶段楼内房间配送路径"""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE2_VIS:
        return
    
    # 创建图
    G = nx.Graph()
    G.add_nodes_from(b)
    G.add_edges_from(a)
    for (i, j) in a:
        G[i][j]['weight'] = c[(i, j)]
    
    # 计算节点位置：楼层垂直排列，房间水平排列
    pos = {}
    for node in b:
        if node == '1.0':
            pos[node] = (0, 0)
        else:
            parts = node.split('.')
            floor = int(parts[0])
            room = int(parts[1])
            
            if room == 0:  # 楼层入口
                pos[node] = (0, floor * 2)
            else:  # 房间
                pos[node] = (room * 1.5, floor * 2)
    
    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # 绘制所有边（物理网络）
    for (u, v) in a:
        x = [pos[u][0], pos[v][0]]
        y = [pos[u][1], pos[v][1]]
        ax.plot(x, y, 'gray', alpha=0.3, linewidth=1, zorder=1)
    
    # 为每条路线使用不同颜色
    colors = plt.cm.Set3(np.linspace(0, 1, len(physical_paths)))
    
    # 绘制每条配送路径
    for idx, (path, color, demand) in enumerate(zip(physical_paths, colors, cycle_demands)):
        for i in range(len(path) - 1):
            start, end = path[i], path[i+1]
            x = [pos[start][0], pos[end][0]]
            y = [pos[start][1], pos[end][1]]
            ax.plot(x, y, color=color, linewidth=3, alpha=0.7, 
                   label=f'Trip {idx+1} (Load: {demand})' if i == 0 else '', zorder=2)
            
            # 添加箭头
            if i < len(path) - 2:  # 不在最后一段添加箭头
                mid_x = (x[0] + x[1]) / 2
                mid_y = (y[0] + y[1]) / 2
                dx = x[1] - x[0]
                dy = y[1] - y[0]
                ax.annotate('', xy=(mid_x + dx*0.1, mid_y + dy*0.1), 
                           xytext=(mid_x - dx*0.1, mid_y - dy*0.1),
                           arrowprops=dict(arrowstyle='->', color=color, lw=2), zorder=3)
    
    # 绘制节点
    for node in b:
        x, y = pos[node]
        if node == '1.0':  # 入口
            ax.scatter(x, y, s=500, c='red', marker='s', zorder=5, edgecolors='black', linewidths=2)
            ax.text(x, y-0.3, 'Entrance', ha='center', fontsize=9, fontweight='bold')
        elif node.endswith('.0'):  # 楼层入口
            ax.scatter(x, y, s=300, c='orange', marker='d', zorder=4, edgecolors='black', linewidths=1.5)
            ax.text(x-0.3, y, f'F{node.split(".")[0]}', ha='right', fontsize=8)
        else:  # 房间
            demand = d.get(node, 0)
            if demand > 0:
                ax.scatter(x, y, s=400, c='lightgreen', marker='o', zorder=4, 
                          edgecolors='darkgreen', linewidths=2)
                ax.text(x, y, f'{node}\n[{demand}]', ha='center', va='center', 
                       fontsize=7, fontweight='bold')
            else:
                ax.scatter(x, y, s=200, c='lightgray', marker='o', zorder=3, 
                          edgecolors='gray', linewidths=1)
                ax.text(x, y+0.3, node, ha='center', fontsize=6, color='gray')
    
    ax.set_title(f'Stage 2: Building {building_id} Room Delivery Routes\n'
                f'Total Demand: {sum(d.values())}, Trips: {len(physical_paths)}, Q={Q}',
                fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Room Position', fontsize=10)
    ax.set_ylabel('Floor', fontsize=10)
    ax.axis('equal')
    plt.tight_layout()
    plt.savefig(f'stage2_building{building_id}_routes.png', dpi=150, bbox_inches='tight')
    print(f"  → Building {building_id} visualization saved as 'stage2_building{building_id}_routes.png'")
    plt.show()


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

    # 可视化一阶段路径
    if ENABLE_VISUALIZATION and SHOW_STAGE1_VIS:
        print("\n=== 生成一阶段可视化 ===")
        visualize_stage1_routes(B, C, D, ORIGINAL_CYCCLES)

    # 二阶段：根据原始需求生成房间需求并优化
    print("\n=== 二阶段：楼内配送优化 ===")
    print(f"配置: Q={Q}, 楼层数={NUM_FLOORS}, 每层房间数={ROOMS_PER_FLOOR}")
    
    stage2_total = 0
    vis_count = 0  # 用于限制可视化的楼栋数量
    
    # 为每栋有需求的楼栋生成房间需求并优化
    for building_id in range(1, len(B)):
        if D[building_id] <= 0:
            continue
        
        total_demand = D[building_id]
        print(f"\n--- 楼栋 {building_id} (总需求: {total_demand}) ---")
        
        # 生成物理网络：楼层链式结构，每层有多个房间
        a = []  # 边
        b = ['1.0']  # 节点，从入口开始
        
        # 为每层生成节点和边
        for floor in range(1, NUM_FLOORS + 1):
            floor_entrance = f"{floor}.0"
            b.append(floor_entrance)
            
            # 连接入口到楼层入口（或上一层到当前层）
            if floor == 1:
                a.append(('1.0', floor_entrance))
            else:
                prev_floor = f"{floor-1}.0"
                a.append((prev_floor, floor_entrance))
            
            # 为该层生成房间
            for room in range(1, ROOMS_PER_FLOOR + 1):
                room_id = f"{floor}.{room}"
                b.append(room_id)
                if room == 1:
                    a.append((floor_entrance, room_id))
                else:
                    prev_room = f"{floor}.{room-1}"
                    a.append((prev_room, room_id))
        
        # 设置边的权重：楼层间距离为5，同层房间间距离为1
        c = {}
        for (u, v) in a:
            # 判断是否为楼层间的边
            u_floor = int(u.split('.')[0]) if u != '1.0' else 1
            v_floor = int(v.split('.')[0])
            if u == '1.0' or (u.endswith('.0') and v.endswith('.0')):
                c[(u, v)] = FLOOR_DISTANCE  # 楼层间距离
            else:
                c[(u, v)] = ROOM_DISTANCE  # 同层房间间距离
        
        # 生成房间需求：随机分配到各个房间
        all_rooms = [node for node in b if not node.endswith('.0') and node != '1.0']
        random.shuffle(all_rooms)
        
        d = {}
        remaining = total_demand
        for room_id in all_rooms:
            if remaining <= 0:
                break
            # 每个房间随机分配1-4个包裹
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
                d[room_id] = packages
                remaining -= packages
        
        # 如果还有剩余需求，分配到最后一个房间
        if remaining > 0 and d:
            last_room = list(d.keys())[-1]
            d[last_room] += remaining
        
        print(f"房间需求: {d}")
        
        if not d:
            print("无房间需求，跳过")
            continue
        
        # 二阶段求解
        b_, c_, d_ = reform_data(b, c, d, level=2)
        cycles, obj = CVRP(b_, c_, d_, Q=Q)
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