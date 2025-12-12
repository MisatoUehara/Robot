from gurobipy import *
import math,random,networkx,time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

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
def CVRP(B,C,D,level,Q=4):
    timeValue=[]
    Lowerbound=[]
    Upperbound=[]
    start_time = time.time()
    last_record_time = start_time
    V = [i for i in range(math.ceil(sum(D.values())/Q))]
    def Cut(model,where):
        #获取求解信息
        nonlocal last_record_time  # 声明使用外层变量
        if where == GRB.Callback.MIP:
            current_time = time.time()
            if current_time - last_record_time >= 0.5: #时间参数，每0.5秒记录一次
                timeValue.append(model.cbGet(GRB.Callback.RUNTIME))
                Lowerbound.append(model.cbGet(GRB.Callback.MIP_OBJBND))
                Upperbound.append(model.cbGet(GRB.Callback.MIP_OBJBST))
                last_record_time = current_time
        #添加Cut
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
    # MD.Params.OutputFlag=0 #如果你想看gurobi的求解过程,可以注释掉这一行。但是，不管注释与否,都能画出上下界变化图。
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

    # 记录最后一个点(可能没有到达Cut中触发时间)
    if MD.Status == GRB.OPTIMAL:        #若最优解
        timeValue.append(MD.Runtime)
        Lowerbound.append(MD.ObjVal)    #最优解时,则上下界相等,直接添加最优解即可
        Upperbound.append(MD.ObjVal)    #最优解时,则上下界相等,直接添加最优解即可
    # 将Upperbound中大于等于1e+100的数值替换为None,避免画图出现极大值影响视觉
    Upperbound = [None if val  >= 1e+100 else val for val in Upperbound]
    # 画图
    plt.figure(figsize=(10, 6))
    plt.plot(timeValue, Upperbound, 'r-', linewidth=2, label='Upper Bound', marker='s', markersize=4) #上界，MILP的整数可行解，LB=UB时为最优解
    plt.plot(timeValue, Lowerbound, 'b-', linewidth=2, label='Lower Bound', marker='o', markersize=4) #下界，MILP的线性松弛解，LB=UB时为最优解
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Objective Value', fontsize=12)
    plt.title('Convergence of Lower and Upper Bounds', fontsize=14, fontweight='bold')
    plt.legend(loc='upper left')
    plt.show()

    return cycles,MD.ObjVal


if __name__ == "__main__":
    sum_obj=0

    # input,一阶段都用大写字母,二阶段都用小写字母,主要参数B,C,D,a,b,c,d
    # 一阶段数据
    B=[i for i in range(18)]                    #仓库(0)和楼栋编号
    C={}                                        #距离
    for i in B:
        for j in B:
            if i != j:
                distance = random.randint(20, 50)
                C[(i,j)] = distance
                C[(j,i)] = distance
            else:
                C[i,j]=0
    D={i:random.randint(0,40) for i in B}       #随机需求
    D[0]=0                                      #仓库(0)需求为0


    #一阶段求解
    B_,C_,D_ = reform_data(B,C,D,level=1)
    CYCCLES,obj=CVRP(B_,C_,D_,level=1)
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


    # 二阶段数据
    a=[('1.0','1.1'),('1.1','1.2'),('1.2','1.3'),('1.0','2.0'),('2.0','2.1'),('2.1','2.2'),('2.2','2.3'),('2.0','2.4'),('2.4','2.5'),('2.0','3.0'),('3.0','3.1')]#物理网络,弧
    b=['1.0','1.1','1.2','1.3','2.0','2.1','2.2','2.3','2.4','2.5','3.0','3.1'] #物理网络,点  1.0,2.0,3.0为楼层入口,其余为房间号,
    c={(i,j):1 for (i,j) in a}                  #路网距离,先设定单位距离
    d={'2.1':5,'3.1':1}  #各房间需求

    # 二阶段求解
    b_,c_,d_ = reform_data(b,c,d,level=2)
    cycles,obj=CVRP(b_,c_,d_,level=2)
    sum_obj+=obj

    # 结果输出,全向图路径
    original_cycles = []
    for i in cycles:
        original_cycle = []
        for node in i:
            original_cycle.append(b_[node])  # 转换为原始楼栋编号
        original_cycles.append(original_cycle+['1.0'])

    print("楼内全向图配送方案:", original_cycles)

    # 结果输出,将全向图路径转换为物理网络路径
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
            # 避免重复添加节点（除了第一个段的起点）
            if i == 0:
                detailed_path.extend(segment_path)
            else:
                detailed_path.extend(segment_path[1:])  # 跳过起点避免重复
        physical_path.append(detailed_path)

    print("楼内物理网络配送方案:", physical_path)
    print("总配送距离:",sum_obj)