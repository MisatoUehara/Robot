from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

random.seed(0)

def read_data():

    #input重构，拆分需求大于4的节点
    B_ = {0: 0}  # depot保持原样
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
    C_ = {}
    for i in B_:
        for j in B_:
            if i != j:
                C_[(i,j)] = C[(B_[i], B_[j])]

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
    MD1 = Model()

    x  = MD1.addVars([(i,j) for (i,j) in C], vtype=GRB.BINARY)

    #约束1,每个客户节点必须有且仅有一条出边（除仓库外）
    MD1.addConstrs(quicksum(x[i,j] for j in B if j!=i)==1 for i in B if i!=0)
    #约束2,每个客户节点必须有且仅有一条入边（除仓库外）
    MD1.addConstrs(quicksum(x[i,j] for i in B if j!=i)==1 for j in B if j!=0)
    #约束3,从仓库出发的车辆数等于返回仓库的车辆数（流平衡约束）
    MD1.addConstr(quicksum(x[0,j] for j in B if j!=0)==quicksum(x[i,0] for i in B if i!=0))
    # #约束4,从仓库出发的车辆数为所需最少车辆数
    MD1.addConstr(quicksum(x[0, j] for j in B if j != 0) == len(V))

    #目标函数
    MD1.setObjective(quicksum(C[i,j]*x[i,j] for (i,j) in C), GRB.MINIMIZE)
    MD1.Params.lazyConstraints = 1
    # MD1.Params.OutputFlag=0 #不输出求解过程
    MD1.optimize(Cut)

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

    return cycles


if __name__ == "__main__":


    # input,一阶段都用大写字母,二阶段都用小写字母
    # 一阶段数据
    B=[i for i in range(15)]                    #仓库(0)和楼栋编号
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
    B_,C_,D_ = read_data()
    CYCCLES=CVRP(B_,C_,D_)

    # 将cycle中的B_路径转换为原始的B路径
    ORIGINAL_CYCCLES = []
    for i in CYCCLES:
        original_cycle = []
        for node in i:
            original_cycle.append(B_[node])  # 转换为原始楼栋编号
        ORIGINAL_CYCCLES.append(original_cycle+[0])

    print("需求:",D)
    print("大楼配送方案:", ORIGINAL_CYCCLES,"共%d次"%(len(ORIGINAL_CYCCLES)))