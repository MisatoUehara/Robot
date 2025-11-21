from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

random.seed(0)

def read_data():
    global c,b,d,v,Q,c_
    
    # input,一阶段都用大写字母,二阶段都用小写字母
    # 一阶段数据
    B=[i for i in range(15)]                    #仓库(0)和楼栋编号

    X={i:random.uniform(0,10) for i in B}       #坐标x
    Y={i:random.uniform(0,10) for i in B}       #坐标y

    D={i:random.randint(0,10) for i in B}       #需求
    D[0]=0                                      #仓库(0)需求为0

    Q=4                                         #机器人容量




    #input重构，拆分需求大于4的节点
    B_ = {0: 0}  # depot保持原样
    D_ = {0: 0}  # depot需求为0
    idx = 1

    for i in range(1, len(B)):
        remaining_demand = D[i]
        while remaining_demand > 0:
            # 每个新节点的需求最多为4,超过4则在原位置复制一个节点
            current_demand = min(remaining_demand, 4)
            B_[idx] = i
            D_[idx] = current_demand
            remaining_demand -= current_demand
            idx += 1

    # 更新坐标
    X_ = {i: X[B_[i]] for i in B_}
    Y_ = {i: Y[B_[i]] for i in B_}

    # 计算距离矩阵
    C_ = {}
    for i in B_:
        for j in B_:
            if i != j:
                distance = math.sqrt((X_[i] - X_[j])**2 + (Y_[i] - Y_[j])**2)
                C_[(i,j)] = round(distance)

    # 机器人派出次数编号
    V = [i for i in range(math.ceil(sum(D.values())/Q))]
    print(D)
    return C_,B_,D_,V,Q

#CVRP模型,一阶段处理楼栋配送,二阶段处理楼内配送
def CVRP(C,B,D,V,Q):

    #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in C if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.DiGraph()
            G.add_edges_from(edges)

            cycles = list(networkx.simple_cycles(G))
            for S in cycles:
                S = [i for i in S if i != 0]
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
    #约束4,从仓库出发的车辆数至少为所需最少车辆数
    MD1.addConstr(quicksum(x[0, j] for j in B if j != 0) >= len(V))

    #目标函数
    MD1.setObjective(quicksum(C[i,j]*x[i,j] for (i,j) in C), GRB.MINIMIZE)
    MD1.Params.lazyConstraints = 1
    MD1.Params.OutputFlag=0 #不输出求解过程
    MD1.optimize(Cut)

    edges = [(i,j) for (i,j) in C if x[i,j].X > 0.5]
    print(edges)

    G = networkx.DiGraph()
    G.add_edges_from(edges)
    cycles = list(networkx.simple_cycles(G))
    print("Cycles:", cycles)

    return cycles


if __name__ == "__main__":
    C_,B_,D_,V,Q = read_data()
    cycles=CVRP(C_,B_,D_,V,Q)

    # 将cycle中的B_路径转换为原始的B路径
    original_cycles = []
    for i in cycles:
        original_cycle = []
        for node in i:
            original_cycle.append(B_[node])  # 转换为原始楼栋编号
        original_cycles.append(original_cycle+[0])

    print("大楼配送方案:", original_cycles)


