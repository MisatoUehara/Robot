from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx

random.seed(0)

def read_data():
    global B,R,K,V,Q,x,y,d,B_,d_,c_,x_,y_
    
    #input
    n=15                                        #仓库(0)和楼栋数
    B=[i for i in range(n)]                     #仓库(0)和楼栋编号

    x={i:random.uniform(0,10) for i in B}       #坐标x
    y={i:random.uniform(0,10) for i in B}       #坐标y

    d=[random.randint(0,10) for i in range(n)]  #需求
    d[0]=0                                      #仓库(0)需求为0

    Q=4                                         #机器人容量




    #input重构，拆分需求大于4的节点
    B_ = {0: 0}  # depot保持原样
    d_ = {0: 0}  # depot需求为0
    idx = 1

    for i in range(1, n):
        remaining_demand = d[i]
        while remaining_demand > 0:
            # 每个新节点的需求最多为4,超过4则在原位置复制一个节点
            current_demand = min(remaining_demand, 4)
            B_[idx] = i
            d_[idx] = current_demand
            remaining_demand -= current_demand
            idx += 1

    # 更新坐标
    x_ = {i: x[B_[i]] for i in B_}
    y_ = {i: y[B_[i]] for i in B_}

    # 计算距离矩阵
    c_ = {}
    for i in B_:
        for j in B_:
            if i != j:
                distance = math.sqrt((x_[i] - x_[j])**2 + (y_[i] - y_[j])**2)
                c_[(i,j)] = round(distance)

    # 机器人派出次数编号
    V = [i for i in range(math.ceil(sum(d)/Q))]

def model():



    #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in c_ if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.DiGraph()
            G.add_edges_from(edges)

            Cycles = list(networkx.simple_cycles(G))
            for S in Cycles:
                S = [i for i in S if i != 0]
                #约束5,子回路消除和载重约束约束,作为lazy constraint动态加入,消除子回路和超载回路
                model.cbLazy(quicksum(x[i,j] for i in B_ if i not in S for j in S if i!=j) >= math.ceil(sum(d_[i] for i in S) / Q))

    MD1 = Model()

    x  = MD1.addVars([(i,j) for (i,j) in c_], vtype=GRB.BINARY)

    #约束1,每个客户节点必须有且仅有一条出边（除仓库外）
    MD1.addConstrs(quicksum(x[i,j] for j in B_ if j!=i)==1 for i in B_ if i!=0)
    #约束2,每个客户节点必须有且仅有一条入边（除仓库外
    MD1.addConstrs(quicksum(x[i,j] for i in B_ if j!=i)==1 for j in B_ if j!=0)
    #约束3,从仓库出发的车辆数等于返回仓库的车辆数（流平衡约束）
    MD1.addConstr(quicksum(x[0,j] for j in B_ if j!=0)==quicksum(x[i,0] for i in B_ if i!=0))
    #约束4,从仓库出发的车辆数至少为所需最少车辆数
    MD1.addConstr(quicksum(x[0, j] for j in B_ if j != 0) >= len(V))

    #目标函数
    MD1.setObjective(quicksum(c_[i,j]*x[i,j] for (i,j) in c_), GRB.MINIMIZE)
    MD1.Params.lazyConstraints = 1
    # MD1.Params.OutputFlag=0 #不输出求解过程
    MD1.optimize(Cut)

    edges = [(i,j) for (i,j) in c_ if x[i,j].X > 0.5]
    print(edges)
    G = networkx.DiGraph()
    G.add_edges_from(edges)
    Cycles = list(networkx.simple_cycles(G))
    print("Cycles:", Cycles)


    import matplotlib.pyplot as plt
    def plot_routes():
        plt.figure(figsize=(10, 8))
        
        # 设置科研风格
        plt.style.use('default')
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.linewidth'] = 1.2
        
        # 绘制节点
        # Depot (节点0) - 红色正方形
        plt.scatter(x_[0], y_[0], c='red', s=200, marker='s', 
                   label='Depot', edgecolors='black', linewidth=2, zorder=5)
        
        # 客户节点 - 蓝色圆圈
        customer_nodes = [i for i in B_ if i != 0]
        customer_x = [x_[i] for i in customer_nodes]
        customer_y = [y_[i] for i in customer_nodes]
        plt.scatter(customer_x, customer_y, c='blue', s=150, marker='o', 
                   label='Customers', edgecolors='black', linewidth=2, zorder=4)
        
        # 绘制路径
        for (i, j) in edges:
            plt.arrow(x_[i], y_[i], x_[j] - x_[i], y_[j] - y_[i], 
                     head_width=0.15, head_length=0.1, fc='gray', ec='gray',
                     length_includes_head=True, zorder=3)
        
        # 添加节点标签
        for i in B_:
            plt.annotate(f'{i}({d_[i]})', (x_[i], y_[i]), 
                        xytext=(5, 5), textcoords='offset points',
                        fontsize=9, ha='left')
        
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Capacity Vehicle Routing Problem Solution')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.show()

    plot_routes()


if __name__ == "__main__":
    read_data()
    model()

