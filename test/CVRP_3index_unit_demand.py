from gurobipy import *
import math,random,networkx
import numpy as np

random.seed(0)

def read_data():
    
    global B,R,K,V,Q,c,c_x,c_y,d
    n=10
    B=[i for i in range(n)]  #0代表depot
    d=[random.randint(0,8) for i in range(n)] #需求
    Q=4 #容量

    c_x={i:random.uniform(0,10) for i in B}
    c_y={i:random.uniform(0,10) for i in B}

    B_ = {0: 0}  # depot保持原样
    idx = 1
    for i in range(1, n):
        for _ in range(d[i]):
            B_[idx] = i
            idx += 1
    
    c_x_ = {i: c_x[B_[i]] for i in B_}
    c_y_ = {i: c_y[B_[i]] for i in B_}

    c = {}
    for i in B_:
        for j in B_:
            if i != j:
                distance = math.sqrt((c_x_[i] - c_x_[j])**2 + (c_y_[i] - c_y_[j])**2)
                c[(i,j)] = round(distance)

    V = [i for i in range(math.ceil(sum(d)/Q))] #车辆数
    B = B_
    print(V)
    print(B_)
    # print(c)
    # exit()

def model():
    def Cut(model, where):
        if where == GRB.Callback.MIPSOL:
            for v in V:
                edges = [(i,j) for (i,j) in c if model.cbGetSolution(x[i,j,v]) > 0.5]
                G = networkx.DiGraph()
                G.add_edges_from(edges)
                
                # 找到不包含depot的连通分量
                undirected_G = G.to_undirected()
                components = list(networkx.connected_components(undirected_G))
                
                for S in components:
                    if 0 not in S and len(S) > 0:  # 不包含depot的子回路
                        model.cbLazy(quicksum(x[i,j,v] for i in S for j in S if (i,j) in c) <= len(S)-1)


    MD1 = Model()

    x  = MD1.addVars([(i,j,v) for (i,j) in c for v in V], vtype=GRB.BINARY)
    y  = MD1.addVars([(i,v) for i in B for v in V], vtype=GRB.CONTINUOUS)

    #1
    # MD1.addConstrs(quicksum(x[i,j,v] for i in B for v in V if i!=j)==1 for j in B)
    MD1.addConstrs(quicksum(x[i,j,v] for i in B for v in V if i!=j)==1 for j in B if j!=0)
    #2
    # MD1.addConstrs(quicksum(x[0,j,v] for j in V if j!=0)<=1 for v in V)
    MD1.addConstrs(quicksum(x[0,j,v] for j in B if j!=0)<=1 for v in V)
    #3
    MD1.addConstrs(quicksum(x[i,p,v] for i in B if i!=p)-quicksum(x[p,j,v] for j in B if j!=p)==0 for p in B for v in V)
    # #4
    # MD1.addConstrs(quicksum(x[0,j,v] for j in V if j!=0)<=1 for v in V)
    #5
    # MD1.addConstrs(quicksum(x[i,j,v] for i in V for j in V if i!=j)<=Q for v in V)
    MD1.addConstrs(quicksum(x[i,j,v] for j in B if j!=0 for i in B if i!=j)<=Q for v in V)

    #对称性
    MD1.addConstrs(quicksum(x[i,j,v-1] for i in B for j in B if i!=j)>=quicksum(x[i,j,v] for i in B for j in B if i!=j) for v in V if v!=0)


    #1
    MD1.setObjective(quicksum(c[i,j]*x[i,j,v] for (i,j) in c for v in V), GRB.MINIMIZE)

    MD1.Params.lazyConstraints = 1
    MD1.optimize(Cut)
    for v in V:
        print(f'\nVehicle {v} route:')
        for (i,j) in c:
            if x[i,j,v].X > 0.5:
                print(i,j)


if __name__ == "__main__":
    read_data()
    model()


    import matplotlib.pyplot as plt

    plt.figure(figsize=(8, 6))
    for i in B:
        plt.scatter(c_x[i], c_y[i], s=100)
        plt.annotate(str(i), (c_x[i], c_y[i]), xytext=(5, 5), textcoords='offset points')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Node Locations')
    plt.grid(True, alpha=0.3)
    plt.show()