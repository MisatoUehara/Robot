from gurobipy import *
import math,random,networkx
import numpy as np
import networkx as nx

random.seed(0)

def read_data():
    
    global B,R,K,V,Q,c,c_x,c_y,d
    n=10
    B=[i for i in range(n)]  #0代表depot
    d=[random.randint(0,6) for i in range(n)] #需求
    Q=4 #容量
    print(d)
    print(sum(d))
    # exit()
    c_x={i:random.uniform(0,10) for i in B}
    c_y={i:random.uniform(0,10) for i in B}

    #重构成单位需求的CVRP的参数
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
    #4,subtour elimination cut
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in c if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.DiGraph()
            G.add_edges_from(edges)

            Cycles = list(networkx.simple_cycles(G))
            #4,subtour elimination cut
            for S in Cycles:
                S = [i for i in S if i != 0]
                model.cbLazy(quicksum(x[i, j] for i in B if i not in S for j in S if i!=j) >= math.ceil(len(S) / Q))            # input()
                # model.cbLazy(quicksum(x[i, j] for i in B if i not in S for j in S if (i,j) in c) >= math.ceil(len(S) / 4))
    

    MD1 = Model()

    x  = MD1.addVars([(i,j) for (i,j) in c], vtype=GRB.BINARY)

    #1
    MD1.addConstrs(quicksum(x[i,j] for j in B if j!=i)==1 for i in B if i!=0)
    MD1.addConstrs(quicksum(x[i,j] for i in B if j!=i)==1 for j in B if j!=0)
    #2
    MD1.addConstr(quicksum(x[0,j] for j in B if j!=0)==quicksum(x[i,0] for i in B if i!=0))
    
    MD1.addConstr(quicksum(x[0, j] for j in B if j != 0) >= len(V))

    #1
    MD1.setObjective(quicksum(c[i,j]*x[i,j] for (i,j) in c), GRB.MINIMIZE)
    MD1.Params.lazyConstraints = 1
    MD1.optimize(Cut)

    edges = [(i,j) for (i,j) in c if x[i,j].X > 0.5]
    print(edges)
    G = networkx.Graph()
    G.add_edges_from(edges)
    


    import matplotlib.pyplot as plt

    # 创建并绘制网络图
    plt.figure(figsize=(10, 8))
    pos = nx.spring_layout(G, seed=42)
    nx.draw(G, pos, with_labels=True, node_color='lightblue', 
        node_size=500, font_size=10, font_weight='bold')
    nx.draw_networkx_edge_labels(G, pos, edge_labels={(i,j): f'{c[i,j]}' for (i,j) in G.edges()})
    plt.show()
    exit()

    for (i,j) in c:
        if x[i,j].X > 0.5:
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