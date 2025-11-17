from gurobipy import *
import math,random,networkx
import numpy as np

random.seed(0)

def read_data():
    
    global B,R,K,V,Q,c,d
    B=[0,1,2,3,4,5]  #0代表depot
    d=[0,1,2,3,4,5]

    Q=4
    V = [i for i in range(math.ceil(sum(d)/Q))]
    c={}#无法用一行快速表示，会导致距离不对称
    for i in B:
        for j in B:
            if i < j:
                distance = random.randint(1, 10)
                c[(i,j)] = distance
                c[(j,i)] = distance


def model():
    #4,subtour elimination cut
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            for v in V:
                edges = [(i,j) for (i,j) in c if model.cbGetSolution(x[i,j,v]) > 0.5]
                G = networkx.Graph()
                G.add_edges_from(edges)
                Components = list(networkx.connected_components(G))

                #4,subtour elimination cut
                for S in Components:
                    if 0 not in S:
                        model.cbLazy(quicksum(x[i,j,v] for i in S for j in S if j!=i) <= len(S)-1)

    MD1 = Model()

    x  = MD1.addVars([(i,j,v) for (i,j) in c for v in V], vtype=GRB.BINARY)
    y  = MD1.addVars([(i,v) for i in B for v in V], vtype=GRB.CONTINUOUS)

    #2
    MD1.addConstrs(quicksum(x[i,j,v] for i in B for v in V if i!=j)>=1 for j in B)
    #3
    MD1.addConstrs(quicksum(x[i,p,v] for i in B if i!=p)-quicksum(x[p,j,v] for j in B if j!=p)==0 for p in B for v in V)
    #5
    MD1.addConstrs(y[i,v]<=d[i]*quicksum(x[i,j,v] for j in B if j!=i) for i in B if i!=0 for v in V)
    #6
    MD1.addConstrs( quicksum(y[i,v] for v in V)==d[i] for i in B if i!=0)
    #7
    MD1.addConstrs( quicksum(y[i,v] for i in B if i!=0)<=Q for v in V)


    #1
    MD1.setObjective(quicksum(c[i,j]*x[i,j,v] for (i,j) in c for v in V), GRB.MINIMIZE)
    MD1.Params.lazyConstraints = 1
    MD1.optimize(Cut)



if __name__ == "__main__":
    read_data()
    model()
