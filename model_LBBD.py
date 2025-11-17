"""
LBBD Model for BS-STSP

This module implements a Logic-Based Benders Decomposition (LBBD) 
for solving the battery swap stochastic traveling salesman problem (BS-STSP).

Output file:
    result_LBBD.txt
Output file format:
    instance    obj (kilometer)    solving_time (seconds)    gap    optimal_route (for stage 1)
Output file note:
    INF: infeasible instance.
    inf: no feasible solution found in timelimit.
    Instance and optimal route can be load on https://vrp-rep.github.io/mapper/

"""
import networkx,sys,time
from gurobipy import *

sys.path.append("..")
from para import *
from functions import *

def model(V_,scenarios,n,B):
    LB = -GRB.INFINITY
    UB = GRB.INFINITY
    ε = 1e-4
    # Add subtour cut
    def Cut(model,where):
        if where == GRB.Callback.MIPSOL:
            edges = [(i,j) for (i,j) in x if model.cbGetSolution(x[i,j]) > 0.5]
            G = networkx.Graph()
            G.add_edges_from(edges)
            Components = list(networkx.connected_components(G))
            # If no subtour, return
            if len(Components) == 1:
                return
            # Subtour cut
            for S in Components:
                if 0 not in S:
                    model.cbLazy(quicksum(x[i,j] for i in S for j in S if j!=i) <= len(S)-1)

    MP = Model('MP')

    # Make variables
    arcs_ = [(i,j) for i in V_ for j in V_ if j!=i]
    x  = MP.addVars(arcs_, vtype=GRB.BINARY, name='x')
    σ = MP.addVar(vtype=GRB.CONTINUOUS,name='σ')

    # Tour constraints
    # Outdegree constraints
    MP.addConstrs((quicksum(x[i,j] for j in V_ if j != i) == 1) for i in V_ if i != n+1)  # All nodes except destination have outdegree 1
    MP.addConstr(quicksum(x[n+1,j] for j in V_ if j != n+1) == 0)  # Destination has outdegree 0
    # Indegree constraints
    MP.addConstrs((quicksum(x[i,j] for i in V_ if i != j) == 1) for j in V_ if j != 0)  # All nodes except origin have indegree 1
    MP.addConstr(quicksum(x[i,0] for i in V_ if i != 0) == 0)  # Origin has indegree 0

    # Calculate the cost of the first stage using the expected value
    expected_c = {}
    for (i,j) in arcs_:
        expected_c[i,j] = sum(prob * c_scenario[i,j] for c_scenario, _, _, prob in scenarios)

    MP.setObjective(quicksum(x[i,j]*expected_c[i,j] for (i,j) in arcs_)+σ, GRB.MINIMIZE)
    MP.Params.OutputFlag = 0 
    start_time = time.process_time()
    MP.Params.lazyConstraints = 1
    MP.Params.Threads = 12
    MP.optimize(Cut)
    LB = max(LB, MP.objval)
    iteration = 0
    end_time = time.process_time()
    while (UB-LB)/UB >= ε and end_time-start_time<1000:


        # Make SP_s for every scenario
        scenario_obj = []
        for s, (c_scenario, d_scenario, d_scenario_, _) in enumerate(scenarios):
            SP = Model(f'SP_{s}')
            y  = SP.addVars(arcs_, vtype=GRB.BINARY, name='y')
            e  = SP.addVars(arcs_, vtype=GRB.CONTINUOUS, name='e')# Added mileage after a swap
            E = SP.addVars([i for i in V_], vtype=GRB.CONTINUOUS,ub=B, name='E')

            # xy constraints
            SP.addConstrs(x[i,j].X >= y[i,j]  for (i,j) in arcs_)

            # Mileage flow constraints
            SP.addConstr( E[0] == B)
            SP.addConstrs(E[j] <= E[i] - c_scenario[i,j]*x[i,j].X +B*(1-x[i,j].X+y[i,j]) for (i,j) in arcs_)
            SP.addConstrs(E[j] <= E[i] - (d_scenario[i,j]+d_scenario_[i,j])*y[i,j] + e[i,j] + B*(1-y[i,j]) for (i,j) in arcs_)

            # Battery capacity constraints for a swap
            SP.addConstrs(B>=E[i]-d_scenario[i,j]*y[i,j]+e[i,j] for (i,j) in arcs_)
            SP.addConstrs(e[i,j]<=B*y[i,j] for (i,j) in arcs_)
            # Mileage feasible constants
            SP.addConstrs(E[i]-c_scenario[i,j]*(x[i,j].X-y[i,j])>=0 for (i,j) in arcs_)
            SP.addConstrs(E[i]-d_scenario[i,j]*y[i,j]>=0 for (i,j) in arcs_)

            # Objective function minimize total detour distance
            SP.setObjective(quicksum(y[i,j]*(d_scenario[i,j]+d_scenario_[i,j]-c_scenario[i,j]) for (i,j) in arcs_), GRB.MINIMIZE)
            SP.Params.OutputFlag = 0 
            SP.optimize()

            if SP.Status == GRB.INFEASIBLE:
                scenario_obj.append(float('inf'))
                break # no need to solve other scenarios
            else:
                scenario_obj.append(SP.ObjVal)
        iteration += 1

        # Make equation for cuts
        eq = 0
        for i,j in arcs_:
            if x[i,j].X <= 0.5:
                eq += x[i,j]
            else:
                eq += 1 - x[i,j]

        # Update UB and add cuts
        if any(obj == float('inf') for obj in scenario_obj):
            # expected_obj=float('inf')
            # UB = min(UB, MP.ObjVal + expected_obj)
            # Feasibility cut
            MP.addConstr(eq >= 1)
        else:
            expected_obj = sum(prob * obj_val for obj_val, (_, _, _, prob) in zip(scenario_obj, scenarios))
            UB = min(UB, MP.ObjVal + expected_obj)
            # Optimality cut
            MP.addConstr(σ >= expected_obj * (1 - eq))

        # Resolve the master problem
        MP.Params.OutputFlag = 0 
        MP.Params.lazyConstraints = 1
        MP.Params.Threads = 12
        MP.optimize(Cut)
        LB = max(LB, MP.objval)
        end_time=time.process_time()

        # We annotate the following print statements to save time in computational experiment
        # print("Iteration{}".format(iteration))
        # print("UB:{}".format(UB))
        # print("LB:{}\n".format(LB))

    end_time=time.process_time()
    MP.Params.lazyConstraints = 1
    MP.Params.Threads = 12
    MP.optimize(Cut)
    print('Objective value',UB)
    print(end_time-start_time,'Seconds')

    # Get optimal route
    try:
        edges = [(i,j) for (i,j) in arcs_ if x[i,j].X > 0.5]
        optimal_route = make_route(edges)
        optimal_route += [0]
        route_str = " ".join(map(str, optimal_route))
    except:
        pass
    #no feasible solution
    if UB >= 1e+50:
        UB = float('inf')
        route_str = "inf"
    with open('result_LBBD.txt', 'a') as f:
        if MP.Status == GRB.OPTIMAL:
            f.write(f"{xml_file}\t{UB:.2f}\t{end_time-start_time:.2f}\t{(UB-LB)/UB*100:.1f}%\t{route_str}\n")
        elif MP.Status == GRB.INFEASIBLE:
            f.write(f"{xml_file}\tINF\t{end_time-start_time:.2f}\tINF\tINF\n")
        else:
            f.write(f"{xml_file}\tOther\t{end_time-start_time:.2f}\tOther\tOther\n")

if __name__ == "__main__":

    # Make instances
    directory = "..\..\data"
    xml_files_list = get_file_names(read_xml_files(directory))
    B=300 #battery size (kilometer)

    # Loop through instances
    for scenario_size in [1, 10, 50]:
        for customer_size in ['10', '20', '40']:
            for xml_file in xml_files_list:
                if xml_file[4:6] == customer_size:
                    V_,c,d,d_,n = read_xml_info(f"..\..\data\{xml_file}",copy_depot = True)
                    scenarios = generate_scenarios(c, d, d_, scenario_size)
                    model(V_,scenarios,n,B)
        print("",file=open('result_LBBD.txt', 'a'))  # Add a blank line after each scenarios size