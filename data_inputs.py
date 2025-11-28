"""data_inputs.py

Sample input definitions for `CVRP_2index_multi_demand.py`.

Provides:
- Stage 1: `B`, `C`, `D` (depot + buildings, distance dict and demands)
- Stage 2: `a`, `b`, `c`, `d` (physical network edges/nodes, edge weights and room demands)

Usage:
    from data_inputs import B, C, D, a, b, c, d

Keep these small, deterministic examples so the main solver can import them directly.
"""
import math

# Stage 1: depot (0) + buildings
B = [0, 1, 2, 3, 4]

# Distance matrix (symmetric). C[(i,j)] for i != j
_dist_matrix = [
    [0, 30, 25, 40, 35],
    [30, 0, 20, 30, 45],
    [25, 20, 0, 15, 30],
    [40, 30, 15, 0, 20],
    [35, 45, 30, 20, 0],
]
C = {}
for i in range(len(B)):
    for j in range(len(B)):
        if i != j:
            C[(i, j)] = _dist_matrix[i][j]

# Demands at each building (0 is depot)
D = {0: 0, 1: 6, 2: 2, 3: 4, 4: 3}

# Stage 2: physical network (string node names)
# Edges (undirected) between rooms and entrances
a = [
    ('1.0','1.1'),('1.1','1.2'),('1.2','1.3'),
    ('1.0','2.0'),('2.0','2.1'),('2.1','2.2'),('2.2','2.3'),
    ('2.0','2.4'),('2.4','2.5'),('2.0','3.0'),('3.0','3.1')
]

# Node list
b = ['1.0','1.1','1.2','1.3','2.0','2.1','2.2','2.3','2.4','2.5','3.0','3.1']

# Edge weights (set to 1 for each physical arc). Add both directions for convenience.
c = {}
for (u, v) in a:
    c[(u, v)] = 1
    c[(v, u)] = 1

# Room demands (only non-zero rooms listed)
d = {'2.1': 5, '3.1': 1}

# Expose helpers
def get_stage1():
    """Return (B, C, D) for stage 1."""
    return B, C, D

def get_stage2():
    """Return (a, b, c, d) for stage 2."""
    return a, b, c, d

__all__ = ['B', 'C', 'D', 'a', 'b', 'c', 'd', 'get_stage1', 'get_stage2', 'build_stage1_from_coords', 'generate_physical_network_by_index', 'generate_physical_network_for_demands', 'generate_networks_from_delivery_plan']


def build_stage1_from_coords(coords, demands=None, depot_index=0, metric='euclidean'):
    """Build stage-1 inputs (B, C, D) from node coordinates.

    Args:
        coords: either a list of (x,y) tuples (nodes indexed 0..n-1) or a dict mapping node ids -> (x,y).
        demands: optional list or dict of demands per node. If omitted, defaults to 0 for all nodes.
        depot_index: index/id of the depot node (its demand will be set to 0).
        metric: 'euclidean' (default) or 'manhattan' distance.

    Returns:
        B: list of node ids
        C: dict mapping (i,j) -> distance (float)
        D: dict mapping node id -> demand (int or float)
    """
    # Normalize coords into a dict id -> (x,y)
    if isinstance(coords, dict):
        pos = dict(coords)
        B = list(pos.keys())
    else:
        pos = {i: coords[i] for i in range(len(coords))}
        B = list(pos.keys())

    # Build distance dict C
    C = {}
    for i in B:
        xi, yi = pos[i]
        for j in B:
            if i == j:
                continue
            xj, yj = pos[j]
            if metric == 'euclidean':
                dist = math.hypot(xi - xj, yi - yj)
            else:
                dist = abs(xi - xj) + abs(yi - yj)
            C[(i, j)] = dist

    # Build demands dict D
    if demands is None:
        D = {i: 0 for i in B}
    else:
        if isinstance(demands, dict):
            D = {i: demands.get(i, 0) for i in B}
        else:
            D = {i: demands[i] for i in B}

    # Ensure depot demand is zero
    if depot_index in D:
        D[depot_index] = 0

    return B, C, D


def generate_physical_network_by_index(num_floors, rooms_per_floor, room_demands=None, k=10, intra_floor_weight=1, entrance_label_func=None):
    """Generate stage-2 physical network (a, b, c, d) by floor/room indices.

    Strategy:
    - Each floor `f` has an entrance node named `"{f}.0"` and rooms named `"{f}.{r}"` for r=1..R.
    - Inter-floor connections exist only between entrances: an undirected edge between `"{f}.0"` and `"{g}.0"` with weight `k * abs(f-g)`.
    - Intra-floor structure is a simple chain: `f.0 - f.1 - f.2 - ... - f.R` with each edge weight `intra_floor_weight`.

    Args:
        num_floors: integer number of floors (floors indexed 1..num_floors).
        rooms_per_floor: either an int (same number on each floor) or a dict {floor: R}.
        room_demands: optional dict mapping node name -> demand (e.g. {'2.1':5}). Missing entries default to 0.
        k: cost multiplier between floors (transfer cost per floor difference).
        intra_floor_weight: cost for adjacent nodes within a floor.
        entrance_label_func: optional function floor->label (defaults to str(floor)+'.0').

    Returns:
        a: list of undirected edges (u,v) (one direction listed)
        b: list of node labels
        c: dict mapping (u,v)->weight (both directions added)
        d: dict mapping node label -> demand (only non-zero entries included)
    """
    # normalize rooms_per_floor
    if isinstance(rooms_per_floor, int):
        rooms_map = {f: rooms_per_floor for f in range(1, num_floors + 1)}
    else:
        rooms_map = dict(rooms_per_floor)

    if entrance_label_func is None:
        def entrance_label_func(f):
            return f"{f}.0"

    a = []
    b = []
    c = {}
    d = {}

    # create nodes per floor
    for f in range(1, num_floors + 1):
        R = rooms_map.get(f, 0)
        entrance = entrance_label_func(f)
        b.append(entrance)
        # create rooms
        for r in range(1, R + 1):
            label = f"{f}.{r}"
            b.append(label)
            # intra-floor chain edges
            if r == 1:
                a.append((entrance, label))
            else:
                prev = f"{f}.{r-1}"
                a.append((prev, label))
            # add weights (both directions) later

    # inter-floor edges between entrances only
    entrances = [entrance_label_func(f) for f in range(1, num_floors + 1)]
    for i, ei in enumerate(entrances):
        for j, ej in enumerate(entrances):
            if i < j:
                weight = k * abs((i+1) - (j+1))
                a.append((ei, ej))

    # build weight dict c (add both directions)
    for (u, v) in a:
        if u == v:
            continue
        # determine if this edge is inter-floor (entrance-to-entrance) or intra-floor
        try:
            # parse floor numbers if possible
            fu = int(str(u).split('.')[0])
            fv = int(str(v).split('.')[0])
            if str(u).endswith('.0') and str(v).endswith('.0') and fu != fv:
                w = k * abs(fu - fv)
            else:
                w = intra_floor_weight
        except Exception:
            w = intra_floor_weight
        c[(u, v)] = w
        c[(v, u)] = w

    # demands
    if room_demands is None:
        room_demands = {}
    for node in b:
        val = room_demands.get(node, 0)
        if val:
            d[node] = val

    return a, b, c, d


def generate_physical_network_for_demands(room_demands, rooms_per_floor, k=10,con_cost=20, intra_floor_weight=1, include_floor1=True, close_loop=False):
    """Generate `a, b, c, d` based only on floors that appear in `room_demands`.

    Behavior:
    - Always include floor 1 entrance (`1.0`) if `include_floor1` is True.
    - For every floor that appears in `room_demands` (parsed from keys like '2.1'), include the entire floor layout
      (rooms `f.1..f.R`) where `R` is `rooms_per_floor` (int or dict/list per floor).
    - Intra-floor layout matches `generate_physical_network_by_index` (chains attached to `f.0`).
    - Inter-floor edges connect only entrances `f.0 <-> g.0` with weight `k * |f-g|`.

    Args:
        room_demands: dict of node -> demand, e.g. {'2.1':5, '3.1':1}
        rooms_per_floor: int (same layout every floor) or dict mapping floor->int or floor->list of chain lengths
        k: transfer cost multiplier between floors
        intra_floor_weight: cost for adjacent intra-floor edges
        include_floor1: whether to include floor 1 entrance even if it has no demand
        close_loop: if True, connect the last room of each chain back to the floor entrance (x.N -> x.0)

    Returns:
        a, b, c, d  (edges list, node list, weight dict, demands dict)
    """
    # determine floors to include from room_demands
    # Handle both 2-level (floor.room) and 3-level (building.floor.room) formats
    floors = set()
    for node in room_demands:
        try:
            parts = str(node).split('.')
            if len(parts) == 3:
                # building.floor.room format -> extract floor
                f = int(parts[1])
            else:
                # floor.room format -> extract floor
                f = int(parts[0])
            floors.add(f)
        except Exception:
            continue
    if include_floor1:
        floors.add(1)
    floors = sorted(floors)

    # normalize rooms_per_floor into a dict for included floors
    rooms_map = {}
    if isinstance(rooms_per_floor, int):
        for f in floors:
            rooms_map[f] = rooms_per_floor
    else:
        # assume dict-like or mapping; use same spec per floor if missing
        try:
            rp_dict = dict(rooms_per_floor)
        except Exception:
            rp_dict = {}
        for f in floors:
            rooms_map[f] = rp_dict.get(f, rp_dict.get('default', None) or rooms_per_floor.get(f) if isinstance(rooms_per_floor, dict) else rooms_per_floor)

    a = []
    b = []
    c = {}
    d = {}

    # build floors
    last_rooms_per_floor = {}  # track last room in each chain for closing loops
    for f in floors:
        entrance = f"{f}.0"
        b.append(entrance)
        room_spec = rooms_map.get(f, 0)
        if isinstance(room_spec, int):
            chains = [room_spec]
        else:
            chains = list(room_spec)
        room_counter = 1
        chain_last_rooms = []
        for chain_len in chains:
            prev = entrance
            for _ in range(chain_len):
                label = f"{f}.{room_counter}"
                b.append(label)
                a.append((prev, label))
                prev = label
                room_counter += 1
            # track last room of this chain
            if chain_len > 0:
                chain_last_rooms.append(prev)
        last_rooms_per_floor[f] = chain_last_rooms

    # close loops if requested
    if close_loop:
        for f in floors:
            entrance = f"{f}.0"
            for last_room in last_rooms_per_floor.get(f, []):
                a.append((last_room, entrance))


    # inter-floor edges between included entrances only
    entrances = [f"{f}.0" for f in floors]
    for i in range(len(entrances)):
        for j in range(i + 1, len(entrances)):
            a.append((entrances[i], entrances[j]))

    # build weight dict c (both directions)
    for (u, v) in a:
        if u == v:
            continue
        try:
            fu = int(str(u).split('.')[0])
            fv = int(str(v).split('.')[0])
            if str(u).endswith('.0') and str(v).endswith('.0') and fu != fv:
                w = k * abs(fu - fv) + con_cost
            else:
                w = intra_floor_weight
        except Exception:
            w = intra_floor_weight
        c[(u, v)] = w
        c[(v, u)] = w

    # demands: include only non-zero entries from room_demands
    for node, val in room_demands.items():
        if val:
            d[node] = val

    return a, b, c, d


def generate_networks_from_delivery_plan(delivery_plan, building_demands, rooms_per_floor, Q=4, k=1, intra_floor_weight=1, close_loop=True, separate_buildings=True, building_room_demands=None):
    """Generate stage-2 physical networks (a,b,c,d) for each delivery route.

    For each route in the delivery plan (e.g., [0,1,0]), creates physical networks
    for buildings in that route. If separate_buildings=True, creates one network per building.
    Properly allocates demand per delivery based on robot capacity Q.

    Args:
        delivery_plan: list of routes, e.g., [[0,1,0], [0,1,0], [0,2,4,0], [0,3,0]]
        building_demands: dict mapping building_id -> demand, e.g., {0:0, 1:8, 2:2, 3:4, 4:2}
        rooms_per_floor: int or dict specifying room layout per floor
        Q: robot capacity (max demand per delivery)
        k: transfer cost multiplier between floors
        intra_floor_weight: cost for intra-floor edges
        close_loop: whether to close loops on each floor
        separate_buildings: if True, create separate networks for each building in a route
        building_room_demands: optional dict mapping building_id -> {room: demand}, e.g., {1: {'1.2.3': 2, '1.5.1': 3}}

    Returns:
        List of (route_idx, route, building_id, a, b, c, d) tuples
        If separate_buildings=False, building_id will be a list of all buildings in route
    """
    results = []
    
    # Track remaining demand per building across all deliveries
    remaining_demands = dict(building_demands)
    
    # Track remaining room demands if provided
    if building_room_demands:
        remaining_room_demands = {bid: dict(rd) for bid, rd in building_room_demands.items()}
    else:
        remaining_room_demands = {}
    
    for route_idx, route in enumerate(delivery_plan):
        # Extract buildings visited in this route (excluding depot 0)
        buildings_in_route = [bldg for bldg in route if bldg != 0]
        
        if not buildings_in_route:
            # Empty route, skip
            continue
        
        if separate_buildings:
            # Create separate network for each building
            for building_id in buildings_in_route:  # keep order, may have duplicates
                remaining = remaining_demands.get(building_id, 0)
                if remaining <= 0:
                    continue
                
                # This delivery carries min(Q, remaining_demand)
                delivery_amount = min(Q, remaining)
                remaining_demands[building_id] = remaining - delivery_amount
                
                # Generate room_demands for this delivery
                if building_room_demands and building_id in remaining_room_demands:
                    # Use detailed room demands and allocate up to delivery_amount
                    room_demands = {}
                    allocated = 0
                    room_list = list(remaining_room_demands[building_id].items())
                    
                    for room, demand in room_list:
                        if allocated >= delivery_amount:
                            break
                        can_deliver = min(demand, delivery_amount - allocated)
                        if can_deliver > 0:
                            room_demands[room] = can_deliver
                            remaining_room_demands[building_id][room] -= can_deliver
                            if remaining_room_demands[building_id][room] <= 0:
                                del remaining_room_demands[building_id][room]
                            allocated += can_deliver
                else:
                    # Fallback: simple single-room allocation
                    room_demands = {f"{building_id}.1": delivery_amount}
                
                # Convert building.floor.room format to floor.room format for network generation
                # (since each building is optimized separately)
                room_demands_converted = {}
                for room, demand in room_demands.items():
                    parts = str(room).split('.')
                    if len(parts) == 3:
                        # building.floor.room -> floor.room
                        floor_room = f"{parts[1]}.{parts[2]}"
                        room_demands_converted[floor_room] = demand
                    else:
                        room_demands_converted[room] = demand
                
                # Generate physical network for this building only
                a, b, c, d = generate_physical_network_for_demands(
                    room_demands=room_demands_converted,
                    rooms_per_floor=rooms_per_floor,
                    k=k,
                    intra_floor_weight=intra_floor_weight,
                    include_floor1=True,  # always include entrance floor
                    close_loop=close_loop
                )
                
                results.append((route_idx, route, building_id, a, b, c, d))
        else:
            # Create combined network for all buildings in route (original behavior)
            room_demands = {}
            for building_id in set(buildings_in_route):
                demand = building_demands.get(building_id, 0)
                if demand > 0:
                    room_demands[f"{building_id}.1"] = demand
            
            # Generate physical network for these floors
            a, b, c, d = generate_physical_network_for_demands(
                room_demands=room_demands,
                rooms_per_floor=rooms_per_floor,
                k=k,
                intra_floor_weight=intra_floor_weight,
                include_floor1=True,
                close_loop=close_loop
            )
            
            results.append((route_idx, route, buildings_in_route, a, b, c, d))
    
    return results


if __name__ == '__main__':
    # quick demo: create 6 nodes (0 is depot) on a grid and sample demands
    coords = {
        0: (0.0, 0.0),
        1: (2.0, 1.0),
        2: (4.0, 0.0),
        3: (3.0, 3.0),
        4: (1.0, 4.0),
        5: (5.0, 4.0),
    }
    demands = {0: 0, 1: 6, 2: 2, 3: 4, 4: 3, 5: 1}
    Bc, Cc, Dc = build_stage1_from_coords(coords, demands)
    print('B =', Bc)
    print('C sample (0->1) =', Cc[(0,1)])
    print('D =', Dc)
