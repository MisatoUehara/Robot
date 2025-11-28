import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

# Reused global flags will be imported from the main script when calling these functions

def visualize_stage1_routes(B, C, D, ORIGINAL_CYCCLES, building_positions=None, Q=4, ENABLE_VISUALIZATION=True, SHOW_STAGE1_VIS=True, RANDOM_SEED=0):
    """Visualize stage 1 building delivery routes."""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE1_VIS:
        return

    if building_positions is None:
        G = nx.Graph()
        G.add_nodes_from(B)
        for i in B:
            for j in B:
                if i < j and i in C and (i, j) in C:
                    G.add_edge(i, j, weight=C[(i, j)])
        building_positions = nx.spring_layout(G, k=2, iterations=50, seed=RANDOM_SEED)

    colors = plt.cm.tab20(np.linspace(0, 1, len(ORIGINAL_CYCCLES)))
    plt.figure(figsize=(14, 10))

    depot_pos = building_positions[0]
    plt.scatter([depot_pos[0]], [depot_pos[1]], s=800, c='red', marker='s', 
                label='Depot', zorder=5, edgecolors='black', linewidths=2)

    for building in B[1:]:
        pos = building_positions[building]
        plt.scatter([pos[0]], [pos[1]], s=500, c='lightblue', marker='o', 
                    zorder=4, edgecolors='black', linewidths=1.5)
        plt.text(pos[0], pos[1], f'{building}\n({D[building]})', 
                 ha='center', va='center', fontsize=10, fontweight='bold')

    for idx, (route, color) in enumerate(zip(ORIGINAL_CYCCLES, colors)):
        for i in range(len(route) - 1):
            start, end = route[i], route[i+1]
            start_pos = building_positions[start]
            end_pos = building_positions[end]
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


def visualize_stage2_routes(building_id, b, a, c, d, physical_paths, original_cycles, cycle_demands, Q=4, ENABLE_VISUALIZATION=True, SHOW_STAGE2_VIS=True):
    """Visualize stage 2 room-level routes for a building."""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE2_VIS:
        return

    G = nx.Graph()
    G.add_nodes_from(b)
    G.add_edges_from(a)
    for (i, j) in a:
        G[i][j]['weight'] = c[(i, j)]

    pos = {}
    for node in b:
        if node == '1.0':
            pos[node] = (0, 0)
        else:
            parts = node.split('.')
            floor = int(parts[0])
            room = int(parts[1])
            if room == 0:
                pos[node] = (0, floor * 2)
            else:
                pos[node] = (room * 1.5, floor * 2)

    fig, ax = plt.subplots(figsize=(12, 10))
    for (u, v) in a:
        x = [pos[u][0], pos[v][0]]
        y = [pos[u][1], pos[v][1]]
        ax.plot(x, y, 'gray', alpha=0.3, linewidth=1, zorder=1)

    colors = plt.cm.Set3(np.linspace(0, 1, len(physical_paths)))

    for idx, (path, color, demand) in enumerate(zip(physical_paths, colors, cycle_demands)):
        for i in range(len(path) - 1):
            start, end = path[i], path[i+1]
            x = [pos[start][0], pos[end][0]]
            y = [pos[start][1], pos[end][1]]
            ax.plot(x, y, color=color, linewidth=3, alpha=0.7, 
                    label=f'Trip {idx+1} (Load: {demand})' if i == 0 else '', zorder=2)
            if i < len(path) - 2:
                mid_x = (x[0] + x[1]) / 2
                mid_y = (y[0] + y[1]) / 2
                dx = x[1] - x[0]
                dy = y[1] - y[0]
                ax.annotate('', xy=(mid_x + dx*0.1, mid_y + dy*0.1), 
                            xytext=(mid_x - dx*0.1, mid_y - dy*0.1),
                            arrowprops=dict(arrowstyle='->', color=color, lw=2), zorder=3)

    for node in b:
        x, y = pos[node]
        if node == '1.0':
            ax.scatter(x, y, s=500, c='red', marker='s', zorder=5, edgecolors='black', linewidths=2)
            ax.text(x, y-0.3, 'Entrance', ha='center', fontsize=9, fontweight='bold')
        elif node.endswith('.0'):
            ax.scatter(x, y, s=300, c='orange', marker='d', zorder=4, edgecolors='black', linewidths=1.5)
            ax.text(x-0.3, y, f'F{node.split(".")[0]}', ha='right', fontsize=8)
        else:
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
