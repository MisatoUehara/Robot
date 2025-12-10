import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl

# Reused global flags will be imported from the main script when calling these functions

# Set global font to Times New Roman for all text elements
# Prefer Times New Roman; fallback to common serif fonts if unavailable
mpl.rcParams['font.family'] = ['Times New Roman', 'DejaVu Serif', 'Liberation Serif', 'serif']

def visualize_stage1_routes(
    B, C, D, ORIGINAL_CYCCLES,
    building_positions=None,
    Q=4,
    ENABLE_VISUALIZATION=True,
    SHOW_STAGE1_VIS=True,
    SHOW_STEP_NUMBERS=False,
    SHOW_DEMAND_NUMBERS=True,
    RANDOM_SEED=0,
    node_size_scale=60,
    show=True,
    save_path='stage1_building_routes.png'
):
    """Visualize stage 1 building delivery routes with clearer labels and demand-sized nodes."""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE1_VIS:
        return

    if building_positions is None:
        G = nx.Graph()
        G.add_nodes_from(B)
        for i in B:
            for j in B:
                if i < j and (i, j) in C:
                    G.add_edge(i, j, weight=C[(i, j)])
        building_positions = nx.spring_layout(G, k=2, iterations=100, seed=RANDOM_SEED)

    colors = plt.cm.tab20(np.linspace(0, 1, len(ORIGINAL_CYCCLES)))
    plt.figure(figsize=(14, 10))

    depot_pos = building_positions[0]
    plt.scatter([depot_pos[0]], [depot_pos[1]], s=900, c='#d62728', marker='s', 
                label='Depot', zorder=5, edgecolors='black', linewidths=2)
    plt.text(depot_pos[0], depot_pos[1]-0.05, 'Depot (0)',
             ha='center', va='top', fontsize=11, fontweight='bold')

    for building in B[1:]:
        pos = building_positions[building]
        demand = D.get(building, 0)
        size = max(300, demand * node_size_scale)
        plt.scatter([pos[0]], [pos[1]], s=size, c='#1f77b4', marker='o', 
                    zorder=4, edgecolors='white', linewidths=1.5)
        plt.text(pos[0], pos[1], f'{building}', 
                 ha='center', va='center', fontsize=10, fontweight='bold', color='white')
        if SHOW_DEMAND_NUMBERS and demand > 0:
            # Use offset in points so label does not get covered by the marker
            from matplotlib import pyplot as _plt
            _ax = _plt.gca()
            _ax.annotate(
                f'{demand}',
                xy=(pos[0], pos[1]),
                xytext=(10, 0),  # 10pt to the right of center
                textcoords='offset points',
                ha='left', va='center', fontsize=10, fontweight='bold', color='darkgreen',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='darkgreen', alpha=0.85),
                zorder=6
            )

    for idx, (route, color) in enumerate(zip(ORIGINAL_CYCCLES, colors)):
        # Use curved arrows to avoid overlap; vary curvature per route
        curvature = 0.1 + 0.03 * idx
        for step in range(len(route) - 1):
            start, end = route[step], route[step+1]
            start_pos = building_positions[start]
            end_pos = building_positions[end]
            try:
                from matplotlib.patches import FancyArrowPatch
                arrow = FancyArrowPatch(
                    posA=start_pos, posB=end_pos,
                    connectionstyle=f"arc3,rad={curvature}",
                    arrowstyle='-|>', mutation_scale=15,
                    lw=2, color=color, alpha=0.75, zorder=3
                )
                plt.gca().add_patch(arrow)
            except Exception:
                dx = end_pos[0] - start_pos[0]
                dy = end_pos[1] - start_pos[1]
                plt.arrow(start_pos[0], start_pos[1], dx*0.85, dy*0.85,
                          head_width=0.03, head_length=0.02, fc=color, ec=color,
                          alpha=0.65, length_includes_head=True, zorder=3)
            mid_x = (start_pos[0] + end_pos[0]) / 2
            mid_y = (start_pos[1] + end_pos[1]) / 2
            if SHOW_STEP_NUMBERS:
                plt.text(mid_x, mid_y, str(step+1), fontsize=9, fontweight='bold', color='black',
                         bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.7))

    plt.title(f'Stage 1: Building Delivery Routes (Q={Q}, Routes: {len(ORIGINAL_CYCCLES)})', 
              fontsize=14, fontweight='bold')
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.grid(False)
    # hide axes/ticks/coordinate frame
    ax_stage1 = plt.gca()
    ax_stage1.set_xticks([])
    ax_stage1.set_yticks([])
    for spine in ax_stage1.spines.values():
        spine.set_visible(False)
    plt.axis('off')
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  → Stage 1 visualization saved as '{save_path}'")
    if show:
        plt.show()


def visualize_stage2_routes(
    building_id, b, a, c, d, physical_paths, original_cycles, cycle_demands,
    Q=4, ENABLE_VISUALIZATION=True, SHOW_STAGE2_VIS=True,
    SHOW_STEP_NUMBERS=False,
    SHOW_DEMAND_NUMBERS=True,
    node_size_base=200, node_size_scale=80,
    show=True, save_path=None,
    split_subplots=False
):
    """Visualize stage 2 room-level routes for a building with clearer layout and labels."""
    if not ENABLE_VISUALIZATION or not SHOW_STAGE2_VIS:
        return

    G = nx.Graph()
    G.add_nodes_from(b)
    G.add_edges_from(a)
    for (i, j) in a:
        G[i][j]['weight'] = c[(i, j)]

    # Build positions for all nodes that appear either in b or a (edges)
    pos = {}
    nodes_for_layout = set(b)
    for (u, v) in a:
        nodes_for_layout.add(u)
        nodes_for_layout.add(v)
    for node in nodes_for_layout:
        parts = node.split('.')
        floor = int(parts[0])
        room = int(parts[1])
        y = floor * 2
        x = 0 if room == 0 else room * 1.6
        pos[node] = (x, y)

    fig, ax = plt.subplots(figsize=(13, 10))
    # physical edges; emphasize entrance-room connectors and inter-floor connectors
    for (u, v) in a:
        x = [pos[u][0], pos[v][0]]
        y = [pos[u][1], pos[v][1]]
        is_entrance_room = (u.endswith('.0') and not v.endswith('.0')) or (v.endswith('.0') and not u.endswith('.0'))
        is_inter_entrance = (u.endswith('.0') and v.endswith('.0'))
        if is_entrance_room:
            ax.plot(x, y, color='#9467bd', alpha=0.6, linewidth=2.2, zorder=2, linestyle='--')
        elif is_inter_entrance:
            ax.plot(x, y, color='#7f7f7f', alpha=0.4, linewidth=1.4, zorder=1)
        else:
            ax.plot(x, y, color='#b0b0b0', alpha=0.35, linewidth=1.0, zorder=1)

    colors = plt.cm.Dark2(np.linspace(0, 1, len(physical_paths)))

    if split_subplots and len(physical_paths) > 1:
        # draw separate subplots per trip to ensure visibility
        fig2, axes = plt.subplots(len(physical_paths), 1, figsize=(13, 3.5*len(physical_paths)), sharex=True)
        if not isinstance(axes, np.ndarray):
            axes = np.array([axes])
        for idx, (path, demand) in enumerate(zip(physical_paths, cycle_demands)):
            ax_i = axes[idx]
            ax_i.set_title(f'Trip {idx+1} (Load: {demand})')
            # draw physical edges lightly
            for (u, v) in a:
                x = [pos[u][0], pos[v][0]]
                y = [pos[u][1], pos[v][1]]
                ax_i.plot(x, y, color='#b0b0b0', alpha=0.25, linewidth=1, zorder=1)
            # draw route
            for i in range(len(path) - 1):
                start, end = path[i], path[i+1]
                x = [pos[start][0], pos[end][0]]
                y = [pos[start][1], pos[end][1]]
                ax_i.plot(x, y, color=colors[idx], linewidth=3, alpha=0.9, zorder=3)
                mid_x = (x[0] + x[1]) / 2
                mid_y = (y[0] + y[1]) / 2
                if SHOW_STEP_NUMBERS:
                    ax_i.annotate(str(i+1), xy=(mid_x, mid_y), xytext=(mid_x, mid_y),
                                  textcoords='data', ha='center', va='center', fontsize=9, color='white',
                                  bbox=dict(boxstyle='circle,pad=0.2', fc=colors[idx], ec='white', alpha=0.9))
            # draw nodes (original coloring by demand)
            for node in b:
                x, y = pos[node]
                if node.endswith('.0'):
                    ax_i.scatter(x, y, s=300, c='#ff7f0e', marker='d', zorder=5)
                else:
                    demand_n = d.get(node, 0)
                    size = node_size_base + demand_n * node_size_scale if demand_n > 0 else node_size_base
                    face = '#2ca02c' if demand_n > 0 else '#c7c7c7'
                    ax_i.scatter(x, y, s=size, c=face, marker='o', zorder=4)
            ax_i.grid(False)
            # hide axes/ticks for per-trip subplot
            ax_i.set_xticks([])
            ax_i.set_yticks([])
            for spine in ax_i.spines.values():
                spine.set_visible(False)
            ax_i.axis('off')
        fig2.tight_layout()

    # routes in combined view; use curvature (not vertical offset) to reduce overlap
    for idx, (path, color, demand) in enumerate(zip(physical_paths, colors, cycle_demands)):
        # no y-offset; keep endpoints aligned to node centers
        for i in range(len(path) - 1):
            start, end = path[i], path[i+1]
            x = [pos[start][0], pos[end][0]]
            y = [pos[start][1], pos[end][1]]
            # draw as curved arrow to avoid overlap; curvature varies by route index
            try:
                from matplotlib.patches import FancyArrowPatch
                curvature = 0.12 * (idx - (len(physical_paths)-1)/2)
                # underlay glow
                glow = FancyArrowPatch(
                    posA=(x[0], y[0]), posB=(x[1], y[1]),
                    connectionstyle=f"arc3,rad={curvature}",
                    arrowstyle='-'
                    , lw=4, color=color, alpha=0.2, zorder=2
                )
                ax.add_patch(glow)
                # main curved arrow with head
                arrow = FancyArrowPatch(
                    posA=(x[0], y[0]), posB=(x[1], y[1]),
                    connectionstyle=f"arc3,rad={curvature}",
                    arrowstyle='-|>', mutation_scale=13,
                    lw=2.2, color=color, alpha=0.95,
                    zorder=3
                )
                ax.add_patch(arrow)
                if i == 0:
                    ax.plot([], [], color=color, linewidth=2.2, alpha=0.95,
                            label=f'Trip {idx+1} (Load: {demand})')
            except Exception:
                # fallback straight line if patches fail
                ax.plot(x, y, color=color, linewidth=2.2, alpha=0.9,
                        label=f'Trip {idx+1} (Load: {demand})' if i == 0 else '', zorder=3)
            # Highlight closing loop edges x.N -> x.0 with a directional arrow overlay
            if (not start.endswith('.0')) and end.endswith('.0'):
                try:
                    from matplotlib.patches import FancyArrowPatch
                    curvature_close = 0.12 * (idx - (len(physical_paths)-1)/2)
                    arrow = FancyArrowPatch(
                        posA=(pos[start][0], pos[start][1]),
                        posB=(pos[end][0], pos[end][1]),
                        connectionstyle=f"arc3,rad={curvature_close}",
                        arrowstyle='-|>', mutation_scale=16,
                        lw=0, color=color, alpha=0.95, zorder=4
                    )
                    ax.add_patch(arrow)
                except Exception:
                    pass
            mid_x = (x[0] + x[1]) / 2
            mid_y = (y[0] + y[1]) / 2
            if SHOW_STEP_NUMBERS:
                ax.annotate(str(i+1), xy=(mid_x, mid_y), xytext=(mid_x, mid_y),
                            textcoords='data', ha='center', va='center',
                            fontsize=9, color='white',
                            bbox=dict(boxstyle='circle,pad=0.2', fc=color, ec='white', alpha=0.9))
        # no per-trip node overlay in combined view (reverted)

    # nodes
    for node in b:
        x, y = pos[node]
        if node.endswith('.0'):
            ax.scatter(x, y, s=500, c='#ff7f0e', marker='d', zorder=5, edgecolors='black', linewidths=1.5)
            ax.text(x, y-0.3, f'Entrance F{node.split(".")[0]}', ha='center', fontsize=9, fontweight='bold')
        else:
            demand = d.get(node, 0)
            size = node_size_base + demand * node_size_scale if demand > 0 else node_size_base
            face = '#2ca02c' if demand > 0 else '#c7c7c7'
            edge = 'darkgreen' if demand > 0 else '#888888'
            ax.scatter(x, y, s=size, c=face, marker='o', zorder=4, edgecolors=edge, linewidths=1.6)
            label = node
            ax.text(x, y, label, ha='center', va='center', fontsize=8, fontweight='bold', color='white' if demand>0 else 'black')
            if SHOW_DEMAND_NUMBERS and demand > 0:
                # show demand number beside the node (offset to the right)
                ax.text(x + 0.35, y, f'{demand}', ha='left', va='center', fontsize=9,
                        fontweight='bold', color='darkgreen',
                        bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='darkgreen', alpha=0.8))

    ax.set_title(f'Building {building_id} Room Delivery Routes\nTotal Demand: {sum(d.values())}, Trips: {len(physical_paths)}, Q={Q}',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(False)
    # hide axes/ticks/coordinate frame for combined view
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.axis('equal')
    ax.axis('off')
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  → Building {building_id} visualization saved as '{save_path}'")
    if show:
        plt.show()
