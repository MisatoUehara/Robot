# Visualization Guide for CVRP_2index_multi_demand_whole_op.py

## Overview
The script now includes automatic visualization for both Stage 1 (building-level delivery) and Stage 2 (room-level delivery) routing solutions.

## Configuration Parameters

At the top of the script, you can control visualization behavior:

```python
# Visualization Parameters
ENABLE_VISUALIZATION = True     # Enable/disable all visualizations
SHOW_STAGE1_VIS = True          # Show stage 1 (building routes) visualization
SHOW_STAGE2_VIS = True          # Show stage 2 (room routes) visualization
MAX_STAGE2_BUILDINGS_VIS = 3    # Limit number of buildings to visualize in stage 2
```

## Generated Visualizations

### Stage 1: Building Delivery Routes
**File:** `stage1_building_routes.png`

**Features:**
- **Red Square**: Depot (starting point)
- **Blue Circles**: Buildings with demand numbers
- **Colored Arrows**: Different delivery routes
- Each arrow shows a delivery trip from depot to building(s) and back

**Reading the visualization:**
- Node labels show building ID and demand in parentheses (e.g., "3 (14)" means building 3 has 14 packages)
- Multiple colored arrows indicate multiple delivery trips
- All routes respect the capacity constraint Q

### Stage 2: Building Room Delivery Routes
**Files:** `stage2_building{N}_routes.png` (one per building)

**Features:**
- **Red Square**: Building entrance (floor 1.0)
- **Orange Diamonds**: Floor entrances (e.g., F1, F2, F3...)
- **Green Circles**: Rooms with demand (label shows room ID and demand in brackets)
- **Gray Circles**: Rooms without demand
- **Colored Lines**: Different delivery trips

**Layout:**
- Vertical axis: Floor levels (higher = upper floors)
- Horizontal axis: Room positions on each floor
- Gray lines: Physical network structure (possible paths)
- Colored lines: Actual delivery routes

**Reading the visualization:**
- Room labels like "2.3 [2]" mean floor 2, room 3, with 2 packages
- Each colored route represents one robot trip (≤ Q capacity)
- Arrows show the direction of movement
- The legend shows trip number and load for each trip

## Usage Examples

### 1. Run with all visualizations enabled (default)
```python
ENABLE_VISUALIZATION = True
SHOW_STAGE1_VIS = True
SHOW_STAGE2_VIS = True
MAX_STAGE2_BUILDINGS_VIS = 3
```

### 2. Only show Stage 1 visualization
```python
ENABLE_VISUALIZATION = True
SHOW_STAGE1_VIS = True
SHOW_STAGE2_VIS = False
```

### 3. Disable all visualizations (faster execution)
```python
ENABLE_VISUALIZATION = False
```

### 4. Show more buildings in Stage 2
```python
MAX_STAGE2_BUILDINGS_VIS = 5  # Visualize up to 5 buildings
```

## Tips

1. **Performance**: Visualizing many buildings in Stage 2 can be slow. Use `MAX_STAGE2_BUILDINGS_VIS` to limit the number.

2. **File Location**: All PNG files are saved in the same directory as the script.

3. **Rerunning**: Each run overwrites previous visualization files with the same names.

4. **Stage 2 Selection**: The script visualizes the first N buildings (where N = `MAX_STAGE2_BUILDINGS_VIS`) that have demand.

5. **Customization**: You can modify the visualization functions (`visualize_stage1_routes` and `visualize_stage2_routes`) to change:
   - Colors
   - Node sizes
   - Font sizes
   - Figure dimensions
   - Arrow styles

## Output Files

After running the script, you will find:
- `stage1_building_routes.png` - Building-level delivery routes
- `stage2_building1_routes.png` - Room-level routes for building 1
- `stage2_building2_routes.png` - Room-level routes for building 2
- `stage2_building3_routes.png` - Room-level routes for building 3
- (and so on, up to `MAX_STAGE2_BUILDINGS_VIS`)

## Interpreting Results

### Stage 1 Verification
- Check that all buildings are visited
- Verify routes start and end at depot (red square)
- Confirm no route exceeds capacity Q

### Stage 2 Verification
- Each colored route should have load ≤ Q (shown in legend)
- All rooms with demand should be visited
- Routes should start and end at building entrance (1.0)
- Path follows the physical network structure (gray lines)

## Troubleshooting

**Problem**: Visualizations don't appear
- **Solution**: Make sure `ENABLE_VISUALIZATION = True` and matplotlib is installed

**Problem**: Too many Stage 2 plots
- **Solution**: Reduce `MAX_STAGE2_BUILDINGS_VIS` to a smaller number

**Problem**: Stage 2 plots are cluttered
- **Solution**: Reduce `NUM_FLOORS` or `ROOMS_PER_FLOOR` in the configuration

**Problem**: Can't see all routes clearly
- **Solution**: Increase figure size in the visualization functions (figsize parameter)
