"""
Headless Python Test for Maze Explorer Algorithm
------------------------------------------------
Equivalent to the native C++ test and browser simulation.
Runs on any standard Python 3 installation without external packages.
"""

# Directions: 0: North, 1: East, 2: South, 3: West
DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3
DIR_DX = [0, 1, 0, -1]
DIR_DY = [1, 0, -1, 0]
DIR_NAMES = ['NORTH', 'EAST', 'SOUTH', 'WEST']

# Bitmasks
WALL_NORTH = 1 << 0  # 0x01
WALL_EAST  = 1 << 1  # 0x02
WALL_SOUTH = 1 << 2  # 0x04
WALL_WEST  = 1 << 3  # 0x08
CELL_VISITED = 1 << 4 # 0x10
WALL_MASKS = [WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST]

MAZE_A_SIZE = 4
MAZE_B_SIZE = 9
GRID_A_WORK_SIZE = 7

TILE_NORMAL_BLACK = 0
TILE_WHITE = 1
TILE_BRIDGE_MARKER = 2

def generate_maze(width, height, seed=12345):
    maze = [[WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST for _ in range(width)] for _ in range(height)]
    visited = [[False for _ in range(width)] for _ in range(height)]
    stack = [(0, 0)]
    visited[0][0] = True

    s = seed
    def rng():
        nonlocal s
        s = (s * 1103515245 + 12345) & 0x7FFFFFFF
        return s

    while stack:
        cx, cy = stack[-1]
        valid = []
        for d in range(4):
            nx, ny = cx + DIR_DX[d], cy + DIR_DY[d]
            if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                valid.append(d)
        if valid:
            d = valid[rng() % len(valid)]
            nx, ny = cx + DIR_DX[d], cy + DIR_DY[d]
            maze[cy][cx] &= ~WALL_MASKS[d]
            maze[ny][nx] &= ~WALL_MASKS[(d + 2) % 4]
            visited[ny][nx] = True
            stack.append((nx, ny))
        else:
            stack.pop()
    return maze

def test_full_exploration(goal_x, goal_y):
    truth_a = [[0 for _ in range(MAZE_A_SIZE)] for _ in range(MAZE_A_SIZE)]
    for y in range(MAZE_A_SIZE):
        for x in range(MAZE_A_SIZE):
            if y == MAZE_A_SIZE - 1: truth_a[y][x] |= WALL_NORTH
            if y == 0: truth_a[y][x] |= WALL_SOUTH
            if x == MAZE_A_SIZE - 1: truth_a[y][x] |= WALL_EAST
            if x == 0: truth_a[y][x] |= WALL_WEST
    truth_a[3][3] &= ~WALL_NORTH # open bridge ramp

    truth_b = generate_maze(MAZE_B_SIZE, MAZE_B_SIZE, 12345)
    truth_b[0][0] &= ~WALL_SOUTH # bridge entry

    # Robot internal state
    work_a = [[0 for _ in range(GRID_A_WORK_SIZE)] for _ in range(GRID_A_WORK_SIZE)]
    grid_b = [[0 for _ in range(MAZE_B_SIZE)] for _ in range(MAZE_B_SIZE)]

    curr_a = [3, 3]
    curr_b = [0, 0]
    heading = DIR_NORTH
    pos_x, pos_y = 0, 0
    in_section = 'A'
    dfs_stack = []
    steps = 0
    state = "EXPLORE_A"

    def read_walls():
        grid = truth_a if in_section == 'A' else truth_b
        cw = grid[pos_y][pos_x]
        f_dir = heading
        r_dir = (heading + 1) % 4
        l_dir = (heading + 3) % 4
        return {
            'front': bool(cw & WALL_MASKS[f_dir]),
            'right': bool(cw & WALL_MASKS[r_dir]),
            'left':  bool(cw & WALL_MASKS[l_dir])
        }

    def read_tile():
        if in_section == 'A':
            if pos_x == 3 and pos_y == 2: return TILE_BRIDGE_MARKER
            if pos_x == 0 and pos_y == 0: return TILE_WHITE
        else:
            if pos_x == goal_x and pos_y == goal_y: return TILE_WHITE
        return TILE_NORMAL_BLACK

    # Run loop
    while steps < 1000:
        steps += 1
        if state == "EXPLORE_A":
            # Map current
            cx, cy = curr_a
            work_a[cy][cx] |= CELL_VISITED
            ws = read_walls()
            f_dir, r_dir, l_dir = heading, (heading + 1) % 4, (heading + 3) % 4
            if ws['front']: work_a[cy][cx] |= WALL_MASKS[f_dir]
            if ws['right']: work_a[cy][cx] |= WALL_MASKS[r_dir]
            if ws['left']:  work_a[cy][cx] |= WALL_MASKS[l_dir]

            if read_tile() == TILE_BRIDGE_MARKER:
                state = "CROSS_BRIDGE"
                continue

            # DFS step
            moved = False
            for d in [heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4]:
                if not (work_a[cy][cx] & WALL_MASKS[d]):
                    nx, ny = cx + DIR_DX[d], cy + DIR_DY[d]
                    if 0 <= nx < GRID_A_WORK_SIZE and 0 <= ny < GRID_A_WORK_SIZE and not (work_a[ny][nx] & CELL_VISITED):
                        dfs_stack.append((cx, cy))
                        heading = d
                        curr_a[0], curr_a[1] = nx, ny
                        pos_x += DIR_DX[d]
                        pos_y += DIR_DY[d]
                        moved = True
                        break
            if not moved and dfs_stack:
                bx, by = dfs_stack.pop()
                for d in range(4):
                    if cx + DIR_DX[d] == bx and cy + DIR_DY[d] == by:
                        heading = d
                        curr_a[0], curr_a[1] = bx, by
                        pos_x += DIR_DX[d]
                        pos_y += DIR_DY[d]
                        break

        elif state == "CROSS_BRIDGE":
            # Move forward onto ramp, cross, enter Section B
            pos_x, pos_y = 0, 0
            curr_b[0], curr_b[1] = 0, 0
            heading = DIR_NORTH
            in_section = 'B'
            dfs_stack.clear()
            state = "EXPLORE_B"

        elif state == "EXPLORE_B":
            cx, cy = curr_b
            grid_b[cy][cx] |= CELL_VISITED
            ws = read_walls()
            f_dir, r_dir, l_dir = heading, (heading + 1) % 4, (heading + 3) % 4
            if ws['front']: grid_b[cy][cx] |= WALL_MASKS[f_dir]
            if ws['right']: grid_b[cy][cx] |= WALL_MASKS[r_dir]
            if ws['left']:  grid_b[cy][cx] |= WALL_MASKS[l_dir]

            if read_tile() == TILE_WHITE:
                return True, (cx, cy), steps

            # DFS step
            moved = False
            for d in [heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4]:
                if not (grid_b[cy][cx] & WALL_MASKS[d]):
                    nx, ny = cx + DIR_DX[d], cy + DIR_DY[d]
                    if 0 <= nx < MAZE_B_SIZE and 0 <= ny < MAZE_B_SIZE and not (grid_b[ny][nx] & CELL_VISITED):
                        dfs_stack.append((cx, cy))
                        heading = d
                        curr_b[0], curr_b[1] = nx, ny
                        pos_x += DIR_DX[d]
                        pos_y += DIR_DY[d]
                        moved = True
                        break
            if not moved:
                if not dfs_stack:
                    return False, None, steps
                bx, by = dfs_stack.pop()
                for d in range(4):
                    if cx + DIR_DX[d] == bx and cy + DIR_DY[d] == by:
                        heading = d
                        curr_b[0], curr_b[1] = bx, by
                        pos_x += DIR_DX[d]
                        pos_y += DIR_DY[d]
                        break

    return False, None, steps

if __name__ == '__main__':
    print("=== Testing MicroMouse Maze Explorer (Python Headless) ===")
    for test_goal in [(3, 0), (4, 4), (8, 8), (1, 7)]:
        found, loc, st = test_full_exploration(test_goal[0], test_goal[1])
        status = "PASSED" if (found and loc == test_goal) else "FAILED"
        print(f"Goal: {test_goal} -> Found: {found} at {loc} in {st} steps [{status}]")
