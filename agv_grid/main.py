import turtle
import random

from warehouse import Warehouse
from renderer import Render
from agv import AGV
from path import Grassfire

NUM_OF_AGV = 15
ROWS, COLUMNS = 25, 25
SHELVES = 50

CELL_SIZE = 30
FRAMES = 5
STEPS = 10

def simulation_step():
    active = False

    for _agv in agv:
        if  _agv.target is None and _agv.path:
            next_target = _agv.path[0]

            if is_cell_free(_agv, next_target, agv, warehouse_map):
                _agv.start_next_step()

        if _agv.target:
            _agv.update()
            renderer.move(_agv)
            active = True

    renderer.screen.update()

    if active:
        renderer.screen.ontimer(simulation_step, FRAMES)


def is_cell_free(current_agv, target, agvs, _map):
    r,c = target

    if _map[r][c] == 1:
        return False

    for other in agvs:
        if other.id == current_agv.id:
            continue

        if other.target == target:
            return False

        if (other.row, other.column) == target:
            return False

    return True

warehouse = Warehouse(rows=ROWS, columns=COLUMNS, shelves=SHELVES)
warehouse_map = warehouse.random_map()
planner = Grassfire(warehouse_map)

renderer = Render(warehouse_map, cell_size = CELL_SIZE)
renderer.map()


agv = []
path = []
start = []
goal = []

#set starting point and goal for all agv
for i in range(NUM_OF_AGV):
    while True:
        sr = random.randint(0, 10)
        sc = random.randint(0, COLUMNS - 1)

        gr = random.randint(ROWS - 10, COLUMNS - 1)
        gc = random.randint(0, COLUMNS - 1)

        # must not be shelves
        if warehouse_map[sr][sc] == 1:
            continue
        if warehouse_map[gr][gc] == 1:
            continue

        # prevent overlapping starts
        if (sr, sc) in start:
            continue

        candidate_path = planner.compute((sr, sc), (gr, gc))

        if candidate_path is not None:
            start.append((sr, sc))
            goal.append((gr, gc))
            path.append(candidate_path)
            break
    agv.append(AGV(agv_id=i, row=start[i][0], column=start[i][1], frames=FRAMES))
    renderer.add_agv(agv[i])

for i in range(NUM_OF_AGV):
    if path[i] is not None:
        renderer.draw_path(path[i])
        agv[i].set_path(path[i][1:])

simulation_step()
turtle.done()
