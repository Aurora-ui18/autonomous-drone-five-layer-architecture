#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2023 电赛 G 题「空地协同智能消防系统」—— 全覆盖路径规划模块

供 global_planner_node 等节点复用。

【G 题场地约定】
  巡防区域: 40dm × 48dm (4m × 4.8m)
  宏格划分: 5列(col=0..4) × 6行(row=0..5)，每格 8dm×8dm
  起飞点: 左下角区域 (1dm, 1dm) 附近
  覆盖宽度: 激光笔覆盖 8dm 宽，每扫过一个宏格中心即覆盖该格

【宏格中心坐标 (dm)】
  col=0: X=4   col=1: X=12   col=2: X=20   col=3: X=28   col=4: X=36
  row=0: Y=4   row=1: Y=12   row=2: Y=20   row=3: Y=28   row=4: Y=36   row=5: Y=44

【蛇形扫描策略】
  从起飞点飞向首个宏格(col=0,row=0)，沿Y方向蛇形覆盖5列:
    col=0: row 0→5 (Y递增)
    col=1: row 5→0 (蛇形折返)
    col=2: row 0→5
    col=3: row 5→0
    col=4: row 0→5
  总航程约 5×48 = 240dm

【对外接口】
  - plan_coverage_route_g: 生成 G 题全覆盖航点序列
  - macro_cell_to_xy_dm / xy_to_macro_cell: 已在 uav_interfaces.py 中定义
"""

try:
    import rospy
except ImportError:
    rospy = None

G_PATROL_COLS = 6       # 48dm / 8dm
G_PATROL_ROWS = 5       # 40dm / 8dm
G_CELL_SIZE_DM = 8


def plan_coverage_route_g():
    """
    G 题全覆盖蛇形路径规划。

    返回按蛇形顺序覆盖所有 30 个宏格中心的航点序列。

    Returns:
        list: [(x_dm, y_dm, z_dm, macro_cell_id), ...]
              z_dm = 14.0（巡逻高度 1.4m）
              macro_cell_id = "C{col}R{row}"
    """
    route = []
    z_dm = 14.0  # G 题巡逻高度 1.4m

    for col in range(G_PATROL_COLS):
        if col % 2 == 0:
            # 偶数列: row 递增
            row_range = range(G_PATROL_ROWS)
        else:
            # 奇数列: row 递减（蛇形折返）
            row_range = range(G_PATROL_ROWS - 1, -1, -1)

        for row in row_range:
            x_dm = col * G_CELL_SIZE_DM  # C0R0=(0,0), +8dm each col
            y_dm = row * G_CELL_SIZE_DM  # C0R0=(0,0), +8dm each row
            cell_id = "C{}R{}".format(col, row)
            route.append((x_dm, y_dm, z_dm, cell_id))

    return route


def plan_landing_route_g(takeoff_x_dm=0.0, takeoff_y_dm=0.0):
    """
    生成返航降落点。

    从最后一个巡逻点飞回起飞区域上空。

    Args:
        takeoff_x_dm (float): 起飞区域 X 坐标（dm），默认 1.0。
        takeoff_y_dm (float): 起飞区域 Y 坐标（dm），默认 1.0。

    Returns:
        list: [(x_dm, y_dm, z_dm, cell_id)]
    """
    return [(takeoff_x_dm, takeoff_y_dm, 10.0, "LAND")]


def validate_coverage_route(route):
    """
    校验全覆盖路径。

    Args:
        route (list): plan_coverage_route_g 返回值。

    Returns:
        tuple: (ok, problems)
    """
    problems = []
    all_cells = set()
    for col in range(G_PATROL_COLS):
        for row in range(G_PATROL_ROWS):
            all_cells.add("C{}R{}".format(col, row))

    covered = set(r[3] for r in route if r[3].startswith("C"))
    missing = all_cells - covered
    if missing:
        problems.append("未覆盖宏格: {}".format(sorted(missing)))

    extra = covered - all_cells - {"START", "LAND"}
    if extra:
        problems.append("多余宏格: {}".format(sorted(extra)))

    # 检查相邻航点距离（蛇形折返时 col 切换跳跃约 20dm 是正常的）
    for i in range(1, len(route)):
        x1, y1 = route[i-1][0], route[i-1][1]
        x2, y2 = route[i][0], route[i][1]
        dist = ((x2-x1)**2 + (y2-y1)**2)**0.5
        if dist > G_CELL_SIZE_DM * 2:
            problems.append("航点{}→{}间距过大: {:.0f}dm".format(i-1, i, dist))

    return (len(problems) == 0, problems)


if __name__ == "__main__":
    route = plan_coverage_route_g()
    print("G 题全覆盖路径 ({} 个宏格):".format(len(route)))
    for x, y, z, cell in route:
        print("  {:6s}  ({:.0f}, {:.0f}) dm".format(cell, x, y))
    ok, problems = validate_coverage_route(route)
    if ok:
        print("校验通过 ✓")
    else:
        print("校验失败:")
        for p in problems:
            print("  ✗", p)


# ==============================================================================
# D 题搜救飞行器路径规划 (蛇形搜索 + A* 引导)
# ==============================================================================
import heapq

D_GRID_COLS = 10
D_GRID_ROWS = 8
D_CELL_SIZE_DM = 5
D_FLIGHT_HEIGHT_DM = 10.0   # 1.0m
D_DROP_HEIGHT_DM = 5.0      # 0.5m 投放高度

# 禁飞区: 从 uav_interfaces 导入唯一真相源 (避免两处定义不同步导致 A* 穿禁飞区)
try:
    from uav_interfaces import NO_FLY_ZONE_SET as _D_NO_FLY_ZONES
except ImportError:
    # 纯逻辑自测时 uav_interfaces 可能不在路径, 用本地副本兜底
    _D_NO_FLY_ZONES = frozenset([
        (5, 0),
        (2, 1), (3, 1), (8, 1), (9, 1),
        (0, 2), (5, 2), (6, 2), (8, 2),
        (2, 3),
        (0, 4), (4, 4), (8, 4),
        (1, 5), (3, 5), (6, 5),
        (6, 6),
        (1, 7), (4, 7), (8, 7),
    ])


def plan_serpentine_route_d():
    """D 题蛇形 (S 形) 遍历 80 格搜索路径。

    沿 Y 方向蛇形、X 方向递进:
      X0: Y0 -> Y7   (X0Y0, X0Y1, ..., X0Y7)
      X1: Y7 -> Y0   (X1Y7, X1Y6, ..., X1Y0)  蛇形折返
      X2: Y0 -> Y7
      ...
      X9: Y7 -> Y0   (X 为偶数索引时 Y 递增, 奇数索引时 Y 递减)
    搜索阶段可飞越禁飞区, 在每格上方 1.0m 悬停检测目标桶。

    Returns:
        list: [(x_dm, y_dm, z_dm, cell_id), ...]  cell_id = "X{gx}Y{gy}"
    """
    route = []
    z_dm = D_FLIGHT_HEIGHT_DM
    for gx in range(D_GRID_COLS):
        if gx % 2 == 0:
            ys = range(D_GRID_ROWS)               # Y0 -> Y7
        else:
            ys = range(D_GRID_ROWS - 1, -1, -1)   # Y7 -> Y0 (蛇形折返)
        for gy in ys:
            x_dm = gx * D_CELL_SIZE_DM
            y_dm = gy * D_CELL_SIZE_DM
            route.append((x_dm, y_dm, z_dm, "X{}Y{}".format(gx, gy)))
    return route


def _a_star_neighbors(gx, gy, no_fly):
    """A* 4 邻域, 避开禁飞区, 不出场地。"""
    result = []
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = gx + dx, gy + dy
        if 0 <= nx < D_GRID_COLS and 0 <= ny < D_GRID_ROWS:
            if (nx, ny) not in no_fly:
                result.append((nx, ny))
    return result


def _a_star_heuristic(a, b):
    """曼哈顿距离启发。"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plan_a_star_route(start, goal, avoid_no_fly=True):
    """A* 栅格最短路径 (引导阶段)。

    Args:
        start: (gx, gy) 起点格 (被引导目标格, 必为白色通行格)
        goal: (gx, gy) 终点格 (出口格, 必为白色通行格)
        avoid_no_fly: True 时避开禁飞区 (引导阶段必须 True)

    Returns:
        list: [(gx, gy), ...] 从起点到终点的格子序列, 起止格均计入。
              无路径时返回空列表。

    注意:
        目标桶只在白色通行格, 出口也在白色通行格, 起止均不会在禁飞区。
        若传入禁飞格作起止点, 视为异常返回空列表。
    """
    no_fly = _D_NO_FLY_ZONES if avoid_no_fly else set()
    if start in no_fly or goal in no_fly:
        return []

    open_heap = []
    counter = 0
    heapq.heappush(open_heap, (0, counter, start))
    came_from = {start: None}
    g_score = {start: 0}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            break
        for nxt in _a_star_neighbors(current[0], current[1], no_fly):
            tentative = g_score[current] + 1
            if nxt not in g_score or tentative < g_score[nxt]:
                g_score[nxt] = tentative
                came_from[nxt] = current
                f = tentative + _a_star_heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (f, counter, nxt))

    if goal not in came_from:
        return []

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path


def select_farthest_target(targets, exit_cell):
    """选择距离出口路径最远 (A* 格数最多) 的目标。

    Args:
        targets: [(gx, gy, color), ...] 3 个已投放目标
        exit_cell: (gx, gy) 出口格

    Returns:
        tuple: (target, path) 被选目标及其 A* 路径; 无路径返回 (None, [])
    """
    best = None
    best_path = []
    best_len = -1
    for t in targets:
        start = (t[0], t[1])
        path = plan_a_star_route(start, exit_cell, avoid_no_fly=True)
        if len(path) > best_len:
            best_len = len(path)
            best = t
            best_path = path
    return best, best_path


def validate_serpentine_route_d(route):
    """校验蛇形路径覆盖全部 80 格。"""
    problems = []
    all_cells = set()
    for gx in range(D_GRID_COLS):
        for gy in range(D_GRID_ROWS):
            all_cells.add("X{}Y{}".format(gx, gy))
    covered = set(r[3] for r in route if r[3].startswith("X"))
    missing = all_cells - covered
    if missing:
        problems.append("未覆盖格: {}".format(sorted(missing)))
    return (len(problems) == 0, problems)
