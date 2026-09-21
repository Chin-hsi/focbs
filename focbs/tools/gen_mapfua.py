#!/usr/bin/env python3
"""Generate .mapfua instances from MovingAI .map and .scen files.

Modes:
  1. With --scen: assigned agents are the first |A| rows from the scenario file,
     and unassigned agents are sampled from free cells.
  2. Without --scen: assigned and unassigned positions are sampled randomly from
     the largest connected component.
"""

import argparse
import math
import random
import shutil
import sys
from collections import deque
from pathlib import Path


def read_movingai_map(path):
    with path.open("r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]
    if len(lines) < 4:
        raise ValueError("map file is too short")
    if lines[0].strip() != "type octile":
        raise ValueError("expected first line to be 'type octile'")
    dimensions = []
    for line, name in zip(lines[1:3], ("height", "width")):
        parts = line.split()
        if len(parts) != 2 or parts[0] != name or int(parts[1]) <= 0:
            raise ValueError(f"invalid map {name}")
        dimensions.append(int(parts[1]))
    height, width = dimensions
    if width * height > 2**31 - 1:
        raise ValueError("map dimensions exceed the supported range")
    if lines[3].strip() != "map":
        raise ValueError("expected 'map' grid header")
    rows = lines[4: 4 + height]
    if len(rows) != height:
        raise ValueError("map file has too few grid rows")

    normalized = []
    for y, row in enumerate(rows):
        if len(row) != width:
            raise ValueError(f"incorrect width in map row {y + 1}")
        if any(cell not in ".GS@OTW" for cell in row):
            raise ValueError(f"unsupported terrain in map row {y + 1}")
        normalized.append("".join("." if cell in ".GS" else "@" for cell in row))
    return width, height, normalized


def read_scen(path, num_assigned, width, height):
    """Return the first num_assigned ((sx, sy), (gx, gy)) pairs from a .scen file."""
    agents = []
    with path.open("r", encoding="utf-8") as f:
        header = f.readline().split()
        if len(header) != 2 or header[0] != "version" or float(header[1]) != 1:
            raise ValueError("expected .scen version 1 header")
        for i in range(num_assigned):
            parts = f.readline().split()
            if len(parts) != 9:
                raise ValueError(f"missing or malformed .scen row {i + 1}")
            if int(parts[0]) < 0 or not math.isfinite(float(parts[8])) or float(parts[8]) < 0:
                raise ValueError(f"invalid .scen row {i + 1}")
            if (int(parts[2]), int(parts[3])) != (width, height):
                raise ValueError(f".scen dimensions do not match the map in row {i + 1}")
            sx, sy = int(parts[4]), int(parts[5])
            gx, gy = int(parts[6]), int(parts[7])
            agents.append(((sx, sy), (gx, gy)))
    return agents


def validate_assigned(assigned, grid, width, height):
    starts, goals = set(), set()
    for start, goal in assigned:
        for x, y in (start, goal):
            if not (0 <= x < width and 0 <= y < height) or grid[y][x] != ".":
                raise ValueError(f".scen coordinate is outside the map or on an obstacle: {x},{y}")
        if start in starts or goal in goals:
            raise ValueError("duplicate .scen start or target location")
        starts.add(start)
        goals.add(goal)


def manhattan_dist(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def connected_components(grid, width, height):
    seen = set()
    components = []
    for y in range(height):
        for x in range(width):
            if (x, y) in seen or grid[y][x] != ".":
                continue
            comp = []
            queue = deque([(x, y)])
            seen.add((x, y))
            while queue:
                cx, cy = queue.popleft()
                comp.append((cx, cy))
                for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                    if 0 <= nx < width and 0 <= ny < height:
                        if (nx, ny) not in seen and grid[ny][nx] == ".":
                            seen.add((nx, ny))
                            queue.append((nx, ny))
            components.append(comp)
    return components


def get_passable_set(grid, width, height):
    cells = set()
    for y in range(height):
        for x in range(width):
            if grid[y][x] == ".":
                cells.add((x, y))
    return cells


def sample_idle(grid, width, height, occupied, num_idle, rng):
    passable = get_passable_set(grid, width, height)
    available = sorted(passable - occupied)
    if num_idle > len(available):
        raise ValueError(
            f"only {len(available)} cells are available for {num_idle} idle agents"
        )
    return rng.sample(available, num_idle)


def sample_instance_random(grid, width, height, num_assigned, num_unassigned, rng):
    comps = connected_components(grid, width, height)
    if not comps:
        raise ValueError("map has no passable cells")
    largest = max(comps, key=len)
    needed = 2 * num_assigned + num_unassigned
    if needed > len(largest):
        raise ValueError(
            f"largest connected component has {len(largest)} cells; need {needed}"
        )
    cells = rng.sample(largest, needed)
    assigned = []
    for i in range(num_assigned):
        assigned.append((cells[2 * i], cells[2 * i + 1]))
    unassigned = cells[2 * num_assigned:]
    return assigned, unassigned


def bfs_dist(grid, width, height, start, goal):
    if start == goal:
        return 0
    sx, sy = start
    gx, gy = goal
    visited = {(sx, sy)}
    queue = deque([(sx, sy, 0)])
    while queue:
        cx, cy, dist = queue.popleft()
        for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
            if 0 <= nx < width and 0 <= ny < height:
                if (nx, ny) in visited or grid[ny][nx] != ".":
                    continue
                if nx == gx and ny == gy:
                    return dist + 1
                visited.add((nx, ny))
                queue.append((nx, ny, dist + 1))
    raise ValueError(f"no path between scenario start {start} and target {goal}")


def compute_horizon(assigned, slack, minimum, grid=None, width=0, height=0):
    if not assigned:
        return minimum
    if grid is not None:
        d_max = max(bfs_dist(grid, width, height, s, g) for s, g in assigned)
    else:
        d_max = max(manhattan_dist(s, g) for s, g in assigned)
    return max(d_max + slack, minimum)


def format_mapfua(map_name, assigned, unassigned, horizon):
    lines = ["mapfua"]
    lines.append(f"map {map_name}")
    lines.append(f"horizon {horizon}")
    lines.append(f"assigned {len(assigned)}")
    for (sx, sy), (gx, gy) in assigned:
        lines.append(f"{sx} {sy} {gx} {gy}")
    lines.append(f"unassigned {len(unassigned)}")
    for sx, sy in unassigned:
        lines.append(f"{sx} {sy}")
    return "\n".join(lines) + "\n"


def ensure_colocated_map(map_path, output_path):
    if output_path == "-":
        return map_path.name
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    dst = out.parent / map_path.name
    if out.resolve() == dst.resolve():
        raise ValueError("output filename must differ from the map filename")
    if dst.exists():
        if dst.read_bytes() != map_path.read_bytes():
            raise ValueError(f"a different map already exists at {dst}")
    else:
        shutil.copyfile(map_path, dst)
    return dst.name


def main():
    parser = argparse.ArgumentParser(
        description="Generate .mapfua instances from MovingAI maps and scenarios"
    )
    parser.add_argument("--map", dest="map_path", required=True)
    parser.add_argument("--scen", dest="scen_path", default=None,
                        help="scenario file for assigned agents")
    parser.add_argument("--num-assigned", type=int, required=True)
    parser.add_argument("--num-unassigned", type=int, required=True)
    parser.add_argument("--horizon", type=int, default=None,
                        help="explicit horizon; otherwise computed automatically")
    parser.add_argument("--horizon-slack", type=int, default=20,
                        help="auto horizon = d_max + slack; default: 20")
    parser.add_argument("--horizon-min", type=int, default=40,
                        help="minimum auto horizon; default: 40")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", required=True, help="output path, or - for stdout")
    args = parser.parse_args()

    if args.num_assigned < 0 or args.num_unassigned < 0:
        parser.error("agent counts must be nonnegative")
    for name in ("horizon", "horizon_slack", "horizon_min"):
        value = getattr(args, name)
        if value is not None and not 0 <= value < 2**31 - 1:
            parser.error(f"--{name.replace('_', '-')} must be between 0 and {2**31 - 2}")

    map_path = Path(args.map_path)
    if not map_path.is_file():
        print(f"map file does not exist: {map_path}", file=sys.stderr)
        return 1

    try:
        if any(c.isspace() for c in map_path.name):
            raise ValueError("map filename cannot contain whitespace")
        if args.output != "-":
            output = Path(args.output).resolve()
            if output == map_path.resolve() or (args.scen_path and output == Path(args.scen_path).resolve()):
                raise ValueError("output must not overwrite an input file")
        width, height, grid = read_movingai_map(map_path)
        rng = random.Random(args.seed)

        if args.scen_path:
            scen_path = Path(args.scen_path)
            if not scen_path.is_file():
                print(f".scen file does not exist: {scen_path}", file=sys.stderr)
                return 1
            assigned = read_scen(scen_path, args.num_assigned, width, height)
            validate_assigned(assigned, grid, width, height)
            occupied = set()
            for (sx, sy), (gx, gy) in assigned:
                occupied.add((sx, sy))
                occupied.add((gx, gy))
            idle_cells = sample_idle(
                grid, width, height, occupied, args.num_unassigned, rng
            )
        else:
            assigned, idle_cells = sample_instance_random(
                grid, width, height, args.num_assigned, args.num_unassigned, rng
            )

        if args.horizon is not None:
            horizon = args.horizon
        else:
            horizon = compute_horizon(
                assigned, args.horizon_slack, args.horizon_min, grid, width, height
            )
        if horizon >= 2**31 - 1:
            raise ValueError("computed horizon exceeds the supported range")

        map_name = ensure_colocated_map(map_path, args.output)
        content = format_mapfua(map_name, assigned, idle_cells, horizon)

        if args.output == "-":
            sys.stdout.write(content)
        else:
            Path(args.output).write_text(content, encoding="utf-8")
    except Exception as exc:
        print(f"generation failed: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
