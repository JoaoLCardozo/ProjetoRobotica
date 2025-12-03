import math


def avoid_point_obstacles(target, obstacles, min_clearance=0.12):
    """Simple obstacle avoidance: if target lies inside obstacle clearance,
    push it outward along the vector from obstacle center to target.
    Returns adjusted (x,y).
    """
    tx, ty = target
    for obs in obstacles:
        ox, oy = obs['pos']
        r = obs['radius'] + min_clearance
        dx = tx - ox
        dy = ty - oy
        d = math.hypot(dx, dy)
        if d < r and d > 1e-6:
            # push target to the circle boundary
            scale = r / d
            tx = ox + dx * scale
            ty = oy + dy * scale
        elif d <= 1e-6:
            # target exactly on obstacle center: push along x
            tx = ox + r
            ty = oy
    return tx, ty
