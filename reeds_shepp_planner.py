import numpy as np
import math
import matplotlib.pyplot as plt


show_animation = True


class Path:

    def __init__(self):
        self.lengths = []
        self.ctypes = []
        self.L = 0.0
        self.x = []
        self.y = []
        self.yaw = []
        self.directions = []


def polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta

def mod2pi(x):
    v = np.mod(x, 2.0 * math.pi)
    if v < -math.pi:
        v += 2.0 * math.pi
    else:
        if v > math.pi:
            v -= 2.0 * math.pi
    return v


def set_path(paths, lengths, ctypes):

    path = Path()
    path.ctypes = ctypes
    path.lengths = lengths

    # check same path exist
    for tpath in paths:
        typeissame = (tpath.ctypes == path.ctypes)
        if typeissame:
            if sum(tpath.lengths) - sum(path.lengths) <= 0.01:
                return paths

    path.L = sum([abs(i) for i in lengths])

    if path.L >= 0.01:
        paths.append(path)

    return paths

# Straight Curve Straight
def SCS(x, y, pthe, paths):
    flag, t, u, v = SLS(x, y, pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "L", "S"])

    flag, t, u, v = SLS(x, -y, -pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "R", "S"])

    return paths

# SCS: SRS and SLS
def SLS(x, y, pthe):
    pthe = mod2pi(pthe)
    if y > 0.0 and pthe > 0.0 and pthe < math.pi * 0.99:
        # if tan(theta) = y/x, xd = 0
        xd = - y / math.tan(pthe) + x
        t = xd - math.tan(pthe / 2.0)
        u = pthe
        v = math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(pthe / 2.0)
        return True, t, u, v
    elif y < 0.0 and pthe > 0.0 and pthe < math.pi * 0.99:
        xd = - y / math.tan(pthe) + x
        t = xd - math.tan(pthe / 2.0)
        u = pthe
        v = -math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(pthe / 2.0)
        return True, t, u, v

    return False, 0.0, 0.0, 0.0

# Curve Curve Curve
def CCC(x, y, pthe, paths):
    # LRL
    flag, t, u, v = LRL(x, y, pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "R", "L"])

    flag, t, u, v = LRL(-x, y, -pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "R", "L"])

    # RLR
    flag, t, u, v = LRL(x, -y, -pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "L", "R"])

    flag, t, u, v = LRL(-x, -y, pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "L", "R"])

    # backwards
    xb = x * math.cos(pthe) + y * math.sin(pthe)
    yb = x * math.sin(pthe) - y * math.cos(pthe)

    # LRL
    flag, t, u, v = LRL(xb, yb, pthe)
    if flag:
        paths = set_path(paths, [v, u, t], ["L", "R", "L"])

    flag, t, u, v = LRL(-xb, yb, -pthe)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["L", "R", "L"])

    # RLR
    flag, t, u, v = LRL(xb, -yb, -pthe)
    if flag:
        paths = set_path(paths, [v, u, t], ["R", "L", "R"])

    flag, t, u, v = LRL(-xb, -yb, pthe)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["R", "L", "R"])

    return paths

# CCC: RLR and LRL
def LRL(x, y, pthe):
    u1, t1 = polar(x - math.sin(pthe), y - 1.0 + math.cos(pthe))

    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u + math.pi)
        v = mod2pi(pthe - t + u)

        if t >= 0.0 and u <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0

# Curve Straight Curve
def CSC(x, y, pthe, paths):
    # LSL
    flag, t, u, v = LSL(x, y, pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "L"])

    flag, t, u, v = LSL(-x, y, -pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "L"])

    # RSR
    flag, t, u, v = LSL(x, -y, -pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "R"])

    flag, t, u, v = LSL(-x, -y, pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "R"])

    # LSR
    flag, t, u, v = LSR(x, y, pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "R"])

    flag, t, u, v = LSR(-x, y, -pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "R"])

    # RSL
    flag, t, u, v = LSR(x, -y, -pthe)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "L"])

    flag, t, u, v = LSR(-x, -y, pthe)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "L"])

    return paths

# CSC: LSR and RSL
def LSR(x, y, pthe):
    u1, t1 = polar(x + math.sin(pthe), y - 1.0 - math.cos(pthe))
    u1 = u1 ** 2
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - pthe)

        if t >= 0.0 and v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0

# CSC: LSL and RSR
def LSL(x, y, pthe):
    u, t = polar(x - math.sin(pthe), y - 1.0 + math.cos(pthe))
    if t >= 0.0:
        v = mod2pi(pthe - t)
        if v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def interpolate(ind, l, m, minr, ox, oy, oyaw, px, py, pyaw, directions):
    # ind: ith point of npoint
    # l: length
    # m: current mode (R,S,L)
    # o: origin state
    # p: procedural state
    
    if m == "S":  # Straight
        px[ind] = ox + l * minr * math.cos(oyaw)
        py[ind] = oy + l * minr * math.sin(oyaw)
        pyaw[ind] = oyaw
    else:  # Curve
        ldx = math.sin(l) * minr
        if m == "L":
            ldy = (1.0 - math.cos(l)) * minr
        elif m == "R":
            ldy = -(1.0 - math.cos(l)) * minr
        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "L":
        pyaw[ind] = oyaw + l
    elif m == "R":
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_local_course(L, lengths, mode, minr, step_size):
    # L: path total length
    # mode: optimal path type (LSL, RSL ...)
    
    npoint = math.trunc(L / step_size) + len(lengths) + 4

    px = [0.0 for i in range(npoint)]
    py = [0.0 for i in range(npoint)]
    pyaw = [0.0 for i in range(npoint)]
    directions = [0.0 for i in range(npoint)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    pd = d
    ll = 0.0

    
    for (m, l, i) in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        # set origin state
        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = - d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = interpolate(ind, pd, m, minr, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        px, py, pyaw, directions = interpolate(ind, l, m, minr, ox, oy, oyaw, px, py, pyaw, directions)

    # remove unused data
    while px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions


def generate_path(q0, q1, minr):
    # start
    # q0 = [sx, sy, syaw]
    # goal
    # q1 = [gx, gy, gyaw]
    # minr: minimun radius, step_size: step size
    
    dx  = q1[0] - q0[0]
    dy  = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = math.cos(q0[2])
    s = math.sin(q0[2])
    x = ( c * dx + s * dy) / minr
    y = (-s * dx + c * dy) / minr

    paths = []
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)

    return paths


def get_paths(sx, sy, syaw, gx, gy, gyaw, minr, step_size):
    # start
    q0 = [sx, sy, syaw]
    # goal
    q1 = [gx, gy, gyaw]
    # minr: minimun radius, step_size: step size

    paths = generate_path(q0, q1, minr)
    for path in paths:
        x, y, yaw, directions = generate_local_course(path.L, path.lengths, path.ctypes, minr, step_size / minr)

        # get coordinates of end in the set of axis where start is (0,0,0)
        path.x = [ math.cos(-q0[2])*ix + math.sin(-q0[2])*iy + q0[0]  for (ix, iy) in zip(x, y)]
        path.y = [-math.sin(-q0[2])*ix + math.cos(-q0[2])*iy + q0[1]  for (ix, iy) in zip(x, y)]
        path.yaw = [(iyaw + q0[2] + math.pi) % (2*math.pi) - math.pi for iyaw in yaw]
        path.directions = directions
        path.lengths = [l * minr for l in path.lengths]
        path.L = path.L * minr

    return paths


def reeds_shepp_planner(sx, sy, syaw,
                              gx, gy, gyaw, minr, step_size):
    # sx: start x, sy: start y, syaw: start orientation, gxgygyaw: goal
    # minr: minimun radius, step_size: step size
    
    paths = get_paths(sx, sy, syaw, gx, gy, gyaw, minr, step_size)

    if len(paths) == 0:
        return None, None, None, None, None

    minL = float("Inf")
    best_path_index = -1
    for i in range(len(paths)):
        if paths[i].L <= minL:
            minL = paths[i].L
            best_path_index = i

    bpath = paths[best_path_index]

    return bpath.x, bpath.y, bpath.yaw, bpath.ctypes, bpath.lengths


def plot_arrow(x, y, yaw, length=0.01, width=0.15, fc="r", ec="k"):

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width/2, head_length=width)
        plt.plot(x, y)
        

def main():
    
    start_x = -2.0
    start_y = -0.5
    start_yaw = 0.0

    end_x = 2.0
    end_y = -0.5
    end_yaw = math.pi / 2

    min_radius = 0.4
    step_size = 0.02

    px, py, pyaw, mode, clen = reeds_shepp_planner(start_x, start_y, start_yaw, end_x, end_y, end_yaw, min_radius, step_size)

    print(px)
    
    if show_animation:
        plt.cla()
        plt.plot(px, py, label= str(mode))

        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    if not px:
        assert False, "No path"


if __name__ == '__main__':
    main()
