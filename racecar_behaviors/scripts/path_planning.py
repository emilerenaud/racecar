#!/usr/bin/env python

import math
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import time
from libbehaviors import brushfire, eucledian_map, get_map
import rospy
from nav_msgs.srv import GetMap
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 200.0  # attractive potential gain
ETA = 1.0  # repulsive potential gain SIZE OF WALL
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 10
N_VOXEL_PER_M = 100
show_animation = False

##########################################################################################
class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, brushfire_map):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)
        self.brushfire_map = brushfire_map

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o])+ self.brushfire_map[int(open_set[o].x)][int(open_set[o].y)])
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = int(round((self.max_x - self.min_x) / self.resolution))
        self.y_width = int(round((self.max_y - self.min_y) / self.resolution))
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
##########################################################################################


def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy, brushfire_map):
    # minx = min(min(ox), sx, gx) / 2.0
    # miny = min(min(oy), sy, gy) / 2.0
    # maxx = max(max(ox), sx, gx) / 2.0
    # maxy = max(max(oy), sy, gy) / 2.0

    minx = 0
    miny = 0
    maxx = brushfire_map.shape[0]/N_VOXEL_PER_M
    maxy = brushfire_map.shape[1]/N_VOXEL_PER_M

    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            pmap[ix][iy] = calc_attractive_potential(x, y, gx, gy)

    pmap = np.array(pmap)


    return pmap, brushfire_map, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)

def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

def update_bump(pmap, brushfire_map, x, y):
    #returns the new cost map
    x = int(x)
    y = int(y)
    # bump = np.array([[1.1,1.2,1.2,1.2],[1.2,1.4,1.4,1.2],[1.2,1.4,1.4,1.2],[1.1,1.2,1.2,1.1]])
    # brushfire_map[x-2:x+2, y-2:y+2] = brushfire_map[x-2:x+2,y-2:y+2] + bump
    
    cost_map = pmap + brushfire_map
    # plt.imshow(cost_map)
    # plt.show()
    return list(cost_map)

def update_maps(pmap, brushfire_map, k_pmap, k_brushfire):
    #returns the new cost map

    pmap = pmap*k_pmap
    brushfire_map = k_brushfire * brushfire_map

    return pmap, brushfire_map


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr, brushfire_map):

    # calc potential field
    pmap, brushfire_map, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy, brushfire_map)

    cost_map = list(pmap + brushfire_map)
    
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(cost_map)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()
    k_pmap = 1
    k_brushfire = 1
    while d >= reso:

        cost_map = list(brushfire_map+pmap)

        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(cost_map) or iny >= len(cost_map[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = cost_map[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            k_pmap -= 0.01
            pmap, brushfire_map = update_maps(pmap, brushfire_map, k_pmap , k_brushfire)
        else:
            k_pmap = 1
            pmap, brushfire_map = update_maps(pmap, brushfire_map, k_pmap , k_brushfire)
        print(k_pmap)
        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.001)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=1000.0, cmap=plt.cm.Blues)


def get_grid(image_path):
    img = Image.open(image_path)
    grid = np.array(img)
    return grid


def get_shortest_path_image(algo, export_path, sx, sy, gx, gy):
    """
    Exports an image showing the shortest path to a point on a map
    image_path = path to get the bitmap (.bmp) from 
    export path = path to where the image will be saved
    start and end _x and _y = points in the robots reference (in meter)
    """


    robot_radius = 0.0001
    grid_size = 0.01 # 0.01 #ratio entre les pixele et les ox oy etc. si les position sont en m et 100 pixel par m : 

    grid, brushfire_map = get_map()

    #get obstacles from map
    ox, oy = [], []
    obstacles = np.where(grid==0)
    for n in range(obstacles[1].shape[0]):
        ox.append(obstacles[1][n]/N_VOXEL_PER_M)
        oy.append(obstacles[0][n]/N_VOXEL_PER_M)

# To show the obstacles on the map
    # map = np.zeros(grid.shape)
    # for i in range(len(ox)):
    #     map[oy[i]][ox[i]] = 1
    # map[sy][sx] = 2
    # map[gy][gx] = 2
    # # plt.imshow(map)

    if algo == "potential":
        if show_animation:
            plt.grid(True)
            plt.axis("equal")

        # path generation
        _, _ = potential_field_planning(
            sx, sy, gx, gy, ox, oy, grid_size, robot_radius, brushfire_map)

        if show_animation:
            plt.show()


        # plt.savefig('scripts/my_beautiful_twisted_path')
        # plt.show()

    elif algo == "a_star":
       
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius, brushfire_map)
        rx, ry = a_star.planning(sx, sy, gx, gy)

        for n, _ in enumerate(rx):
            grid[int(ry[n]*N_VOXEL_PER_M), int(rx[n]*N_VOXEL_PER_M)] = 0
        plt.imshow(grid)
        plt.savefig(export_path)


    
def path_rapport(algo, save_path,sx,sy,gx,gy):
    print("started rapport for fastest path")
    # sx = 140.0/N_VOXEL_PER_M  # [m]
    # sy = 317.0/N_VOXEL_PER_M  # [m]
    # gx = 168.0/N_VOXEL_PER_M  # [m]
    # gy = 230.0/N_VOXEL_PER_M  # [m]

    #grid = get_grid("map.bmp")

    get_shortest_path_image(algo, save_path, sx, sy, gx, gy)


if __name__ == '__main__':
    path_rapport("a_star", 'moma.png', 140.0/N_VOXEL_PER_M,317.0/N_VOXEL_PER_M,168.0/N_VOXEL_PER_M, 230.0/N_VOXEL_PER_M)