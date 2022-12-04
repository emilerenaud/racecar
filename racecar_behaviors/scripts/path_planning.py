"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math

import matplotlib.pyplot as plt
import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libbehaviors import *

show_animation = True


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
        # self.min_x = 7
        # self.min_y = 2.5
        # self.max_x = 22
        # self.max_y = 7
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 1, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 1, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o])+ self.brushfire_map[int(open_set[o].x)][int(open_set[o].y)])
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
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        # rospy.loginfo(type(self.min_x))
        # rospy.loginfo(type(min(ox)))
        self.min_x = int(round(min(ox)))
        rospy.loginfo(type(self.min_x))
        self.min_y = int(round(min(oy)))
        self.max_x = int(round(max(ox)))
        self.max_y = int(round(max(oy)))

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


##############################  CRETATE BRUSHFIRE MAP  ################################
def get_map():

    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return
    
    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0
    cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1)))
    brushfireMap = brushfire(grid)
    return np.flip(grid.T), np.flip(brushfireMap)


############################## A * PATH TO DEBRIS FUNCTION  ################################
def path_to_debris(sx, sy, gx, gy, file_path):
        
    grid,brushfire_map = get_map()
    robot_radius = 0.01
    ox, oy = [], []
    obstacles = np.where(grid==0)
    for n in range(obstacles[1].shape[0]):
        ox.append((obstacles[1][n]/10))
        oy.append((obstacles[0][n]/10))

    a_star = AStarPlanner(ox, oy, 0.1, robot_radius,brushfire_map)
    rx, ry = a_star.planning(sx, sy, gx, gy)


    for n, _ in enumerate(rx):
        grid[int(ry[n]*10), int(rx[n]*10)] = 0
    plt.imshow(grid)
    plt.savefig(file_path)


def main():
    print(__file__ + " start!!")

    ####################################### B R U S H F I R E #######################################
    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return
    
    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    
    brushfireMap = brushfire(grid)
    
    
    # Export brusfire map for visualization
    # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
    maximum = np.amax(brushfireMap)
    if maximum > 1:
        mask = brushfireMap==1; 
        brushfireMap = brushfireMap.astype(float) / float(maximum) *225.0 + 30.0
        brushfireMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfireMap, -1)))
        rospy.loginfo("Exported brushfire.bmp")
    else:
        rospy.loginfo("brushfire failed! Is brusfire implemented?")
    
    # Example to show grid with same color than RVIZ
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0
    # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1))) 
    rospy.loginfo("Exported map.bmp")

    # plt.plot(brushfireMap,"xb")
    # plt.show()


    ############################## A *  P A T H  P L A N N I N G ################################

    # start and goal position
    sx = 7.5# [m]
    sy = 3  # [m]
    # gx = 13.5  # [m]
    # gy = 2.1  # [m]
    gx = 21
    gy = 5.1
 
    robot_radius = 0.01
    # grid_size = 0.01

    # Faire matrice d'obstale
    ox, oy = [], []

    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    
    # np.flip(grid.T)
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0

    nRows, nCols = grid.shape

    ox, oy = [], []
    obstacles = np.where(grid==0) # 1200
    # obstacles = np.where(brushfireMap<50) #68199

    rospy.loginfo(obstacles[0].size)

    rospy.loginfo(obstacles)
    for n in range(obstacles[1].shape[0]):
        ox.append((obstacles[1][n]/10))
        oy.append((obstacles[0][n]/10))

    
    # oy = obstacles[0]
    # ox = obstacles[1]

    # for x in range(nRows):
    #     for y in range(nCols):
    #         if(grid[x][y] == 0):
    #             ox.append(x)
    #             oy.append(y)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    # a_star = AStarPlanner(ox, oy, response.map.info.resolution, robot_radius)
    a_star = AStarPlanner(ox, oy, 0.1, robot_radius,brushfireMap)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        # plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()