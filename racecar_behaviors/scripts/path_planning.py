
import math

import matplotlib.pyplot as plt
import rospy
import cv2
import numpy as np
from libbehaviors import *



import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from random import random

import math
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import time
from libbehaviors import brushfire
import rospy
from nav_msgs.srv import GetMap
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import cv2


def astar(start, goal, c_fun, n_fun, h_fun,brushfireMap):
    # Cherche le chemin le plus court entre start et goal à l'aide de l'algorithme A*.
    # Retourne un tuple de la séquence et du coût : (seq, cost)
    # Utilise les fonctions :
    #   c_fun(edge): retourne le coût de parcours du lien edge
    #   n_fun(node): retourne les voisins du noeud node
    #   h_fun(node_a, node_b): retourne la valeur de l'heuristique du coût de parcours entre node_a et node_b
    
    from math import inf # Valeur infinie
    
    search_set = [start] # Ensemble de recherche, ne contient que le noeud de départ pour l'instant.
    
    # Dictionnaire contenant le plus bas coût réel du chemin (G) jusqu'au noeud spécifié.
    g = {}
    # On initialise avec g = 0 pour le départ
    g[start] = 0
    
    # Dictionnaire contenant la plus basse valeur de la fonction f(node) pour chaque noeud.
    f = {} 
    # On initialise f(start) avec g (qui est 0) et l'heuristique jusqu'à la fin.
    f[start] = g[start] + h_fun(start, goal)
   
    # Dictionnaire qui conserve pour chaque noeud la provenance permettant la valeur f la plus faible.
    # Utilisé pour reconstruire le chemin lorsque l'objectif est atteint.
    from_node = {}
    # On initialise avec le noeud de départ sans provenance (None).
    from_node[start] = None
    
    while (len(search_set) > 0):
        # On trouve le noeud ayant la plus petite valeur f dans l'ensemble de recherche :
        min_f = inf
        min_n = None
        for n in search_set:
            assert n in f, "Error : %s not in F!"%(str(n))
            if f[n] < min_f:
                min_f = f[n]
                min_n = n
        
        # On retire le noeud ayant le plus petit 'f' et on poursuit avec lui :
        current = min_n
        print("Current: %s, F(%s) = %d"%(current, current, min_f))
        search_set.remove(min_n)
        
        # Si le noeud en cours est l'objectif, c'est qu'aucun autre noeud dans l'espace de recherche n'a
        # un meilleur potentiel du chemin le plus court. On reconstruit le chemin ensuite. 
        if (current == goal):
            path_r = [goal]
            previous = from_node[goal]
            while (previous is not None):
                path_r.append(previous)
                previous = from_node[previous]             
            path_r.reverse() # On remet la liste dans le bon ordre
            return (path_r, g[goal])
        
        ns = n_fun(current) # Les voisins du noeud en cours
        for n in ns:
            # Pour chaque voisin, on calcule sa fonction f et on l'ajoute à l'ensemble de recherche seulement si
            # la valeur de g est plus basse que celle déjà connue pour ce noeud
            g_n = g[current] + c_fun(current, n,brushfireMap)
            f_n = g_n + h_fun(n, goal)
            print("G(%s) = %d, F(%s) = %d"%(n, g_n, n, f_n))
            if ((n not in g) or (g_n < g[n])):
                from_node[n] = current
                g[n] = g_n
                f[n] = f_n
                if (n not in search_set):
                    print("Adding %s"%(str(n)))
                    search_set.append(n)
   
    # Nous avons vidé l'espace de recherche sans trouver de solution. Retourner une liste vide et un coût négatif.
    return ([], -1)

def rnd_map(size, n_obs):
    # Génère une matrice de taille size * size contenant n_obs obstacles.
    obs = np.zeros((size,size))
    
    for oi in range(n_obs):
        xl = min(size - 1, int(random()*size))
        xh = min(size - 1, xl + max(1, int(random()*size/3)))
        yl = min(size - 1, int(random()*size))
        yh = min(size - 1, yl + max(1, int(random()*size/3)))
        obs[xl:xh, yl:yh] = 1
            
    return obs

def rnd_point(shape):
    # Génère un point au hasard dans les limites définies par shape.
    # Retourne un tuple (x,y)
    from random import random
    w = shape[0]
    h = shape[1]
    x = int(random()*w)
    y = int(random()*h)
    return (x,y)

def rnd_task(obs_map):
    # Génère deux points au hasard (start, goal) dans des cases vides de la carte obs_map.
    # Retourne le tuple (start, goal)
    start = rnd_point(obs_map.shape)
    # On répète si on tombe sur autre chose qu'une case vide :
    while (obs_map[start] != 0):
        start = rnd_point(obs_map.shape)
    # Même chose pour l'objectif :
    goal = rnd_point(obs_map.shape)
    while (obs_map[goal] != 0):
        goal = rnd_point(obs_map.shape)
    return (start, goal)


def draw_map(obs_map, start, goal):

    sns.heatmap(data=obs_map, annot=False)
    # NOTE : Le système de coordonnées est inversé pour le "scatter plot" :
    (s_y, s_x) = start
    (g_y, g_x) = goal
    plt.scatter(x=s_x+0.4, y=s_y+0.4, color="blue")
    plt.scatter(x=g_x+0.4, y=g_y+0.4, color="green")
    

    


def m_cost(node_a, node_b,brushfireMap):
    # Pour une carte, le coût est toujours de 1.
    # On s'assure tout de même que les cellules sont adjacentes.
    dist_x = abs(node_a[0] - node_b[0])
    dist_y = abs(node_a[1] - node_b[1])
    assert dist_x <= 1 and dist_y <= 1, "m_cost : %s is not a neighbor of %s"%(str(node_a), str(node_b))
    return math.sqrt(dist_x**2 + dist_y**2) + brushfireMap[node_b[0]][node_b[1]]



def m_neighbors_8(node, obs_map):
    # Génère les voisins valides de node selon la carte (obs_map)
    # Connectivité 8.
    # Retourne une liste de tuples dont les cases sont vides (=0).
    ns = []
    x = node[0]
    y = node[1]
    lx = obs_map.shape[0] - 1
    ly = obs_map.shape[1] - 1
    
    min_x = -1 if (x > 0) else 0
    min_y = -1 if (y > 0) else 0
    max_x = 2 if (x < lx) else 1
    max_y = 2 if (y < ly) else 1
    
    for dx in range(min_x, max_x):
        for dy in range(min_y, max_y):
            if ((dx == 0) and (dy == 0)):
                continue
            n = (x+dx, y+dy)
            if (obs_map[n] == 0):
                ns.append(n)
            
    return ns

def m_h(node_a, node_b):
    from math import sqrt
    (ax, ay) = node_a
    (bx, by) = node_b
    return sqrt((ax-bx)**2 + (ay-by)**2)

def draw_path(obs_map, start, goal, seq):
    import matplotlib.pyplot as plt
    import numpy as np
    draw_map(obs_map, start, goal)
    points = np.asarray(seq)
    plt.scatter(y=points[:,0]+0.4, x=points[:,1]+0.4, color="white")

def m_to_px(m):
    px = m/0.1
    return round(px)

def main():
    print(__file__ + " start!!")

    

############################## A * PATH TO DEBRIS FUNCTION  ################################

def path_to_debris(start, end, file_path):
    ################## B R U S H F I R E ####################
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
        # max_brush = brushfireMap.max()
        # # print(max_brush)
        # for x in range(brushfireMap.shape[0]):
        #     for y in range(brushfireMap.shape[1]):
        #         brushfireMap[x][y] = max_brush - brushfireMap[x][y]
            
        # cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfireMap, -1)))
        rospy.loginfo("Exported brushfire.bmp")
    else:
        rospy.loginfo("brushfire failed! Is brusfire implemented?")
    
    # # Example to show grid with same color than RVIZ
    grid[grid == -1] = 0
    grid[grid == 0] = 0
    grid[grid == 100] = 1
    # # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    # cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1))) 
    rospy.loginfo("Exported map.bmp")

    # plt.plot(grid,"xb")
    # plt.show()
    ############################## A *  P A T H  P L A N N I N G ################################
 
    
    max_brush = brushfireMap.max()
    for x in range(brushfireMap.shape[0]):
            for y in range(brushfireMap.shape[1]):
                brushfireMap[x][y] = max_brush - brushfireMap[x][y]


    m_start = (start[0],start[1])
    m_goal = (end[0],end[1])
    draw_map(grid, m_start, m_goal)
    print("On cherche le trajet de %s à %s."%(str(m_start), str(m_goal)))

    # Permet de définir une nouvelle fonction à un seul paramètre. Ainsi, m_n(node) correspond à m_neighbors_8(node, obs_map)
    m_n = lambda node : m_neighbors_8(node, grid)

    (seq, cost) = astar(m_start, m_goal, m_cost, m_n, m_h,brushfireMap)
    print("Le plus court chemin entre %s et %s est : %s (%d)"%(m_start, m_goal, seq, cost))
    draw_path(grid, m_start, m_goal, seq[1:len(seq)-1])
    
    # aStartPath = r'/home/emile/racecar/debris'
    plt.savefig(file_path)

    # plt.show()




def main():
    print(__file__ + " start!!")


if __name__ == '__main__':
    main()