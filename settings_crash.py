import numpy as np
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import MultiPolygon, Point
from scipy import interpolate
from matplotlib.pyplot import quiver
import math
import random
from copy import deepcopy
from scipy.spatial   import Delaunay,KDTree
from scipy.spatial.distance import euclidean as distance



def padd_walls(lower_wallp, left_wallp, right_wallp, radius):
    # Adds a padding to the walls to make sure agents don't collide
    lwb = lower_wallp.exterior.coords
    lower_wall_padded = [(lwb[0][0],lwb[0][1]),(lwb[1][0],lwb[1][1]+radius), (lwb[2][0],lwb[2][1]+radius),(lwb[3][0],lwb[3][1])]  
    lwbpol = Polygon(lower_wall_padded)

    lewb = left_wallp.exterior.coords
    left_wall_padded = [(lewb[0][0],lewb[0][1]-radius),(lewb[1][0],lewb[1][1]), (lewb[2][0]+radius,lewb[2][1]),(lewb[3][0]+radius,lewb[3][1]-radius)]
    lwebpol = Polygon(left_wall_padded)

    rwb = right_wallp.exterior.coords
    right_wall_padded = [(rwb[0][0]-radius,rwb[0][1]-radius),(rwb[1][0]-radius,rwb[1][1]), (rwb[2][0],rwb[2][1]),(rwb[3][0],rwb[3][1]-radius)]
    rwbpol = Polygon(right_wall_padded)

    polygons = [lwbpol, lwebpol, rwbpol]
    return polygons


def init_map(size_field, ax, radius):

    size_field = 40

    pos = []
    polygons = []
    for x in range(5,40,5):
        for y in range(5,40,5):
            pos.append((x,y))

    for coord in pos:
            dd = Point(coord[0], coord[1]).buffer(1)
            rrwe = PolygonPatch(dd, facecolor='#8A2BE2', edgecolor='#8A2BE2', alpha=0.9, zorder=2)
            ax.add_patch(rrwe)
            polygons.append(dd)
    
    ax.axis([0, size_field, 0, size_field], 'equal')   
    
    return polygons



def init_map_old(size_field, ax, radius):

    
    return []


def init_pos_goal(n,size_field):
    pos = []
    goal = []

    # generate start and end positions
    for x in range(5,35,6):
        for y in range(2,18,4):
            pos.append((x,y))
            goal.append((40-x,40-y))
            pos.append((40-x,40-y))
            goal.append((x,y))
        
    #     for i in range(n):  
    #         x = np.random.uniform(low=(size_field-25), high =(size_field))
    #         y = 12
    #         pos.append((x,y))        
    #     for i in range(n):  
    #         x = np.random.uniform(low=(0), high =(5))
    #         y = 5
    #         pos.append((x,y))

        

    #random.shuffle(pos)
    return pos,goal


def globalset():
    n = 40 # Nr of agents
    size_field = 40
    max_velocity = 6##0.5 these works for smaller radiuses, also produces the dancing thingy mentioned in the paper
    max_acceleration = 2##0.5
    dv = 0.05#0.1 # Step size when looking for new velocities
    t = 1 # timestep I guess
    simulation_time = 400
    radius = 0.5
    return [n,size_field,max_velocity,max_acceleration,dv,t,simulation_time,radius]
