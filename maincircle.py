import numpy as np
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import MultiPolygon, Point
from scipy import interpolate
from matplotlib.pyplot import quiver
import math
import random

class Agents:

    def __init__(self,pos, vel, goal, size, colorr):
        self.pos = pos
        self.vel = vel
        self.goal = goal
        self.size = size # radius of circle
        self.shape = plt.Circle((pos), radius=size, color=colorr) # CHANGED RAIDUS HERE FROM SIMPLY HAVING NUMERICAL STUFF
        self.velocity_objects = []

    def find_preffered_velocity(self):
        v_i_direction = np.subtract(self.goal, self.pos)
        #print("WHAT DIS SHIT BE", v_i_direction)
        v_i_star = v_i_direction*max_velocity/(np.sqrt(np.sum(np.square(v_i_direction))))
        #print("STTARRYY",v_i_star)
        s = 0.5*max_acceleration*t*t
        d = np.sqrt(np.sum(np.square(np.subtract(self.goal, self.pos))))
        if d < s: # decrease velocity if we approach goal
            #print("Does this ever Happen? ")
            #print("\n d:", d)
            #print("\n s:", s)
            #print("\n ############################################################################N")
            v_i_star = v_i_star*(d/s)
            #print("\n v-Istar:", v_i_star)
        return v_i_star


    def calculate_velocity_obstacles(self, list_of_agents):#, boundary_polygons):
        velocity_objects = []
        for agent in list_of_agents:
            if agent == self:
                #print("Skip this")
                continue

            # Check if within neighbourhood, seems to be working really well!
            distance = np.sqrt(np.sum(np.square(np.subtract(self.pos, agent.pos))))
            neighbourhood_radius = self.size*8# funny one 15
            if distance > neighbourhood_radius: # Ignore this obstacle!
                continue

            safety_padding = 0.2
            r = self.size + agent.size+safety_padding # enlarge radius. safety padding is in order to avoid agents touching resulting in breakdown of arcsin..
            #print("aded size", r)
            tp1, tp2 = self.find_tangents(self.pos, agent.pos, r)
            #global tp11, tp22, pos11
            # translate CC by agents vel
            #print("agentvel", agent.vel)
            #agentvel = [agent.vel[0]*dv, agent.vel[1]*dv] # for only VO 
            agentvel = [(agent.vel[0]+self.vel[0])*dv*0.5, (agent.vel[1]+self.vel[1])*dv*0.5] #### CHANGE HERE FROM ONLY USING  agent.vel below, seems better
            # Now, with reciprocal features!
            tp11 = np.add(tp1,agentvel)
            tp22 = np.add(tp2, agentvel)
            pos11 = np.add(self.pos, agentvel)
            tp11, tp22 = self.extrapolate_cone(pos11, tp11, tp22, self.pos) # might actually work pretty well
            velocity_object = Polygon([pos11, tp22, tp11])
            velocity_objects.append(velocity_object)

        self.velocity_objects = velocity_objects # Use this later
        # Add map obstacles    
        #for polygon in boundary_polygons:
            # add padding to boundaries
        #    self.velocity_objects.append(polygon)
        return velocity_objects



    def find_tangents(self, pos_i, pos_j, r):
        # finds tangents used to calculate collision cone, r= radius of agent j + r of agent i
        d = np.sqrt(np.sum(np.square(np.subtract(pos_i, pos_j))))
        if r > d: # To prevent imaginary values # WOuldn't want this to happen but I guess it's ok since r is extended
            #print("\n how often",r) # This does not seem to work very well atm, In the paper they mention this special case
            r = d-0.1   # Becomes NaN if we don't use this!
            print("SHIT'S ABOUT TO BREAK DOWN")
            #print("THIS IS INDEED WHAT CAUSES COLLISSIONS!!")
            #print("\noch sen",r)

        alpha = np.arcsin(r/d)
        #print("u", alpha)
        gamma = 0.5*np.pi-alpha
        offseta = np.arctan2((pos_i[1]-pos_j[1]),(pos_i[0]-pos_j[0])) # arctan2 is a special version of arctan, takes (y, x) as input
        #print("alpha:", alpha, "gamma", gamma, "offseta", offseta)
        theta1 = offseta - gamma
        theta2 = offseta + gamma
        #print(np.degrees(theta1), np.degrees(theta2)) # should be in radians for below computations though
        result_Y1 = pos_j[1] + r*np.sin(theta1)
        result_X1 = pos_j[0] + r*np.cos(theta1)
        tp1 = (result_X1, result_Y1)
        result_Y2 = pos_j[1] + r*np.sin(theta2)
        result_X2 = pos_j[0] + r*np.cos(theta2)
        tp2 = (result_X2, result_Y2)
        return tp1, tp2

    def extrapolate_cone(self, agent_pos, tp1, tp2, self_pos):
        # Do this in order to expand cone to cover entire map, probably unnecessary, 
        # especially if we restrict velocities but looks better when plotting
        # interp can extrapolate as well. also keeps track of how to extrapolate
        eps = np.random.normal(0, 0.001, 1) # add small noise to avoid division by zero
        if self_pos[0] < tp1[0]:
            x1 = tp1[0]+size_field
            f1 = interpolate.interp1d([agent_pos[0]+eps, tp1[0]], [agent_pos[1], tp1[1]],fill_value="extrapolate")
            y1 = f1(x1)#interp([x1], [agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]])
            tp_extr1 = [x1, y1]
        else:
            x1 = tp1[0]-size_field
            f1 = interpolate.interp1d([agent_pos[0]+eps, tp1[0]], [agent_pos[1], tp1[1]],fill_value="extrapolate")
            y1 = f1(x1)#interp([x1], [agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]])
            tp_extr1 = [x1, y1]
        if self_pos[0] < tp2[0]:
            x2 = tp2[0]+size_field
            f2 = interpolate.interp1d([agent_pos[0]+eps, tp2[0]], [agent_pos[1], tp2[1]],fill_value="extrapolate")
            y2 = f2(x2)#interp([x2], [agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]])
            tp_extr2 = [x2, y2]
        else:
            x2 = tp2[0]-size_field
            f2 = interpolate.interp1d([agent_pos[0]+eps, tp2[0]], [agent_pos[1], tp2[1]],fill_value="extrapolate")
            y2 = f2(x2)#interp([x2], [agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]])
            tp_extr2 = [x2, y2]

        return tp_extr1, tp_extr2

    def avoidance_strategy(self, v_star):
        # Finds a velocity which does not lead to a collision
        dv_i = np.subtract(v_star, self.vel)
        #print("DV_I :", dv_i)
        #eps = np.random.normal(0, 0.001, 1)[0]
        #dv_i = dv_i+eps # ATTEMPT TO HANDLE ZERO DIVI, didn't make much difference..
        nd_vi = dv*dv_i/(max_acceleration*t) ######## WTTFFFFFFF # this can probably grow large
        #print("nd_vi", nd_vi)
        new_velocity = [0,0]
        for degree in range(0,46):#181): INCREASED THIS TO Make the agents avoid each other more
            theta = 4*degree * 0.0174533 # in radians
            c, s = np.cos(theta), np.sin(theta)
            rotMatrix = np.matrix([[c, -s], [s, c]])
            #rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
            #print("Innan rotation1", nd_vi)
            md_vi = rotMatrix.dot(nd_vi).tolist()[0] # rotated by theta radians
            #print("efter rotation1", md_vi) 
            #change_vel = np.add(md_vi,self.vel) # Kanske inte ska vara här!! DETTA VAR FETT FEL!!!
            ## Need to translate point to current position ## CHANGE HERE
            change_vel = np.add(md_vi, self.pos)
            if not self.collision_point(change_vel):
                new_velocity = md_vi#change_vel
                break
            theta = -theta
            c, s = np.cos(theta), np.sin(theta)
            rotMatrix = np.matrix([[c, -s], [s, c]])
            #print("Innan rotation1", md_vi)
            md_vi = rotMatrix.dot(nd_vi).tolist()[0] # rotated by -theta radians
            #print("efter rotation1", md_vi) 
            #change_vel = np.add(md_vi,self.vel)
            change_vel = np.add(md_vi, self.pos)
            if not self.collision_point(change_vel):
                new_velocity = md_vi#change_vel
                break

        if new_velocity == [0,0]:
            print("sdsdsdsdsdsd")
        test_change_vel = np.add(new_velocity,self.vel)#new_velocity#np.add(md_vi,self.vel)
        new_velocity_magn = np.sqrt(np.sum(np.square(test_change_vel)))
        if new_velocity_magn > max_velocity:
            #print("HUUr", new_velocity, new_velocity_magn)
            v_temp = test_change_vel#new_velocity
            #print("NEGER", v_temp, max_velocity, new_velocity_magn)
            v_temp = [v_temp[0]*(max_velocity/new_velocity_magn), v_temp[1]*(max_velocity/new_velocity_magn)]
            #print("AAAAAAA", v_temp, self.vel)
            new_velocity = np.subtract(v_temp, self.vel)
            new_velocity = [new_velocity[0]*dv, new_velocity[1]*dv]
        #print("FINAL NEW VELOCITY", new_velocity)
        return new_velocity


    def collision_point(self,p):
        conv_to_point = Point(p[0], p[1])
        collision_found = False
        for velocity_object in self.velocity_objects: 
            if velocity_object.contains(conv_to_point):
                collision_found = True
                #print("COLLISION FOUND", conv_to_point) 
                return collision_found

        return collision_found


    def rotate(self,velocity, angle):
        # translates velocity vector to origin, rotates it
        # then moves it back to it's previous position
        # might not be needed??? we might be using the velocities from a local coord. syst
        transl_to_origin = [velocity[0]-self.pos[0], velocity[1]-self.pos[1]] 
        rotMatrix = array([[cos(angle), -sin(angle)], [sin(angle),  cos(angle)]])

def create_agents(n, radius, pos, goals):
    agents = []
    for i in range(n):
        #theta = 2*math.pi*random.random()
        #r = 10
        #x = 10 + r * math.cos(theta) 
        #y = 10 + r * math.sin(theta)
        vx = np.random.uniform(low=-max_velocity, high=max_velocity)
        vy = np.random.uniform(low=-max_velocity, high=max_velocity)
        #theta = 2*math.pi*random.random()
        #xgoal = 10 + r * math.cos(theta)
        #ygoal = 10 + r * math.sin(theta)
        posit = pos.pop()
        goa = goals.pop()
        agent = Agents([posit[0],posit[1]], [vx, vy], [goa[0],goa[1]], radius,'r')
        agents.append(agent)
    return agents

def create_fake_agents(n, radius, pos, goals):
    agents = []
    for posi in pos:
        #theta = 2*math.pi*random.random()
        #r = 10
        #x = 10 + r * math.cos(theta) 
        #y = 10 + r * math.sin(theta)
        vx = 0#np.random.uniform(low=-max_velocity, high=max_velocity)
        vy = 0#np.random.uniform(low=-max_velocity, high=max_velocity)
        #theta = 2*math.pi*random.random()
        #xgoal = 10 + r * math.cos(theta)
        #ygoal = 10 + r * math.sin(theta)
        
        posit = posi.centroid

        print("KUKEN",posi.centroid.y)
        agent = Agents([posit.x, posit.y], [vx, vy], [posit.x, posit.y], radius,'r')
        agents.append(agent)
    return agents

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



def init_map_old(size_field, ax, radius):

    lower_wallp = Polygon([(0,0), (0,3),(size_field,3),(size_field,0)])
    lower_wall = PolygonPatch(lower_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
    ax.add_patch(lower_wall)

    left_wallp = Polygon([(0,10), (0,size_field),(15,size_field), (15,10)])
    left_wall = PolygonPatch(left_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
    ax.add_patch(left_wall)

    right_wallp = Polygon([(25,10), (25,size_field),(size_field,size_field), (size_field,10)])
    right_wall = PolygonPatch(right_wallp, facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
    ax.add_patch(right_wall)

    ax.axis([0, size_field, 0, size_field], 'equal')


    polygons = padd_walls(lower_wallp, left_wallp, right_wallp, radius)
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

def rotate(l, n): 
    # cyclic permutation of positions
    return l[n:] + l[:n]

def main():
    n = 12 # Nr of agents
    global dv, size_field, max_velocity, max_acceleration, t # change this later
    size_field = 40
    max_velocity = 6##0.5 these works for smaller radiuses, also produces the dancing thingy mentioned in the paper
    max_acceleration = 2##0.5
    dv = 0.05#0.1 # Step size when looking for new velocities
    t = 1 # timestep I guess
    simulation_time = 170
    radius = 1

    pos = []
    goal = []

    # generate start and end positions
    # for t in range(n):
    #     for i in range(n):
    #         x = np.random.uniform(low=(size_field/2-5), high =(size_field/2+5))
    #         y = size_field+10
    #         goal.append((x,y))

    pos = []
    # for x in range(5,35,3):
    #     if x ==5:
    #         pos.append((x-2.5,2))
    #     else:
    #         pos.append((x-2.5,2))
    #         pos.append((2.5,x-2.5))
        
    for degree in range(0,360,30):#181): INCREASED THIS TO Make the agents avoid each other more
        theta = degree * 0.0174533
        r = 15
        x = 16 + r * math.cos(theta) 
        y = 16 + r * math.sin(theta)
        #vx = np.random.uniform(low=-max_velocity, high=max_velocity)
        #vy = np.random.uniform(low=-max_velocity, high=max_velocity)
        pos.append((x,y))



    #random.shuffle(pos)
    goal = rotate(pos, 6)
    
    #goal = pos[::-1]
    agents = create_agents(n, radius, pos, goal)

    fig, ax = plt.subplots() # Fig ska man aldrig bry sig om om man inte vill ändra själva plotrutan
    #boundary_polygons = init_map(size_field, ax, radius)
    # Consider the obstacles as agents with zero velocity! Don't consider these when updating velocities etc
    #obstacle_agents = create_fake_agents(len(boundary_polygons), radius, boundary_polygons, 1)
    time = 0
    #avoid = [agents+obstacle_agents] # want to send in both agents and obstacles when creating VOs
    save_trajectories = [[] for i in range(n)]
    k = 0
    while time < simulation_time:
        for agent in agents:
            VOs = agent.calculate_velocity_obstacles(agents)#, boundary_polygons)
            preffered_vel = agent.find_preffered_velocity()
            #print("preffered velocity", preffered_vel)
            new_velocity = agent.avoidance_strategy(preffered_vel)
            agent.pos = [agent.pos[0]+new_velocity[0]*t, agent.pos[1]+new_velocity[1]*t]
            agent.vel = new_velocity
            save_trajectories[k].append(agent.pos) # Keep going from here!
            k += 1

        k = 0

        if time==0:
            anims = []
            patches = []
            fs = []
            for agent in agents:
                dd = ax.plot(agent.pos[0],agent.pos[1],'go')
                anims.append(dd)
                oo = ax.add_artist(agent.shape)
                fs.append(oo)
            
            ax.axis([-0, size_field, -0, size_field], 'equal')
        else:
            for (agent,anim, f) in zip(agents,anims,fs):
                anim[0].set_data(agent.pos[0],agent.pos[1])
                f.center= agent.pos[0],agent.pos[1] 
                ax.plot(agent.goal[0], agent.goal[1],'r*')


        time += 1
        print("time:",time)
        plt.pause(0.1)
        if time == 1:
            ddd = input("Tryck")
   
    #print(save_trajectories[0])
    fig, ax = plt.subplots()
    colors = ['r-','b-','g-','c-','m-','y-','k-','w*','b*','g*','r*','c*','m*','y*','k*','w*']

    for agent in save_trajectories:
        color = colors.pop()
        x = []
        y = []
        for spot in agent:
            x.append(spot[0])#ax.plot(spot[0],spot[1])#,color)
            y.append(spot[1])
        ax.plot(x,y)#,color)
    plt.show()

main()