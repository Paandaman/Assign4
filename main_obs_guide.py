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


from settings_base import*

class Agents:

    def __init__(self,pos, vel, goal, size, colorr):
        self.final=goal
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


    def calculate_velocity_obstacles(self, list_of_agents, boundary_polygons):
        velocity_objects = []
        for agent in list_of_agents:
            if agent == self:
                #print("Skip this")
                continue

            # Check if within neighbourhood, seems to be working really well!
            distance = np.sqrt(np.sum(np.square(np.subtract(self.pos, agent.pos))))
            neighbourhood_radius = self.size*5
            if distance > neighbourhood_radius: # Ignore this obstacle!
                continue

            safety_padding = 0.1
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
        for polygon in boundary_polygons: # THis is unnecessary now, double check
            # add padding to boundaries
            self.velocity_objects.append(polygon)
        return velocity_objects



    def find_tangents(self, pos_i, pos_j, r):
        # finds tangents used to calculate collision cone, r= radius of agent j + r of agent i
        d = np.sqrt(np.sum(np.square(np.subtract(pos_i, pos_j))))
        if r > d: # To prevent imaginary values # WOuldn't want this to happen but I guess it's ok since r is extended
            #print("\n how often",r) # This does not seem to work very well atm, In the paper they mention this special case
            r = d-0.1   # Becomes NaN if we don't use this!
            print("SHIT'S ABOUT TO BREAK DOWN") # appearently this was important
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
class node:
    def __init__(self,point):
        self.pos=point
        self.neb=[]
        self.weight=0
        #self.state=-1  # -1 not visited, 1 visited
        self.F=np.Inf
        self.G=np.Inf
        self.father=-1

    def addneb(self,nod):
        self.neb.append(nod)



def showedage(n1,n2,c):
    prit=[n1.pos,n2.pos]
    prit=np.array(prit).T
    plt.plot(prit[0],prit[1],c)

def makepath(A,B):
    dist=distance(A,B)
    total=int(dist/0.1)
    x=np.linspace(A[0],B[0],total)
    y=np.linspace(A[1],B[1],total)
    xy=np.concatenate([[x],[y]]).T
    xy=xy.tolist()
    return xy

class Guid:
    def __init__(self,mapsize,polygons):
        self.polys=polygons
        self.size=mapsize
        self.N=int(self.size**2/10)
        self.points=[]
        self.nodes=[]
        i=0
        while i<self.N:
            point=[np.random.uniform(0,self.size),np.random.uniform(0,self.size)]
            if(self.collision(point)):
                continue
            self.points.append(point)
            new_node=node(point)
            self.nodes.append(new_node)
            #plt.plot(point[0],point[1],'y+')
            i=i+1

        pset=np.array(self.points)
        tri=Delaunay(pset)
        (indices, indptr) = tri.vertex_neighbor_vertices
        for i in range(self.N):
            neblist=indptr[indices[i]:indices[i+1]]
            for id in neblist:

                path=makepath(self.points[i],self.points[id])
                coll=0
                for pt in path:
                    if(self.collision(pt)):
                        coll=1
                if(coll==1):
                    continue

                self.nodes[i].addneb(id)
                # prit=[self.nodes[i].pos,self.nodes[id].pos]
                # prit=np.array(prit).T
                # plt.plot(prit[0],prit[1],'y-')

    def collision(self,p):
        conv_to_point = Point(p[0], p[1])
        collision_found = False
        for velocity_object in self.polys: 
            if velocity_object.contains(conv_to_point):
                collision_found = True
                #print("COLLISION FOUND", conv_to_point) 
                return collision_found

        return collision_found



    def update(self,agents,dis):
        aglist=[]
        for agent in agents:
            pos=agent.pos
            aglist.append(pos)
        treedata=np.asarray(aglist)
        tree=KDTree(treedata)
        for nd in self.nodes:
            avail = tree.query_ball_point(nd.pos,dis)
            nd.weight=len(avail)
            #print(nd.weight)


    def score(self,n1,n2,final):
        G=n1.G+distance(n1.pos,n2.pos)*(n2.weight+1)
        H=distance(n2.pos,final)*1.2
        return G,G+H




    def astar(self,agent):
        node_star=deepcopy(self.nodes)
        pos=agent.pos
        mindis=np.Inf
        st=0
        ed=0
        mined=np.Inf
        ed_pos=agent.final

        # print(agent.pos)
        # print(agent.final)

        for nd in node_star:
            #print(nd.pos)
            dis=distance(nd.pos,pos)
            if dis<mindis:
                st=nd
                mindis=dis
            eddis=distance(nd.pos,ed_pos)
            if(eddis<mined):
                ed=nd
                mined=eddis
        # print(ed.pos)
        # print(st.pos)
        # plt.plot(ed.pos[0],ed.pos[1],'b+')
        # plt.plot(st.pos[0],st.pos[1],'r+')

        # print(node_star.index(st))
        # print(node_star.index(ed))



        if(mindis>distance(agent.pos,agent.final)):
            return agent.final

        st.G=0

        thisnode=st
        


        openset=[]
        closeset=[]
        #print("astar!!")
        while(1):

            closeset.append(thisnode)
            if((ed in openset) or (ed in closeset)):
                break

            n_id=node_star.index(thisnode)
            #print(n_id)

            for nebid in thisnode.neb:
                nb=node_star[nebid]
                if (nb in closeset):
                    #print("1")
                    continue
                elif (nb in openset):
                    #print("2")
                    newG,newscore=self.score(thisnode,nb,ed.pos)
                    if(newscore<nb.F):
                        nb.F=newscore
                        nb.G=newG
                        nb.father=n_id

                else:
                    #print("3")
                    newG,newscore=self.score(thisnode,nb,ed.pos)
                    nb.F=newscore
                    nb.G=newG
                    nb.father=n_id
                    #print("open add!!", nb.pos,"F=:",nb.F)
                    openset.append(nb)


            minF=np.Inf;
            for nd in openset:
                if (nd.F<=minF):
                    thisnode=nd
                    minF=nd.F

            openset.remove(thisnode)
            #print("pickone!", thisnode.pos,"F=:",thisnode.F)
            if(len(openset)==0):
                break
        #print("traceback")

        ##traceback
        tc=[]
        
        while (node_star[ed.father]!=st ):
            if ed.father==-1:
                return agent.final

            #print(ed.father)
            #plt.plot(ed.pos[0],node_star.index(ed.father),'g+')
            
            #showedage(ed,node_star[ed.father],'r-')
            tc.append(ed)

            ed=node_star[ed.father]

        ltc=len(tc)
        if ltc<=4:
            target=ed
        else:

            target=tc[math.floor(ltc/4*3)]
        #target=ed


        return target.pos


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






def main():
    
    global dv, size_field, max_velocity, max_acceleration, t # change this later

    data=globalset()

    n = data[0]
    size_field = data[1]
    max_velocity = data[2]##0.5 these works for smaller radiuses, also produces the dancing thingy mentioned in the paper
    max_acceleration = data[3]##0.5
    dv = data[4]#0.1 # Step size when looking for new velocities
    t = data[5] # timestep I guess
    simulation_time = data[6]
    radius = data[7]


    pos,goal=init_pos_goal(n,size_field)
    
    #goals = pos[::-1]
    agents = create_agents(n, radius, pos, goal)
    #agentA = Agents([1,1], [0.5, 0.5], [8,8], radius,'r') # position, velocity, goal, radius, color
    #agentB = Agents([8,8], [1, -0.5], [1,1], radius, 'b')
    #agentC = Agents([1,8], [1, 2], [8,1], radius, 'y')
    #agentD = Agents([8,1], [-1, 2], [1,8], radius, 'g')
    #agentF = Agents([1, 5], [1, 2], [8, 5], radius, 'b')
    #agentG = Agents([8, 5], [-1, 2], [1, 5], radius, 'b')
    #agents = [agentA, agentB]#, agentC, agentD, agentF, agentG]
    

    fig, ax = plt.subplots() # Fig ska man aldrig bry sig om om man inte vill ändra själva plotrutan
    boundary_polygons = init_map_old(size_field, ax, radius)

    guid=Guid(size_field,boundary_polygons)


    # guid.update(agents,radius*2)
    # guid.astar(agents[0])
    # plt.show()

    # Consider the obstacles as agents with zero velocity! Don't consider these when updating velocities etc
    obstacle_agents = create_fake_agents(len(boundary_polygons), radius, boundary_polygons, 1)
    time = 0
    avoid = [agents+obstacle_agents] # want to send in both agents and obstacles when creating VOs
    save_trajectories = [[] for i in range(n)]
    k = 0
    while time < simulation_time:
        guid.update(agents,radius*2)
        for agent in agents:

            if (time>simulation_time/4*3):
                agent.goal=agent.final
            else:
                next=guid.astar(agent)
                agent.goal=next
            VOs = agent.calculate_velocity_obstacles(avoid[0], boundary_polygons)
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
            #goals= []
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
                #goal[0].set_data(agent.goal[0],agent.goal[1])
                f.center= agent.pos[0],agent.pos[1] 


                ax.plot(agent.final[0], agent.final[1],'r*')


        time += 1
        print("time:",time)
        plt.pause(0.1)




    for agent in save_trajectories:
        #color = colors.pop()
        x = []
        y = []
        for spot in agent:
            x.append(spot[0])#ax.plot(spot[0],spot[1])#,color)
            y.append(spot[1])
        ax.plot(x,y)#,color)
    plt.show()
   

main()