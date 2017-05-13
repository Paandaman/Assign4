import numpy as np
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import MultiPolygon, Point
from scipy import interpolate
from matplotlib.pyplot import quiver

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
            print("Does this ever Happen? ")
            print("\n d:", d)
            print("\n s:", s)
            print("\n ############################################################################N")
            v_i_star = v_i_star*(d/s)
            print("\n v-Istar:", v_i_star)
        return v_i_star


    def calculate_velocity_obstacles(self, list_of_agents):
        velocity_objects = []
        for agent in list_of_agents:
            if agent == self:
                #print("Skip this")
                continue
            print("self size", self.size)
            print("self size", agent.size)
            r = self.size + agent.size # enlarge radius
            print("aded size", r)
            tp1, tp2 = self.find_tangents(self.pos, agent.pos, r)
            #global tp11, tp22, pos11
            # translate CC by agents vel
            print("agentvel", agent.vel)
            agentvel = [agent.vel[0]*dv, agent.vel[1]*dv] #### CHANGE HERE FROM ONLY USING  agent.vel below, seems better
            tp11 = np.add(tp1,agentvel)
            tp22 = np.add(tp2, agentvel)
            pos11 = np.add(self.pos, agentvel)
            #tp11, tp22 = self.extrapolate_cone(pos11, tp11, tp22, self.pos)
            velocity_object = Polygon([pos11, tp22, tp11])
            velocity_objects.append(velocity_object)
        self.velocity_objects = velocity_objects # Use this later
        return velocity_objects



    def find_tangents(self, pos_i, pos_j, r):
        # finds tangents used to calculate collision cone, r= radius of agent j + r of agent i
        d = np.sqrt(np.sum(np.square(np.subtract(pos_i, pos_j))))
        if r > d: # To prevent imaginary values # WOuldn't want this to happen but I guess it's ok since r is extended
            print("\n how often",r) # This does not seem to work very well atm, In the paper they mention this special case
            r = d   # Becomes NaN if we don't use this!
            print("\noch sen",r)
        alpha = np.arcsin(r/d)
        print("u", alpha)
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
        # NOT TESTED!!!!
        dv_i = np.subtract(v_star, self.vel)
        #print("DV_I :", dv_i)
        #eps = np.random.normal(0, 0.001, 1)[0]
        #dv_i = dv_i+eps # ATTEMPT TO HANDLE ZERO DIVI, didn't make much difference..
        nd_vi = dv*dv_i/(max_acceleration*t) ######## WTTFFFFFFF # this can probably grow large
        #print("nd_vi", nd_vi)
        new_velocity = [0,0]
        for degree in range(0,181): 
            theta = degree * 0.0174533 # in radians
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


def main():
    n = 2 # Nr of agents
    global dv, size_field, max_velocity, max_acceleration, t # change this later
    size_field = 10
    max_velocity = 2##0.5 these works for smaller radiuses, also produces the dancing thingy mentioned in the paper
    max_acceleration = 2##0.5
    dv = 0.1#0.1 # Step size when looking for new velocities
    t = 1 # timestep I guess
    simulation_time = 200
    radius = 1


    agentA = Agents([1,1], [0.5, 0.5], [8,8], radius,'r') # position, velocity, goal, radius, color
    agentB = Agents([8,8], [1, -0.5], [1,1], radius, 'b')
    agentC = Agents([1,8], [1, 2], [8,1], radius, 'y')
    agentD = Agents([8,1], [-1, 2], [1,8], radius, 'g')
    agentF = Agents([1, 5], [1, 2], [8, 5], radius, 'b')
    agentG = Agents([8, 5], [-1, 2], [1, 5], radius, 'b')
    agents = [agentA, agentB, agentC, agentD, agentF, agentG]

    fig, ax = plt.subplots() # Fig ska man aldrig bry sig om om man inte vill ändra själva plotrutan
    time = 0
    while time < simulation_time:
        for agent in agents:
            VOs = agent.calculate_velocity_obstacles(agents)
            preffered_vel = agent.find_preffered_velocity()
            #print("preffered velocity", preffered_vel)
            new_velocity = agent.avoidance_strategy(preffered_vel)
            if agent == agentA:
                diff_x = new_velocity[0]*t 
                diff_y = new_velocity[1]*t
                print(time)
                #print("NEW X", diff_x)
                #print("NEW Y", diff_y)
                print("\n")
            agent.pos = [agent.pos[0]+new_velocity[0]*t, agent.pos[1]+new_velocity[1]*t]
            agent.vel = new_velocity

            #velocity_ob = agentA.calculate_velocity_obstacles([agentB])
        if time==0:
            anim1, anim2, anim3, anim4, anim5, anim6 = ax.plot(agentA.pos[0],agentA.pos[1],'go',agentB.pos[0],agentB.pos[1],'bo', agentC.pos[0],agentC.pos[1],'ro', agentD.pos[0],agentD.pos[1],'yo', agentF.pos[0],agentF.pos[1],'yo', agentG.pos[0],agentG.pos[1],'yo')
            patch = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch)
            patch1a = PolygonPatch(agentA.velocity_objects[1], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch1a)
            patch2 = PolygonPatch(agentA.velocity_objects[2], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch2)
            patch3a = PolygonPatch(agentA.velocity_objects[3], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch3a)
            patch4 = PolygonPatch(agentA.velocity_objects[4], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch4)
            # patch2 = PolygonPatch(agentB.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch2)
            # patch3 = PolygonPatch(agentC.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch3)
            # patch4 = PolygonPatch(agentD.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch4)

            ax.axis([-0, size_field, -0, size_field], 'equal')
            f = ax.add_artist(agentA.shape)
            f2 = ax.add_artist(agentB.shape)
            f3 = ax.add_artist(agentC.shape)
            f4 = ax.add_artist(agentD.shape)
            f5 = ax.add_artist(agentF.shape)
            f6 = ax.add_artist(agentG.shape)
            vel1 = ax.quiver(agentB.pos[0], agentB.pos[1], agentB.vel[0], agentB.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel2 = ax.quiver(agentA.pos[0], agentA.pos[1], agentA.vel[0], agentA.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel3 = ax.quiver(agentC.pos[0], agentC.pos[1], agentC.vel[0], agentC.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel4 = ax.quiver(agentD.pos[0], agentD.pos[1], agentD.vel[0], agentD.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
        else:
            anim1.set_data(agentA.pos[0],agentA.pos[1])
            anim2.set_data(agentB.pos[0], agentB.pos[1])
            anim3.set_data(agentC.pos[0], agentC.pos[1])
            anim4.set_data(agentD.pos[0], agentD.pos[1])
            anim5.set_data(agentF.pos[0], agentF.pos[1])
            anim6.set_data(agentG.pos[0], agentG.pos[1])

            #velocity_object = Polygon([pos11, tp22, tp11])
            #s11 = patch.get_path()
            #print(s11)
            #patch.bounds= (ps11, tp22, tp11)# = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            #print(type(patch))
            patch.remove()
            patch = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch)
            patch1a.remove()
            patch1a = PolygonPatch(agentA.velocity_objects[1], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch1a)
            patch2.remove()
            patch2 = PolygonPatch(agentA.velocity_objects[2], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch2)
            patch3a.remove()
            patch3a = PolygonPatch(agentA.velocity_objects[3], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch3a)
            patch4.remove()
            patch4 = PolygonPatch(agentA.velocity_objects[4], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch4)
            # patch2.remove()
            # patch2 = PolygonPatch(agentB.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch2)
            # patch3.remove()
            # patch3 = PolygonPatch(agentC.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch3)
            # patch4.remove()
            # patch4 = PolygonPatch(agentD.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            # ax.add_patch(patch4)
            f.center= agentA.pos[0],agentA.pos[1]
            f2.center = agentB.pos[0], agentB.pos[1]
            f3.center = agentC.pos[0], agentC.pos[1]
            f4.center = agentD.pos[0], agentD.pos[1]
            f5.center = agentF.pos[0], agentF.pos[1]
            f6.center = agentG.pos[0], agentG.pos[1]
            vel1.remove()
            vel2.remove()
            vel3.remove()
            vel4.remove()
            vel1 = ax.quiver(agentB.pos[0], agentB.pos[1], agentB.vel[0], agentB.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel2 = ax.quiver(agentA.pos[0], agentA.pos[1], agentA.vel[0], agentA.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel3 = ax.quiver(agentC.pos[0], agentC.pos[1], agentC.vel[0], agentC.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            vel4 = ax.quiver(agentD.pos[0], agentD.pos[1], agentD.vel[0], agentD.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            #f = ax.add_artist(agentA.shape)
            #f2 = ax.add_artist(agentB.shape)
            #ax.quiver(agentB.pos[0], agentB.pos[1], agentB.vel[0], agentB.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            #ax.quiver(agentA.pos[0], agentA.pos[1], agentA.vel[0], agentA.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            ax.plot(agentA.goal[0], agentA.goal[1],'r*')
            ax.plot(agentB.goal[0], agentB.goal[1],'g*')
            ax.plot(agentC.goal[0], agentC.goal[1],'b*')
            ax.plot(agentD.goal[0], agentD.goal[1],'y*')
            ax.plot(agentF.goal[0], agentF.goal[1],'b*')
            ax.plot(agentG.goal[0], agentG.goal[1],'b*')


        time += 1
        plt.pause(0.0000000000001)
   

main()