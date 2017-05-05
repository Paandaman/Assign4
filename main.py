import numpy as np
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import MultiPolygon, Point
from scipy import interpolate
from matplotlib.pyplot import quiver

class Agents:

    def __init__(self,pos, vel, goal, size):
        self.pos = pos
        self.vel = vel
        self.goal = goal
        self.size = size # radius of circle
        self.shape = plt.Circle((pos), size, color='r')
        self.velocity_objects = []

    def find_preffered_velocity(self):
        v_i_direction = np.subtract(self.goal, self.pos)
        v_i_star = v_i_direction*max_velocity/(np.sqrt(np.sum(np.square(v_i_direction))))
        s = 0.5*max_acceleration*t*t
        d = np.sqrt(np.sum(np.square(np.subtract(self.goal, self.pos))))
        if d < s: # decrease velocity if we approach goal
            v_i_star = v_i_star*(d/s)
        return v_i_star


    def calculate_velocity_obstacles(self, list_of_agents):
        velocity_objects = []
        for agent in list_of_agents:
            if agent == self:
                print("KERSTIN")
                continue
            r = self.size + agent.size # enlarge radius
            tp1, tp2 = self.find_tangents(self.pos, agent.pos, r)
            global tp11, tp22, pos11
            # translate CC by agents vel
            tp11 = np.add(tp1,agent.vel)
            tp22 = np.add(tp2, agent.vel)
            pos11 = np.add(self.pos, agent.vel)
            tp11, tp22 = self.extrapolate_cone(pos11, tp11, tp22, self.pos)
            velocity_object = Polygon([pos11, tp22, tp11])
            velocity_objects.append(velocity_object)
        self.velocity_objects = velocity_objects # Use this later
        return velocity_objects



    def find_tangents(self, pos_i, pos_j, r):
        # finds tangents used to calculate collision cone, r= radius of agent j
        d = np.sqrt(np.sum(np.square(np.subtract(pos_i, pos_j))))
        if r > d: # To prevent imaginary values
            r = d
        alpha = np.arcsin(r/d)
        gamma = 0.5*np.pi-alpha
        offseta = np.arctan2((pos_i[1]-pos_j[1]),(pos_i[0]-pos_j[0])) # arctan2 is a special version of arctan, takes (y, x) as input
        print("alpha:", alpha, "gamma", gamma, "offseta", offseta)
        theta1 = offseta - gamma
        theta2 = offseta + gamma
        print(np.degrees(theta1), np.degrees(theta2)) # should be in radians for below computations though
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
        # interp can extrapolate as well, also keeps track of how to extrapolate
        if self_pos[0] < tp1[0]:
            x1 = tp1[0]+size_field
            f1 = interpolate.interp1d([agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]],fill_value="extrapolate")
            y1 = f1(x1)#interp([x1], [agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]])
            tp_extr1 = [x1, y1]
        else:
            x1 = tp1[0]-size_field
            f1 = interpolate.interp1d([agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]],fill_value="extrapolate")
            y1 = f1(x1)#interp([x1], [agent_pos[0], tp1[0]], [agent_pos[1], tp1[1]])
            tp_extr1 = [x1, y1]
        if self_pos[0] < tp2[0]:
            x2 = tp2[0]+size_field
            f2 = interpolate.interp1d([agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]],fill_value="extrapolate")
            y2 = f2(x2)#interp([x2], [agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]])
            tp_extr2 = [x2, y2]
        else:
            x2 = tp2[0]-size_field
            f2 = interpolate.interp1d([agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]],fill_value="extrapolate")
            y2 = f2(x2)#interp([x2], [agent_pos[0], tp2[0]], [agent_pos[1], tp2[1]])
            tp_extr2 = [x2, y2]

        return tp_extr1, tp_extr2

    def avoidance_strategy(self, v_star):
        # Finds a velocity which does not lead to a collision
        # NOT TESTED!!!!
        dv_i = np.subtract(v_star, self.vel)
        nd_vi = dv*max_acceleration*t/dv_i
        new_velocity = [0,0]
        for degree in range(0,181): 
            theta = degree * 0.0174533 # in radians
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
            md_vi = rotMatrix.dot(nd_vi) # rotated by theta radians 
            change_vel = np.add(md_vi,self.vel)
            if not self.collision_point(change_vel):
                new_velocity = change_vel
                break

            theta = -theta
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
            md_vi = rotMatrix.dot(nd_vi) # rotated by -theta radians 
            change_vel = np.add(md_vi,self.vel)
            if not self.collision_point(change_vel):
                new_velocity = change_vel
                break

        new_velocity_magn = np.sqrt(np.sum(np.square(new_velocity)))
        if new_velocity_magn > max_velocity:
            v_temp = new_velocity
            v_temp = v_temp*max_velocity/new_velocity_magn
            new_velocity = np.subtract(v_temp, self.vel)
        print("FINAL NEW VELOCITY", new_velocity)
        return new_velocity


    def collision_point(self,p):
        conv_to_point = Point(p[0], p[1])
        collision_found = False
        for velocity_object in self.velocity_objects: 
            if velocity_object.contains(conv_to_point):
                collision_found = True
                print("COLLISION FOUND", conv_to_point) 
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
    size_field = 14
    max_velocity = 2
    max_acceleration = 1
    dv = 0.1 # Step size when looking for new velocities I think
    t = 1 # timestep I guess
    simulation_time = 30

    agentA = Agents([1,1], [0.5, 0.5], [8,8], 1) # position, velocity, goal, radius
    agentB = Agents([8,8], [1, -0.5], [1,1], 1)
    agents = [agentA, agentB]

    fig, ax = plt.subplots() # Fig ska man aldrig bry sig om om man inte vill ändra själva plotrutan
    time = 0
    while time < simulation_time:
        for agent in agents:
            VOs = agent.calculate_velocity_obstacles(agents)
            preffered_vel = agent.find_preffered_velocity()
            print("preffered velocity", preffered_vel)
            new_velocity = agent.avoidance_strategy(preffered_vel)
            agent.pos = [agent.pos[0]+new_velocity[0]*t, agent.pos[1]+new_velocity[1]*t]
            agent.vel = new_velocity

            #velocity_ob = agentA.calculate_velocity_obstacles([agentB])
        if time==0:
            anim1, anim2 = ax.plot(agentA.pos[0],agentA.pos[1],'go',agentB.pos[0],agentB.pos[1],'bo')
            patch = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch)
            patch2 = PolygonPatch(agentB.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            ax.add_patch(patch2)

            ax.axis([0, size_field, 0, size_field], 'equal')
            f = ax.add_artist(agentA.shape)
            f2 = ax.add_artist(agentB.shape)
            #ax.quiver(agentB.pos[0], agentB.pos[1], agentB.vel[0], agentB.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            #ax.quiver(agentA.pos[0], agentA.pos[1], agentA.vel[0], agentA.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
        else:
            anim1.set_data(agentA.pos[0],agentA.pos[1])
            anim2.set_data(agentB.pos[0], agentB.pos[1])

            #velocity_object = Polygon([pos11, tp22, tp11])
            #s11 = patch.get_path()
            #print(s11)
            #patch.bounds= (ps11, tp22, tp11)# = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            #print(type(patch))
            patch.remove()
            patch = PolygonPatch(agentA.velocity_objects[0], facecolor='#ff3333', edgecolor='#6699cc', alpha=0.5, zorder=2)
            ax.add_patch(patch)
            patch2.remove()
            patch2 = PolygonPatch(agentB.velocity_objects[0], facecolor='#FFFF00', edgecolor='#FFFF00', alpha=0.5, zorder=2)
            ax.add_patch(patch2)
            f.center= agentA.pos[0],agentA.pos[1]
            f2.center = agentB.pos[0], agentB.pos[1]
            #f = ax.add_artist(agentA.shape)
            #f2 = ax.add_artist(agentB.shape)
            #ax.quiver(agentB.pos[0], agentB.pos[1], agentB.vel[0], agentB.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector
            #ax.quiver(agentA.pos[0], agentA.pos[1], agentA.vel[0], agentA.vel[1], scale=6, scale_units ='width') ##in case we want velocity vector


        time += 1
        plt.pause(0.5)

main()