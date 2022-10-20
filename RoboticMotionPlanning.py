#!/usr/bin/env python

# A simple ROS subscriber node in Python

import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from PIL import Image, ImageDraw
import time
import matplotlib.pyplot as plt
import matplotlib.colors as clr

 # Creatting publisher to output velocity commands
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

dummy_count = 0
# visualisation parameters
zoom = 20
borders = 6
images = []
start_i, start_j = 1, 1
end_i, end_j = 29, 19
start = time.time()
view_map = np.zeros((40, 40))
view_map[start_i, start_j] = 0
view_map[end_i, end_j] = 0



class Subscriber:
    '''
    Class to subscribe and do the mapping and motion tasks
    '''

    def callback(self, data):
        '''
        Function to accept odometry data
        '''
        # Read position from odometry
        pose_msg = data.pose.pose
        # read x coordinate
        self.x_pos = pose_msg.position.x
        # read y coordinate
        self.y_pos = pose_msg.position.y
        if self.timestart:
            self.x_initial=pose_msg.position.x
            self.y_initial=pose_msg.position.y
            self.timestart=False
        # convert roll pitch and yaw as euler values from quaternion
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w], 'sxyz')
        # print the position data in the running shell
        #print("Pose: ({},{},{})".format(self.x_pos, self.y_pos, self.yaw))
        #rospy.loginfo_once(format(x_pos, y_pos, yaw))

        """
        if abs(x_pos - self.goal_position_x) <= 0.1 and abs(y_pos - self.goal_position_y) <= 0.1:
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.y = 0
            vel.angular.x = 0
            vel.angular.y = 0
            pub.publish(vel)
        else:

            # two cell in one meter
            x_cell = int(round(x_pos*2))
            y_cell = int(round(y_pos*2))
            for i in range(x_cell-2, x_cell+2):
                for j in range(y_cell-2, y_cell+2):
                    None

    def calculate_attract(self, x, y):
        self.scaling_factor = 10
        d = math.sqrt(pow((self.goal_position_x - x), 2) +
                      pow((self.goal_position_y - y), 2))
        force = self.scaling_factor*d
        """
    def ObstacleAvoidance(self,message):
        if self.target_achieved is False:
            
            if abs(self.x_pos-self.x_initial)>=10 or abs(self.y_pos-self.y_initial)>=10:
                self.vel.angular.z=0.5
                self.vel.linear.x=0.0
                pub.publish(self.vel)
            else:
                target_angle=np.arctan2(self.goal_position_y-self.y_pos,self.goal_position_x-self.x_pos)
                print("Target Angle - ", target_angle, "Current Yaw - ", self.yaw, )
                print( "Current position X-",self.x_pos," Y- ",self.y_pos)
                if abs(target_angle-self.yaw)>=0.5:
                    self.vel.linear.x=0.0
                    self.vel.angular.z=0.5
                    pub.publish(self.vel)
                else:
                    if message.ranges[0]>2:
                        self.vel.linear.x=0.5
                        self.vel.angular.z=0.0
                        pub.publish(self.vel)
                    else:
                        if message.ranges[0]<2:
                            self.vel.linear.x=0.0
                            self.vel.angular.z=0.5
                            pub.publish(self.vel)
            if abs(self.x_pos-self.goal_position_x)<=0.2 and abs(self.y_pos-self.goal_position_y)<=0.2:
                self.vel.linear.x=0.0
                self.vel.angular.z=0.0
                pub.publish(self.vel)
                self.target_achieved=True
                print("Target achieved")
                


    def MeasureDistance(self, msg):
        '''
        Function to read the lidar data
        '''
        # self.map_plot(msg)
        # read the distance in the front from the ranges list
        self.measuredDistance = msg.ranges[0]
        # print the distance in the shell
        # TODO:rospy.loginfo("Distance")
        # TODO:rospy.loginfo(self.measuredDistance)

        # implementing map function
        # self.map_plot(msg)
        self.ObstacleAvoidance(msg)

    def MoveRobot(self, location):
        
        direction_theta = math.atan2(location[1]-self.y_pos, location[0]-self.x_pos)
        #print(self.yaw, " yaw : theta ", direction_theta)
        #print("")
        print(self.yaw - direction_theta)
        global dummy_count
        dummy_count+=1
        if abs(self.yaw - direction_theta) >= 0.1:
            self.vel.angular.z = 0.5
            pub.publish(self.vel)
            while abs(self.yaw - direction_theta) > 0.1:
                dummy_count+=1
                if dummy_count%3000==0:
                    #print(self.yaw, " yaw : theta ", direction_theta)
                    continue
            self.vel.angular.z = 0.0
            pub.publish(self.vel)
        distance = np.sqrt(
            ((location[1]-self.y_pos)**2)+((location[0]-self.x_pos)**2))
        if abs(distance) <= 1:
            self.vel.linear.x = 0.1
            pub.publish(self.vel)
            while abs(distance) > 1:
                distance = np.sqrt(
                    ((location[1]-self.y_pos)**2)+((location[0]-self.x_pos)**2))
                #print(" distance : ", distance)
                continue
            self.vel.linear.x = 0.0
            pub.publish(self.vel)

    def potential_field(self):
        '''
        Function to implement potential field algorithm
        '''
        x = np.arange(-0, 50, 1)
        y = np.arange(-0, 50, 1)

        # Goal is at (40,40)
        goal = [40, 40]

        #obstacle is at(25,25)
        obstacle = [25, 25]
        X, Y = np.meshgrid(x, y)

        delx = np.zeros_like(X)
        dely = np.zeros_like(Y)

        """
            Inside the nested loop, distance from each point to the goal and ostacle is 
            calculated, Similarly angles are calculated. Then I simply used the formula give and 
            superimposed it to the Goals potential field.Also
            α = 50
            β = 50
            s = 15
            r = 2
        """
        s = 15
        r = 2
        for i in range(len(x)):
            for j in range(len(y)):

                # finding the goal distance and obstacle distance
                d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
                d_obstacle = np.sqrt(
                    (obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
                #print(f"{i} and {j}")

                # finding theta correspoding to the goal and obstacle
                theta_goal = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
                theta_obstacle = np.arctan2(
                    obstacle[1] - Y[i][j], obstacle[0] - X[i][j])

                if d_obstacle < r:
                    delx[i][j] = np.sign(np.cos(theta_obstacle)) + 0
                    dely[i][j] = np.sign(np.cos(theta_obstacle)) + 0
                elif d_obstacle > r+s:
                    delx[i][j] = 0 + (50 * s * np.cos(theta_obstacle))
                    dely[i][j] = 0 + (50 * s * np.sin(theta_goal))
                elif d_obstacle < r+s:
                    delx[i][j] = -120 * (s+r-d_obstacle) * \
                        np.cos(theta_obstacle)
                    dely[i][j] = -120 * (s+r-d_obstacle) * \
                        np.sin(theta_obstacle)
                if d_goal < r+s:
                    if delx[i][j] != 0:
                        delx[i][j] += (50 * (d_goal-r) * np.cos(theta_goal))
                        dely[i][j] += (50 * (d_goal-r) * np.sin(theta_goal))
                    else:
                        delx[i][j] = (50 * (d_goal-r) * np.cos(theta_goal))
                        dely[i][j] = (50 * (d_goal-r) * np.sin(theta_goal))

                if d_goal > r+s:
                    if delx[i][j] != 0:
                        delx[i][j] += 50 * s * np.cos(theta_goal)
                        dely[i][j] += 50 * s * np.sin(theta_goal)
                    else:
                        delx[i][j] = 50 * s * np.cos(theta_goal)
                        dely[i][j] = 50 * s * np.sin(theta_goal)

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.quiver(X, Y, delx, dely)
        ax.add_patch(plt.Circle((25, 25), r, color='y'))
        ax.add_patch(plt.Circle((33, 40), r, color='m'))

        ax.annotate("Obstacle", xy=(25, 25), fontsize=8, ha="center")
        ax.annotate("Goal", xy=(33, 40), fontsize=10, ha="center")

        ax.set_title(
            'Combined Potential when Goal and Obstacle are different ')

        plt.show()

    def map_plot(self, message):
        '''
        Function to plot the map
        '''
        # print("Map Class : ", self.x_pos, ":",
        #     self.y_pos, ":", message.ranges[0])
        global dummy_count

        for i in range(720):
            #self.X_MAP[i] = message.ranges[i]*np.cos(0.5*i*(np.pi/180))
            #self.Y_MAP[i] = message.ranges[i]*np.sin(0.5*i*(np.pi/180))
            theta = message.angle_min+i*0.5
            #  Converting position and range to X & Y wrt robot
            self.X_OBJ[i] = message.ranges[i]*np.cos(theta)
            self.Y_OBJ[i] = message.ranges[i]*np.sin(theta)
            # if indexes are finite add to the map
            if np.isfinite(self.X_OBJ[i]) and np.isfinite(self.Y_OBJ[i]):
                # converting from robot frame to world frame
                self.X_OBJ_WORLD[i] = self.X_OBJ[i]+self.X_OBJ[i] * \
                    np.cos(self.yaw)-self.Y_OBJ[i]*np.sin(self.yaw)
                self.Y_OBJ_WORLD[i] = self.Y_OBJ[i]+self.Y_OBJ[i] * \
                    np.sin(self.yaw)+self.Y_OBJ[i]*np.cos(self.yaw)
                # converting from world frame to map grid
                self.X_OBJ_GRID[i] = (
                    self.X_OBJ_WORLD[i]/self.map_resolution)+20
                self.Y_OBJ_GRID[i] = (
                    self.Y_OBJ_WORLD[i]/self.map_resolution)+20
                # adding the point to the map
                self.motion_map[round(self.X_OBJ_GRID[i]),
                                round(self.Y_OBJ_GRID[i])] = 1

                self.motion_map[round(self.X_OBJ_GRID[i]),
                                round(self.Y_OBJ_GRID[i])] = 1

        '''
        dummy_count += 1

        if(dummy_count % 100 == 0):
            # print(self.Y_OBJ_GRID)
            for i in range(40):
                for j in range(40):
                    if self.motion_map[i, j] != 0:
                        print(i, end="")
        '''
        # if(dummy_count%100 == 0):
         #   print("")
        '''
            new_element = plt.figure()
            result = new_element.add_subplot(133, projection='2d')
            result.plot(self.motion_map[:, 0], self.motion_map[:, 1])
            plt.show()
        '''
        # self.draw_matrix(self.motion_map)
        # images[0].save('maze.gif', save_all=True,
        #              append_images=images[1:], optimize=False, duration=50, loop=0)

    def draw_matrix(self, grid):
        im = Image.new(
            'RGB', (zoom * len(grid[0]), zoom * len(grid)), (255, 255, 255))
        draw = ImageDraw.Draw(im)
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                color = (255, 255, 255)
                r = 0
                if grid[i][j] == np.Infinity or grid[i][j] == 255:
                    color = (0, 0, 0)
                else:
                    color = (255*int(grid[i][j]), 255 *
                             int(grid[i][j]), 255*int(grid[i][j]))
                if i == start_j and j == start_j:
                    color = (0, 255, 0)
                    r = borders
                if i == end_i and j == end_j:
                    color = (0, 255, 0)
                    r = borders
                draw.rectangle((j * zoom + r, i * zoom + r, j * zoom +
                               zoom - r - 1, i * zoom + zoom - r - 1), fill=color)

    def __init__(self):
        '''
        Init function
        '''
        rospy.init_node('odom_node', anonymous=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.MeasureDistance)
        self.vel=Twist()
        self.timestart=True
        self.target_achieved=False
        self.x_initial=0
        self.y_initial=0
        self.goal_position_x = 2
        self.goal_position_y = 5
        self.X_MAP = np.zeros(720)
        self.Y_MAP = np.zeros(720)
        self.x_pos = 0
        self.y_pos = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        # fix map size 40 x 40 and resolution = 0.5
        self.motion_map = np.zeros((40, 40))
        self.map_resolution = 0.5
        # Object location X & Y around the robot
        self.X_OBJ = np.zeros(720)
        self.Y_OBJ = np.zeros(720)
        # Object location X & Y in the world
        self.X_OBJ_WORLD = np.zeros(720)
        self.Y_OBJ_WORLD = np.zeros(720)
        # Object location X & Y in the grid
        self.X_OBJ_GRID = np.zeros(40)
        self.Y_OBJ_GRID = np.zeros(40)
        # self.potential_field()
        self.MoveRobot([10, 10])
        
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()


if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
