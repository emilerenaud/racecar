#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


#########################################
class slash_controller(object):

    #######################################
    def __init__(self):

        # Init subscribers
        self.sub_ref     = rospy.Subscriber("ctl_ref", Twist, self.read_ref, queue_size=1)
        self.sub_prop    = rospy.Subscriber("prop_sensors", Float32MultiArray , self.read_arduino, queue_size=1)
        self.sub_laser   = rospy.Subscriber("car_position", Twist, self.read_laser, queue_size=1)

        # Init publishers
        self.pub_cmd    = rospy.Publisher("prop_cmd", Twist , queue_size=1)

        # Timer
        self.dt         = 0.05
        self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )

        # Paramters

        # Controller
        self.steering_offset = 0.0 # To adjust according to the vehicle

        self.K_autopilot = np.array([[-3.9350e-17, 8.2422, -2.7685e-16], [0.3162, -1.5156e-16, 0.5383]]) # DESIGN LQR
        self.N_autopilot = np.array([[10.1874, -3.935e-17], [-1.5156e-16, 0.3162]])

        self.K_parking   = np.array([[0, 1, 0], [0.3187, 0, 0.6]]) # DESIGN PLACEMENT DE POLES
        self.N_parking   = np.array([[1, 0], [0, 0.3187]])

        # Memory

        # References Inputs
        self.propulsion_ref  = 0
        self.steering_ref    = 0
        self.high_level_mode = 0  # Control mode of this controller node

        # Ouput commands
        self.propulsion_cmd = 0  # Command sent to propulsion
        self.arduino_mode   = 0  # Control mode
        self.steering_cmd   = 0  # Command sent to the steering servo

        # Sensings inputs
        self.laser_y        = 0
        self.laser_theta    = 0
        self.velocity       = 0
        self.position       = 0

        # Filters
        self.laser_y_old    = 0
        self.laser_dy_fill  = 0

    #######################################
    def timed_controller(self, timer):

        # Computation of dy / dt with filtering
        self.laser_dy_fill = 0.9 * self.laser_dy_fill + 0.1 * (self.laser_y - self.laser_y_old) / self.dt
        self.laser_y_old   = self.laser_y

        if (self.high_level_mode < 0 ):
            # Full stop mode
            self.propulsion_cmd = 0  # Command sent to propulsion
            self.arduino_mode   = 0  # Control mode
            self.steering_cmd   = 0  # Command sent to the steering servo

        else:

            # APP2 (open-loop steering) Controllers Bellow
            if  ( self.high_level_mode == 1 ):
                # Open-Loop
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode   = 1
                self.steering_cmd   = self.steering_ref + self.steering_offset

            # For compatibility mode 0 needs to be closed-loop velocity
            elif ( self.high_level_mode == 0 ):
                # Closed-loop velocity on arduino
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode   = 2
                self.steering_cmd   = self.steering_ref + self.steering_offset

            elif ( self.high_level_mode == 2 ):
                # Closed-loop position on arduino
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode   = 3
                self.steering_cmd   = self.steering_ref + self.steering_offset

            # APP4 (closed-loop steering) controllers bellow
            elif ( self.high_level_mode == 3 or self.high_level_mode == 5  ):
                # Closed-loop velocity and steering

                x = [self.laser_y, self.velocity, -self.laser_theta]
                r = [self.propulsion_ref, self.steering_ref]

                u = self.controller1(x, r) # Volts and steering angle

                self.propulsion_cmd = self.propulsion_ref # u[0]
                self.steering_cmd   = u[1] + self.steering_offset
                self.arduino_mode = 5 # Mode where V is calculated on Arduino | 1 # Open-loop on arduino

            elif ( self.high_level_mode == 4 ):
                # Closed-loop position and steering

                x = [self.laser_y, self.position, -self.laser_theta]
                r = [self.propulsion_ref, self.steering_ref]

                u = self.controller2(x , r)

                self.propulsion_cmd = u[0] # Speed in m/s
                self.steering_cmd   = u[1] + self.steering_offset
                self.arduino_mode   = 2 # Closed-loop speed on arduino

            elif ( self.high_level_mode == 6 ):
                # Reset encoders
                self.propulsion_cmd = 0
                self.arduino_mode   = 4
                self.steering_cmd   = 0


            elif ( self.high_level_mode == 7 ):
                # Template for custom controllers

                self.steering_cmd   = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode   = 0 # Mode ??? on arduino


            elif ( self.high_level_mode == 8 ):
                # Template for custom controllers

                self.steering_cmd   = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode   = 0 # Mode ??? on arduino

        self.send_arduino()


    #######################################
    def controller1(self, x , r):
        # Control Law Velocity
        # Returns u (Volts, steering angle)
        return np.dot(self.N_autopilot, r) - np.dot(self.K_autopilot, x)

    #######################################
    def controller2(self, x , r ):
        # Control Law Position
        # Returns u (Speed m/s, steering angle)
        return np.dot(self.N_parking, r) - np.dot(self.K_parking, x)

    #######################################
    def read_ref(self, ref_msg):

        #Read received references
        self.propulsion_ref  = ref_msg.linear.x
        self.high_level_mode = ref_msg.linear.z
        self.steering_ref    = ref_msg.angular.z

    #######################################
    def read_laser(self, msg):

        self.laser_y     = msg.linear.y
        self.laser_theta = msg.angular.z


    #######################################
    def read_arduino( self, msg):

        # Read feedback from arduino
        self.velocity       = msg.data[1]
        self.position       = msg.data[0]

    ##########################################################################################
    def send_arduino(self):

      #Init encd_info msg
      cmd_prop = Twist()

      #Msg
      cmd_prop.linear.x  = self.propulsion_cmd   # Command sent to propulsion
      cmd_prop.linear.z  = self.arduino_mode     # Control mode

      cmd_prop.angular.z = self.steering_cmd #Command sent to the steering servo

      # Publish cmd msg
      self.pub_cmd.publish(cmd_prop)


    #######################################
    def pub_kinematic( self ):
        # init encd_info msg
        pos = Twist()
        vel = Twist()
        acc = Twist()

        # Msg
        pos.linear.x = 0
        vel.linear.x = 0
        acc.linear.x = 0

        # Publish cmd msg
        self.pub_pos.publish( pos )
        self.pub_vel.publish( vel )
        self.pub_acc.publish( acc )


#########################################

if __name__ == '__main__':
    rospy.init_node('controller',anonymous=False)
    node = slash_controller()
    rospy.spin()
