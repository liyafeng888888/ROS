#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, Quaternion, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import radians, pi, copysign, copysign, sqrt, pow

import tf
import PyKDL
import actionlib
import dashgo_tools.msg


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class CheckServer():
    
    # create messages that are used to publish feedback/result
    _goal = dashgo_tools.msg.check_msgGoal()
    _feedback = dashgo_tools.msg.check_msgFeedback()
    _result = dashgo_tools.msg.check_msgResult()

    def __init__(self):
        
        print("loginfo: CheckServer init()")
        self._as = actionlib.SimpleActionServer('check_server', dashgo_tools.msg.check_msgAction, self.server_callback, False)
        self._as.start()
        self._ac = actionlib.SimpleActionClient('check_server', dashgo_tools.msg.check_msgAction)
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)        
        rospy.Subscriber("check", String, self.topic_callback)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

    def topic_callback(self,msg_data):

        print("loginfo: topic_callback()")
        _msg = msg_data.data.split()
        print(_msg)

        if _msg[0].upper() == "STOP":
            rospy.loginfo("Stopping the robot...")
            goal_pub = dashgo_tools.msg.check_msgGoal(sensor=_msg[0].upper(),imu=0, goal=0, vel=0, error=1)
            self._ac.wait_for_server()
            self._ac.send_goal(goal_pub)
            return

        if len(_msg) != 5:
            print("numbers of parameters error: 5 expected, %s received." %len(_msg))
            return
        if _msg[0].upper() not in ("LINE","LINEAR","ANGULE","ANGULAR"):
            print("No.1 parameter error: \"line, linear\", \"angule, angular\" only.")
            return
        if _msg[1].upper() not in ("TRUE","1","FALSE","0"):
            print("No.2 parameter error: \"true\" or \"false\" only, using imu or not.")
            return
        try:
            float(_msg[2])
            float(_msg[3])
            float(_msg[4])
        except ValueError:
            print("No.3~5 parameters error, \"goal, vel, tolerance\" should be float")
            return

        if _msg[1].upper() in ("FALSE","0"):
            goal_pub = dashgo_tools.msg.check_msgGoal(sensor=_msg[0].upper(),imu=0,goal=float(_msg[2]), vel=float(_msg[3]), error=float(_msg[4]))
        else:
            goal_pub = dashgo_tools.msg.check_msgGoal(sensor=_msg[0].upper(),imu=1,goal=float(_msg[2]), vel=float(_msg[3]), error=float(_msg[4]))
        self._ac.wait_for_server()
        self._ac.send_goal(goal_pub)
        return
    
    def server_callback(self,goal):

        print("loginfo: server_callback()")

        if goal.sensor.upper() in ("STOP"):
            self.stop_execute()
        elif goal.sensor.upper() in ("LINE","LINEAR"):
            if (goal.goal>10 or goal.vel<0.02 or goal.vel>1.0):
                print("for safety, please make sure the distance < 10m, the velocity is between 0.02~1.0 m/s.")
                self.cmd_vel.publish(Twist())
                self._result.issuccess = True
                self._as.set_succeeded(self._result)
                print("waiting for next commend")
                return
            if goal.imu==0:
                self.linear_execute(goal)
            else:
                self.linear_imu_execute(goal)
        elif goal.sensor.upper() in ("ANGULE","ANGULAR"):
            if (goal.vel<0.06):
                print("for safety reason, please make sure the velocity is faster than 0.05rad/s.")
                self.cmd_vel.publish(Twist())
                self._result.issuccess = True
                self._as.set_succeeded(self._result)
                print("waiting for next commend")
                return
            if goal.imu==0:
                self.angular_execute(goal)
            else:
                self.angular_imu_execute(goal)
        else:
            print("loginfo: server_callback() error")

    def stop_execute(self):
#        
        print("loginfo: linear_execute()")
        rospy.loginfo("stop_execute(): Stopping the robot...")
        self.cmd_vel.publish(Twist())
        self._result.issuccess = True
        self._as.set_succeeded(self._result)

        print("waiting for next commend")


    def linear_execute(self, goal):

        print("loginfo: linear_execute()")

        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        self._goal.goal = goal.goal
        self._goal.vel = goal.vel
        self._goal.error = goal.error

        # Set the distance to travel
        self.test_distance = rospy.get_param('~test_distance', self._goal.goal) # meters
        self.speed = rospy.get_param('~speed', self._goal.vel) # meters per second
        self.tolerance = rospy.get_param('~tolerance', self._goal.error) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)

        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()

        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()

            if self._as.is_preempt_requested():
                rospy.loginfo('check_linear: Preempted')
                self._as.set_preempted()
                break

            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    # rospy.loginfo(params)

                    self._result.issuccess = True
                    rospy.loginfo('linear_execute: Succeeded')
                    self._as.set_succeeded(self._result)
                    break
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)

                    self._feedback.accomplished = distance
                    self._as.publish_feedback(self._feedback)

            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")

    def linear_imu_execute(self, goal):

        print("loginfo: linear_imu_execute()")

        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        self._goal.goal = goal.goal
        self._goal.vel = goal.vel
        self._goal.error = goal.error

        # Set the distance to travel
        self.test_distance = rospy.get_param('~test_distance', self._goal.goal) # meters
        self.speed = rospy.get_param('~speed', self._goal.vel) # meters per second
        self.tolerance = rospy.get_param('~tolerance', self._goal.error) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)

        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        #self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()

        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()

            if self._as.is_preempt_requested():
                rospy.loginfo('check_linear: Preempted')
                self._as.set_preempted()
                break

            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    # rospy.loginfo(params)

                    self._result.issuccess = True
                    rospy.loginfo('linear_imu_execute: Succeeded')
                    self._as.set_succeeded(self._result)
                    break
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)

                    self._feedback.accomplished = distance
                    self._as.publish_feedback(self._feedback)

            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")


    def angular_execute(self, goal):

        print("loginfo: angular_execute() called")
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        self._goal.goal = goal.goal
        self._goal.vel = goal.vel
        self._goal.error = goal.error

        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', self._goal.goal))
        self.speed = rospy.get_param('~speed', self._goal.vel) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', self._goal.error)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        
        while not rospy.is_shutdown():

            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    if self._as.is_preempt_requested():
                        rospy.loginfo('check_angular: Preempted')
                        self._as.set_preempted()
                        break

                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    self._feedback.accomplished = turn_angle * 57.2957795 # 180/pi
                    self._as.publish_feedback(self._feedback)

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                                    
                

                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}

                self._result.issuccess = True
                rospy.loginfo('angular_execute: Succeeded')
                self._as.set_succeeded(self._result)
                break
                
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")

    def angular_imu_execute(self, goal):

        print("loginfo: angular_imu_execut() called")
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        self._goal.goal = goal.goal
        self._goal.vel = goal.vel
        self._goal.error = goal.error

        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', self._goal.goal))
        self.speed = rospy.get_param('~speed', self._goal.vel) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', self._goal.error)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.023)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        #self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        
        while not rospy.is_shutdown():

            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    if self._as.is_preempt_requested():
                        rospy.loginfo('check_angular: Preempted')
                        self._as.set_preempted()
                        break

                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    self._feedback.accomplished = turn_angle * 57.2957795 # 180/pi
                    self._as.publish_feedback(self._feedback)

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                    #rospy.loginfo('error:'+str(error*180/3.1415926)+" "+str(error)+"tolerance:"+str(self.tolerance))   
                

                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}

                self._result.issuccess = True
                rospy.loginfo('angular_imu_execut: Succeeded')
                self._as.set_succeeded(self._result)
                break
                
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def get_odom_angle(self):
            # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))
            
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    print("")
    print("check_server begin")
    rospy.init_node('check_server')
    server = CheckServer()
    rospy.spin()
