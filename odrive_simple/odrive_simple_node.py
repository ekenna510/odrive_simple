import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
#import tf2_py #import  Quaternion
import tf2_py as tf2
from math import pi, cos, sin,fmod
#from geometry_msgs.msg import Quaternion
#from scipy.spatial.transform import Rotation as R
#from tf2 import Quaternion
#from odrive import OperationAbortedException
# this node assumes you have properly configured the odrive and FULL_CALIBRATION_SEQUENCE has been run on each axis
# and this configuration has been save after set odrv0.axis0.motor.config.pre_calibrated = True 



class odrive_simple_node(Node):
    def __init__(self):
        super().__init__("drive_simple_node")

        # keep track of last command so we only forward to odrive changes 
        self.last_cmd_vel=Twist()

        self.declare_parameter('my_debug', 'false')

        #default initial position
        self.pose = Pose()
        self.pose.position.x=0.0
        self.pose.position.y=0.0
        self.pose.position.z=0.0
        self.pose.orientation.x=0.0
        self.pose.orientation.y=0.0
        self.pose.orientation.z=0.0
        self.pose.orientation.w=0.0
        
        # quat
        self.Q = Quaternion()
        
        # odom message for publishing default values
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x=0.0
        self.odom.pose.pose.position.y=0.0
        self.odom.pose.pose.position.z=0.0
        #self.odom.pose.pose.orientation.x = 0
        #self.odom.pose.pose.orientation.y = 0
        #self.odom.pose.pose.orientation.z = 0
        #self.odom.pose.pose.orientation.w = 0
        self.odom.twist.twist.linear.x=0.0
        self.odom.twist.twist.linear.y=0.0
        self.odom.twist.twist.linear.z=0.0
        self.odom.twist.twist.angular.x=0.0
        self.odom.twist.twist.angular.y=0.0
        self.odom.twist.twist.angular.z=0.0
        
        self.odom.twist.covariance=[1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e3]
        self.odom.pose.covariance= [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e3]



        #starting theta
        self.lasttheta=0.0

        self.totallefttick=0

        self.totalrighttick=0
        self.Diameter= .170
        self.Radius=.085
        self.Wheelbase= 0.5461

        # testing odom

        debug_str = self.get_parameter('my_debug')
        self.get_logger().info(str(debug_str.value))

        if str(debug_str.value) == 'True':
            self.get_logger().info("Debug true")
            self.debug=True
        else: 
            self.get_logger().info("Debug false")
            self.debug=False
        self.test_lefttick=0
        self.test_righttick=0
        self.test_leftincrease=0
        self.test_rightincrease=0

        if not self.debug:
            self.get_logger().info("finding the odrive")
            self.odrv0=odrive.find_any()
            self.odrv0.axis0.controller.input_vel=0.0
            self.odrv0.axis1.controller.input_vel=0.0
            self.print_status()

        self.get_logger().info("setup subscriber")
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            100)
        self.setpose = self.create_subscription(
            PoseWithCovarianceStamped,
            'set_pose',
            self.setpose_callback,
            100)

        #  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        #  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);    
        self.Odompublisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(.1, self.calc_odom)
        self.odom.header.stamp = self.get_clock().now().to_msg() 
        self.get_logger().info("End odrive init")

        self.lasttime = self.get_clock().now()


    def print_status(self):
        self.get_logger().info("Voltage " + str(self.odrv0.vbus_voltage))
        self.get_logger().info("Axis state L " + str(self.odrv0.axis0.current_state) + " R " +  str(self.odrv0.axis1.current_state) )
        self.get_logger().info("Axis error L " + str(self.odrv0.axis0.error) + " R " +  str(self.odrv0.axis1.error) )
        self.get_logger().info("Axis requested_state L " + str(self.odrv0.axis0.requested_state) + " R " +  str(self.odrv0.axis1.requested_state) )        
        self.get_logger().info("Motor armed_state L " + str(self.odrv0.axis0.motor.armed_state) + " R " +  str(self.odrv0.axis1.motor.armed_state) )
        self.get_logger().info("Motor is_calibrated L " + str(self.odrv0.axis0.motor.is_calibrated) + " R " +  str(self.odrv0.axis1.motor.is_calibrated) )        
        self.get_logger().info("Motor config pre_calibrated L " + str(self.odrv0.axis0.motor.config.pre_calibrated) + " R " +  str(self.odrv0.axis1.motor.config.pre_calibrated) )        
        self.get_logger().info("Motor error L " + str(self.odrv0.axis0.motor.error) + " R " +  str(self.odrv0.axis1.error) )
        self.get_logger().info("Encoder is_ready L " + str(self.odrv0.axis0.encoder.is_ready) + " R " + str(self.odrv0.axis1.encoder.is_ready) )        
        self.get_logger().info("Encoder config pre_calibrated L " + str(self.odrv0.axis0.encoder.config.pre_calibrated) + " R " +  str(self.odrv0.axis1.encoder.config.pre_calibrated) )        
        self.get_logger().info("Encoder error L " + str(self.odrv0.axis0.encoder.error) + " R " +  str(self.odrv0.axis1.encoder.error) )        

    def full_calibration(self):
        self.get_logger().info("Running full calibration")
        self.odrv0.axis0.requested_state=AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv0.axis1.requested_state=AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(40)
        self.get_logger().info("completed full calibration")

    def encoder_calibration(self):
        self.get_logger().info("Running encoder calibration")
        self.odrv0.axis0.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.odrv0.axis1.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(30)
        self.print_status()
        self.odrv0.axis0.encoder.set_linear_count(0) #this reset encoder
        self.odrv0.axis1.encoder.set_linear_count(0) #this reset encoder        

    def cmd_vel_callback(self, msg):
        if self.last_cmd_vel.linear.x == msg.linear.x and self.last_cmd_vel.angular.z == msg.angular.z:
            self.get_logger().debug('got same msg ' )        
        else:
            self.get_logger().info('got new x= ' + str(msg.linear.x) + " z = " +str(msg.angular.z))     
            self.send_velocity(msg.linear.x,msg.angular.z)
            self.last_cmd_vel.linear.x=msg.linear.x
            self.last_cmd_vel.angular.z=msg.angular.z

    def send_velocity(self,v,z):
        # https://www.youtube.com/watch?v=aE7RQNhwnPQ 
        # vr= (2v + zL)/2R
        # vl= (2v - zL)/2R
        # L = wheel base R = wheel radius v=linear speed z is rotation
        # 2R= diameter
        # v = meter per second z is radians per second.
        # for odrive need rotations
        # I think vl and vr are maybe radians per second divide by 2 pi 6.283184

        vr= (((2 * v) + (z * self.Wheelbase))/self.Diameter)/6.2832
        vl=(((2 * v) - (z * self.Wheelbase))/self.Diameter)/6.2832
        if self.debug:
            self.test_leftincrease = int( vl*3200)
            self.test_rightincrease = int(vr*3200)
            self.get_logger().info('new vl= ' + str(vl) + " vr = " +str(vr)+ " left inc " + str(self.test_leftincrease)+ " right " + str(self.test_rightincrease ) )            
        else:
            self.get_logger().info('new vl= ' + str(vl) + " vr = " +str(vr))
            self.odrv0.axis0.controller.input_vel=vl
            self.odrv0.axis1.controller.input_vel=-vr
            self.odrv0.axis0.requested_state=AxisState.CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state=AxisState.CLOSED_LOOP_CONTROL

    def debugodomticks(self):
        self.test_lefttick += self.test_leftincrease 
        self.test_righttick += self.test_rightincrease
        return self.test_lefttick,self.test_righttick

    def calc_odom(self):
        #https://www.youtube.com/watch?v=XbXhA4k7Ur8
        #dc = Dl + Dr/2
        #
        # newx = x + dc * cos(o)
        # newy = y + dc * sin(o)
        # newo = o + ((Dr - Dl)/L)
        #
        # deltatick = newtick - oldtick
        # D = 2PIR*(deltatick/N)
        #right (axiz1) comes as negative so we negate it
        # 

        if self.debug:
            newlefttick,newrighttick =self.debugodomticks()
        else:
            newlefttick,newrighttick = self.odrv0.axis0.encoder.shadow_count, -self.odrv0.axis1.encoder.shadow_count

        #,leftspeed,rightspeed ,self.odrv0.axis0.encoder.vel_estimate,self.odrv0.axis1.encoder.vel_estimate
        # max int32 is 2147483648 this should be good for 222 miles need to figureout how to deal with overflow.




        deltalefttick= newlefttick -self.totallefttick
        deltarighttick= newrighttick-self.totalrighttick

        self.totallefttick = newlefttick
        self.totalrighttick = newrighttick
        # hardcoding (3.141592 * self.Diameter) =circumference=0.53407064
        # hardcoding circumference/3200=0.000166897
        # distance per tick * number of ticks = distance
        distanceleft = 0.000166897 * deltalefttick
        distanceright = 0.000166897 * deltarighttick
        distancecenter=( distanceleft +distanceright)/2
        self.get_logger().info("clicks " + str(newlefttick) + " " + str(newrighttick) + " delta " + str(deltalefttick) + " " + str(deltarighttick) + " dist " + str(distanceleft) + " " + str(distanceright) + " "+ str(distancecenter) , throttle_duration_sec=1)  

        delta_theta   =  (deltarighttick - distanceleft) / self.Wheelbase
        if distanceleft == distanceright:
            delta_theta=0.0
            self.pose.position.x +=  distancecenter * cos( self.pose.position.z ) # delta_s * cos(bot_pose.theta + (delta_theta / 2.0));
            self.pose.position.y +=  distancecenter * sin( self.pose.position.z  ) # bot_pose.theta + (delta_theta / 2.0));           
        else:
            delta_theta = (distanceright - distanceleft) / self.Wheelbase
            r = distancecenter / delta_theta
            self.pose.position.x  += r * (sin(delta_theta + self.pose.position.z ) - sin(self.pose.position.z ))
            self.pose.position.y  -= r * (cos(delta_theta + self.pose.position.z ) - cos(self.pose.position.z ))
            self.pose.position.z  = self.normalize_angle(self.pose.position.z  + delta_theta)

        #-----------------------------------------------------------------------------
        # https://github.com/code-iai/roboclaw_ros/blob/master/roboclaw_node/nodes/roboclaw_node.py
        #    dist_left = left_ticks / self.TICKS_PER_METER
        #dist_right = right_ticks / self.TICKS_PER_METER
        #dist = (dist_right + dist_left) / 2.0

        #current_time = rospy.Time.now()
        #d_time = (current_time - self.last_enc_time).to_sec()
        #self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted

        #    d_theta = 0.0
        #    self.cur_x += dist * cos(self.cur_theta)
        #    self.cur_y += dist * sin(self.cur_theta)
        #else:
        #    d_theta = (dist_right - dist_left) / self.BASE_WIDTH
        #    r = dist / d_theta
        #    self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
        #    self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
        #    self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        #if abs(d_time) < 0.000001:
        #    vel_x = 0.0
        #    vel_theta = 0.0
        #else:
        #    vel_x = dist / d_time
        #    vel_theta = d_theta / d_time

        #return vel_x, vel_theta
        #-----------------------------------------------------------------------------
        current_time = self.get_clock().now()
        delta_time= ((current_time-self.lasttime).nanoseconds)/100000000
        a = 0
        #delta_time

        self.lasttime = current_time

        #self.pose.pose.position.x = self.pose.position.x + delta_s * cos(bot_pose.theta + (delta_theta / 2.0));
        #self.pose.pose.position.y = self.pose.position.y + delta_s * sin(bot_pose.theta + (delta_theta / 2.0));
        #self.pose.pose.position.z = self.pose.position.z+ delta_theta;

        #quat = R.from_euler('xyz',[)
        #Quaternion quat
        #quat.SetRPY(0,0,self.pose.position.z])
        self.odom.header.stamp = current_time.to_msg() #self.get_clock().now().to_msg()  
        self.odom.pose.pose.position.x = self.pose.position.x 
        self.odom.pose.pose.position.y =self.pose.position.y 
        self.odom.pose.pose.position.z = 0.0

        #tf_conversions.transformations.quaternion_from_euler(0, 0, theta*np.pi/180)
        quat = self.quaternion_from_euler(0.0,0.0,self.pose.position.z)
        self.Q.w =quat[0]
        self.Q.x =quat[1]
        self.Q.y =quat[2]
        self.Q.z =quat[3]

        self.odom.pose.pose.orientation = self.Q
        #self.odom.pose.pose.orientation = self.Q
        #self.odom.pose.pose.orientation = self.Q
        #self.odom.pose.pose.orientation = self.Q
        #self.odom.pose.pose.orientation = self.Q

        self.odom.twist.twist.linear.x = distancecenter/delta_time
        self.odom.twist.twist.linear.y= 0.0
        self.odom.twist.twist.linear.z= 0.0
        self.odom.twist.twist.angular.x= 0.0
        self.odom.twist.twist.angular.y= 0.0
        self.odom.twist.twist.angular.z= delta_theta/delta_time

        self.Odompublisher.publish(self.odom)
        if not self.debug:
            if self.odrv0.vbus_voltage < 23.2:
                self.get_logger().info("LOW BATTERY SENDING STOP PLEASE RECHARGE VOLTAGE " + str(self.odrv0.vbus_voltage) )  
                print('\a')
                self.send_velocity(0.0,0.0)    
            elif self.odrv0.vbus_voltage < 23.8:
                self.get_logger().info("BATTERY WARNING " + str(self.odrv0.vbus_voltage) )
                print('\a')


    def quaternion_from_euler(self,roll, pitch, yaw):

        #Converts euler roll, pitch, yaw to quaternion (w in last place)
        #quat = [x, y, z, w]
        #Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    def normalize_angle_positive(self,angle):
        # Normalizes the angle to be 0 to 2*pi
        # It takes and returns radians. 
         return fmod(fmod(angle, 2.0*pi) + 2.0*pi, 2.0*pi)
    def normalize_angle(self,angle):
        #Normalizes the angle to be -pi to +pi
        #    It takes and returns radians.
        a = self.normalize_angle_positive(angle)
        if a > pi:
            a -= 2.0 *pi
        return a    
    def setpose_callback(self, msg):

        self.get_logger().info("resetting")
        self.pose.position.x=msg.pose.pose.position.x
        self.pose.position.y=msg.pose.pose.position.y
        self.pose.position.z=0.0        

        #ros2 topic pub /<robot_namespace>/odom geometry_msgs/Twist "linear:  x: 0.0  y: 0.0"

def main(args=None):
    rclpy.init(args=args)
    node = odrive_simple_node()
    if not node.debug:
        node.encoder_calibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()