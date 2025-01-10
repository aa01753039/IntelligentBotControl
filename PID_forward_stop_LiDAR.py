
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from pid_controller import PID


target_distance = 0.35
PID_RIGHT_= PID(k_p=0.1,k_i=0.0109999,k_d=0,distance=target_distance)
# FINAL CONSTANTS PID_RIGHT_= PID(k_p=0.1,k_i=0.0109999,k_d=0,distance=target_distance)
#STARTINF POSITION near to the edge, i chose these constants because the p controls and the i just makes sure it doesnt go to close (bigger p more angular speed "more drastical" and it would need bigger i)
mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
         #LIDAR readings are anti-clockwise
         'front1':  find_nearest(msg.ranges[0:5]),
         'front2': find_nearest (msg.ranges[355:360]),
         'fleft':  find_nearest (msg.ranges[40:50]),
         'left':  find_nearest (msg.ranges[85:95]),
         'right':   find_nearest (msg.ranges[265:275]),
         'fright':   find_nearest (msg.ranges[310:320])
        
    }
   
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)




#Basic movement method
def movement():
    global regions_, mynode_,PID_RIGHT_
    regions = regions_
    right_keys=['right','fright']
    right_regions=[regions[key] for key in right_keys]
    target =  min(right_regions)
    print("Min distance in target region: ", target)
   
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    
    #initial velocities
    msg.linear.x = 0.1
        
    msg.angular.z=PID_RIGHT_.activate(target)
    print("vel ang",msg.angular.z)
    return msg

#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos_profile=qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
