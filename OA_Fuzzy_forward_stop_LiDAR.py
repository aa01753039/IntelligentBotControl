import rclpy
import math
import fuzzy_logic as fuzzy
import fuzzy_coordinator as FuzzyC
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from pid_controller import PID


#OBSTACLE AVOIDANCE
# FINAL TEST PASSED

inputs_OA = {
    "FRS": {
        "near": [0, 0.55, 0.85],
        "medium": [0.55, 0.85, 1.2],
        "far": [0.85, 1.2, 11]
    },
    "FLS": {
        "near": [0, 0.55, 0.85],
        "medium": [0.55, 0.85, 1.2],
        "far": [0.85, 1.2, 11]
    },
    "FS": {
        "near": [0, 0.55, 0.85],
        "medium": [0.55, 0.85, 1.2],
        "far": [0.85, 1.2, 11]
    }
}
outputs_OA = {
    "LinX": {"slow": 0.01, "medium": 0.05, "fast": 0.1},
    "AngZ": {"right": -0.45, "front": 0, "left": 0.45},
}
memb_func_OA = fuzzy.MembershipFunctions(inputs_OA, outputs_OA)
rules_OA = fuzzy.RuleBase(memb_func_OA.mf)
out_OA = [
    ["slow", "left"],
    ["slow", "front"],
    ["medium", "front"],
    ["slow", "left"],
    ["medium", "left"],
    ["fast", "front"],
    ["medium", "left"],
    ["fast", "left"],
    ["fast", "left"],

    ["slow", "right"],
    ["medium", "front"],
    ["medium", "front"],
    ["medium", "left"],
    ["medium", "front"],
    ["fast", "front"],
    ["fast", "left"],
    ["fast", "left"],
    ["fast", "front"],

    ["slow", "right"],
    ["fast", "right"],
    ["fast", "front"],
    ["fast", "right"],
    ["fast", "right"],
    ["fast", "front"],
    ["fast", "left"],
    ["fast", "left"],
    ["fast", "front"],

]

rules_OA.set_rules(out_OA)

print(out_OA.rule_base)


mynode_ = None
pub_ = None
regions_ = {"right": 0, "fright": 0, "front": 0, "fleft": 0, "left": 0}
twstmsg_ = None


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if twstmsg_ != None:
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        # LIDAR readings are anti-clockwise
        "front1": find_nearest(msg.ranges[0:5]),
        "front2": find_nearest(msg.ranges[355:360]),
        "fleft": find_nearest(msg.ranges[40:50]),
        "left": find_nearest(msg.ranges[85:95]),
        "right": find_nearest(msg.ranges[265:275]),
        "fright": find_nearest(msg.ranges[310:320]),
    }
    # regions_ = {
    #     #LIDAR readings are anti-clockwise
    #     'front1':  find_nearest(msg.ranges[0:5]),
    #     'right':   find_nearest (msg.ranges[265:275]),
    #     'fright':   find_nearest (msg.ranges[310:320])

    # }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


# Basic movement method
def movement():
    global regions_, mynode_, PID_RIGHT_
    regions = regions_
    

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    #OBSTACLE AVOIDANCE

    crisp_inp_OA = {"FRS": regions["fright"], "FLS": regions["fleft"], "FS": min(regions["front1"],regions["front2"])}
    OA = fuzzy.Fuzzifier(crisp_inp_OA)
    OA.fuzzify(memb_func_OA.mf, rules_OA.rule_base)
    
    d1 = min(regions["fright"],regions["fleft"],min(regions["front1"],regions["front2"]))
    
    
    linear, angular = OA.final_outputs.values()

    print("Min distance in OA region: ", d1)


    print("Linear velocity set to: ", linear)
    print("Angular velocity set to: ", angular)

    msg.angular.z = angular
    msg.linear.x = linear
    
    return msg

    

# used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node("reading_laser")

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, "/cmd_vel", 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, "/scan", clbk_laser, qos_profile=qos)

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


if __name__ == "__main__":
    main()
