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

# RIGHT EDGE FOLLOWING
# FINAL TEST PASSED
inputs_RF = {
    "FRS": {
        "near": [0, 0.36, 0.56],
        "medium": [0.36, 0.56, 0.85],
        "far": [0.56, 0.85, 10],
    },
    "BRS": {
        "near": [0, 0.36, 0.56],
        "medium": [0.36, 0.56, 0.85],
        "far": [0.56, 0.85, 10],
    },
}
outputs_RF = {
    "LinX": {"slow": 0.055, "medium": 0.09, "fast": 0.101},
    "AngZ": {"right": -0.385, "front": 0, "left": 0.385},
}
memb_func_RF = fuzzy.MembershipFunctions(inputs_RF, outputs_RF)
rules_RF = fuzzy.RuleBase(memb_func_RF.mf)
out_RF = [
    ["slow", "left"],
    ["slow", "left"],
    ["slow", "left"],
    ["slow", "right"],
    ["medium", "right"],
    ["medium", "left"],
    ["slow", "right"],
    ["medium", "right"],
    ["fast", "right"],
]
rules_RF.set_rules(out_RF)




# PROBANDO CON DISTANCIA INICIAL APROX DE 0.54
target_distance = 0.35
PID_RIGHT_ = PID(k_p=0.095, k_i=0.007, k_d=0, distance=target_distance)
# FUNCIONA "BIEN" CON k_p = 0.12, k_i = 0.75, k_d = 0 y limite de error_i if self.error_i > 50 or self.error_i <  50 ???? (o sea igual que si tuviera k_p = 0 ) jasjjsjs
# FUNCIONA CHANCE MEJOR CON kp O.095 Y ki = 0.005 y limite de error i de -1 a 1
# ME GUSTO MAS EL FUNCIONAMIENTO CON kp 0.095 y ki 0.007 (chance [puedo aumentar kp)
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

 

    #RIGHT EDGE FOLLOWING

    crisp_inp_REF = {"FRS": regions["fright"], "BRS": regions["right"]}
    REF = fuzzy.Fuzzifier(crisp_inp_REF)
    REF.fuzzify(memb_func_RF.mf, rules_RF.rule_base)


    d2 = min(regions["fright"],regions["right"])

    linear, angular = REF.final_outputs.values()

    print("Min distance in REF region: ",d2)


    print("Linear velocity set to: ", linear)
    print("Angular velocity set to: ", angular)

    msg.angular.z = angular
    msg.linear.x = linear
    
    return msg

    

# used to stop the robot
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
