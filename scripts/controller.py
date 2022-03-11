import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from math import atan2


x = 1.0
y = 3.0
theta = 0.0
command = "STOP"
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/motor_commands", String, queue_size = 1)


r = rospy.Rate(4)

goal = Point()
goal.x = 2.0
goal.y = 2.0

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)

    angle = angle_to_goal - theta
    if angle < -0.1 and command != "STOP":
        command = "RIGHT"
    elif angle > 0.1 and command != "STOP":
        command = "LEFT"
    else:
        command = "GO"
    print(x, y)
    if x >= (goal.x - 0.2) and y >= (goal.y - 0.2):
        command = "STOP"


    pub.publish(command)
    r.sleep()