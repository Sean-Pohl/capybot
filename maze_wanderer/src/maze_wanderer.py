import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

dist = None
llook = None
rLook = None

rospy.init_node("maze_wanderer")

fl = rospy.Publisher('front_left_wheel_controller/command', Float64, queue_size = 0)
bl = rospy.Publisher('back_left_wheel_controller/command', Float64, queue_size = 0)
fr = rospy.Publisher('front_right_wheel_controller/command', Float64, queue_size = 0)
br = rospy.Publisher('back_right_wheel_controller/command', Float64, queue_size = 0)

def callback(data):
	global dist
	global llook
	global rlook
	ahead = data.ranges[(len(data.ranges)/2)]
	llook = data.ranges[(len(data.ranges)/6)]
	rlook = data.ranges[5*(len(data.ranges)/6)]
	dist = ahead
	print(ahead)

scan = rospy.Subscriber('scan', LaserScan, callback)

def drive(left_speed, right_speed):
	fl.publish(left_speed)
	fr.publish(right_speed)
	bl.publish(left_speed)
	br.publish(right_speed)
rospy.sleep(0.5)
while(True):
	while(dist > 0.25):
		drive(5,5)
		rospy.sleep(0.1)
	drive(0,0)
	rospy.sleep(0.5)
	if(llook < rlook):
		drive(-2.5,2.5)
		rospy.sleep(1.85)
	else:
		drive(2.5,-2.5)
		rospy.sleep(1.85)
	drive(0,0)
	rospy.sleep(0.5)
