import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
#Position 0 is back right

dist = None
lfeel = None
rfeel = None
rlook = None
llook = None

rospy.init_node("maze_wanderer")

fl = rospy.Publisher('front_left_wheel_controller/command', Float64, queue_size = 0)
bl = rospy.Publisher('back_left_wheel_controller/command', Float64, queue_size = 0)
fr = rospy.Publisher('front_right_wheel_controller/command', Float64, queue_size = 0)
br = rospy.Publisher('back_right_wheel_controller/command', Float64, queue_size = 0)

def callback(data):
	global dist
	global llook
	global rlook
	global lfeel
	global rfeel
	mid = (len(data.ranges)/2)
	ahead = data.ranges[mid]
	rlook = data.ranges[(len(data.ranges)/6)]
	llook = data.ranges[5*(len(data.ranges)/6)]
	rfeel = data.ranges[5*(len(data.ranges)/12)]
	lfeel = data.ranges[7*(len(data.ranges)/12)]
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
	drive(5,5)
	rospy.sleep(0.1)
	if(rfeel < 0.35):
		while(rfeel < 0.4):
			drive(-2.5,2.5)
			rospy.sleep(0.1)
	if(lfeel < 0.35):
		while(lfeel < 0.4):
			drive(2.5,-2.5)
			rospy.sleep(0.1)
	if(dist < 0.25):
		drive(0,0)
		rospy.sleep(.25)
		if(rlook > llook):
			while(lfeel < 0.4):
				drive(2.5,-2.5)
				rospy.sleep(0.1)
		else:
			while(rfeel < 0.4):
				drive(-2.5,2.5)
				rospy.sleep(0.1)
