import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

def talker():
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('window_pub', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	robotMarker = Marker()
	robotMarker.header.frame_id = "/Cmap"
	robotMarker.header.stamp    = rospy.get_rostime()
	robotMarker.ns = "window"
	robotMarker.id = 0
	robotMarker.type = 5
	robotMarker.action = 0
	robotMarker.color.r = 0.0
	robotMarker.color.g = 1.0
	robotMarker.color.b = 0.0
	robotMarker.color.a = 1.0   
	#self.robotMarker.lifetime = 0
	robotMarker.pose.position = self.state.point
	robotMarker.pose.orientation.x = 0
self.robotMarker.pose.orientation.y = 0
self.robotMarker.pose.orientation.z = 0
self.robotMarker.pose.orientation.w = 1.0
self.robotMarker.scale.x = 1.0
self.robotMarker.scale.y = 1.0
self.robotMarker.scale.z = 1.0




	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()




if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
