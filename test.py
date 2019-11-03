import rospy
from std_msgs.msg import Float64MultiArray

def test_arm_callback(msg):
    print 'drive train values: ', msg

def test_dt_callback(msg):
    print 'drive train values: ', msg

def arm_listener():
    rospy.init_node('arm_listener')
    rospy.loginfo("started arm listener")
    test_arm_sub = rospy.Subscriber('arm', Float64MultiArray, test_arm_callback)
    rospy.spin()

def dt_listener():
    rospy.init_node('dt_listener')
    rospy.loginfo("started drive_train")
    test_arm_sub = rospy.Subscriber('drive_train', Float64MultiArray, test_dt_callback)
    rospy.spin()


dt_listener() 
