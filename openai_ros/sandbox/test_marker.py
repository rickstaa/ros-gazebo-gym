import rospy
from openai_ros.common.markers import TargetMarker

if __name__ == "__main__":
    rospy.init_node("marker_basic_node", anonymous=True)
    p = rospy.Publisher("/panda_reach/current_target", TargetMarker, queue_size=1)
    r = rospy.Rate(1)
    target_marker = TargetMarker(x=1.0)

    while not rospy.is_shutdown():
        p.publish(target_marker)
        r.sleep()
