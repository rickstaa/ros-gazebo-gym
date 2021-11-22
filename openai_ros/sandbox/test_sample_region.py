import rospy
from openai_ros.common.markers import SampleRegionMarker
from geometry_msgs.msg import Vector3

if __name__ == "__main__":
    rospy.init_node("marker_basic_node", anonymous=True)
    p = rospy.Publisher(
        "/panda_reach/target_sample_region", SampleRegionMarker, queue_size=1
    )
    r = rospy.Rate(1)
    sample_region = SampleRegionMarker(
        x_min=-0.7, x_max=0.7, y_min=-0.7, y_max=0.7, z_min=0, z_max=1.2
    )

    while not rospy.is_shutdown():
        p.publish(sample_region)
        r.sleep()
