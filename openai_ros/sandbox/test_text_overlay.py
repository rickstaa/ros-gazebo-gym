import rospy
from openai_ros.common.markers import TextOverlay

if __name__ == "__main__":
    rospy.init_node("marker_basic_node", anonymous=True)
    p = rospy.Publisher("/text_overlay", TextOverlay, queue_size=1)
    r = rospy.Rate(1)
    text_overlay = TextOverlay(text="Reward: 12345\nStep: 1\nEpoch: 2")

    while not rospy.is_shutdown():
        p.publish(text_overlay)
        r.sleep()
