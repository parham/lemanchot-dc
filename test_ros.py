
import rospy
import sensor_msgs.msg as msg


def test(img):
    print(img.height)

rospy.init_node('lemanchot_gui')
rospy.Subscriber('/phm/depth_camera/color/image_raw', msg.Image, callback=test)