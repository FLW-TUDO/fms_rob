import rospy
from math import pow, atan2, sqrt, cos, sin, pi
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PointStamped




class test1:

    def __init__(self):
        rospy.init_node('test')
        ROBOT_ID = rospy.get_param('/ROBOT_ID')
        cart_id = 'KLT_7_neu'

        pose_sub = rospy.Subscriber('/vicon/'+ROBOT_ID+'/'+ROBOT_ID, TransformStamped, self.update_pose) ####
        cart_pose_sub = rospy.Subscriber('/vicon/'+cart_id+'/'+cart_id, TransformStamped, self.get_cart_pose) # obtaining picked cart id pose



    def get_cart_pose(self, data):
        #rospy.loginfo_throttle(1, 'getting cart pose')
        cart_pose_trans = [data.transform.translation.x, data.transform.translation.y]
        cart_pose_rot = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(cart_pose_rot)
        cart_theta = rot_euler[2]
        print('\t \t \t \t KLT Theta: {}'.format(cart_theta))

    def update_pose(self, data):
        """ Robot vicon pose update. """
        curr_pose_trans_x = data.transform.translation.x
        curr_pose_trans_y = data.transform.translation.y
        rot=[data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        curr_theta = rot_euler[2]
        print('Curr Theta: {}'.format(curr_theta))


if __name__ == '__main__':
    test1()
    rospy.spin()