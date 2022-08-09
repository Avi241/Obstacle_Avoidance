#!/usr/bin/env python

import imp
import rospy
import cv2
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,Imu
from std_msgs.msg import Bool,Empty
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

bridge = CvBridge()
rospy.init_node("Obstacle_Avoidance", anonymous=True) # initialize ros node


image= Image()
imu = Imu()
vel = Twist()
flag = False

def cb_img(data):
    global image,flag
    image = data
    flag = True
    # for i in range(100):
    # print("received image")
    # print(flag)

def cb_imu(data):
    global imu
    imu = data


rospy.Subscriber("/drone/front_camera/image_raw", Image,cb_img)
rospy.Subscriber("/drone/imu", Imu,cb_imu)
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
vel_ctrl_pub = rospy.Publisher("/drone/vel_mode", Bool, queue_size=10)
takeoff_pub = rospy.Publisher("/drone/takeoff", Empty, queue_size=10)

rospy.sleep(0.5)


def pub_vel(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    vel.linear.x=linear_x
    vel.linear.y=linear_z
    vel.linear.z=linear_z
    vel.angular.x = angular_x
    vel.angular.y = angular_y
    vel.angular.z = angular_z
    vel_pub.publish(vel)

def start_vel_ctrl():
    ctrl = Bool(True)
    vel_ctrl_pub.publish(ctrl)
    print("Started Velocity Control")

def takeoff():
    print("Take off")
    takeoff_pub.publish(Empty())
    rospy.sleep(0.5)

def main():
    takeoff()
    start_vel_ctrl()
    rospy.sleep(2)
    while not rospy.is_shutdown():
        if flag:
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') # To convert from ros image to opencv image
            quaternion = (
                            imu.orientation.x,
                            imu.orientation.y,
                            imu.orientation.z,
                            imu.orientation.w)

            euler = tf.transformations.euler_from_quaternion(quaternion) # convert quaternio to euler angles
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            # print(roll,pitch,yaw)

            ######################### Write your Algorithm here #############
            











            ## cv_image is your input image

            ## Roll, Pitch and Yaw angles are availabe in the corresponding variables roll,pitch,yaw





            





            # Velocity control varibales
            lin_x=0.0 # Linear velocities
            lin_y=0.0
            lin_z=0.0

            ang_x=0.0 # Angular velocities
            ang_y=0.0
            ang_z=0.0    

            #############################################################
            pub_vel(lin_x,lin_y,lin_z,ang_x,ang_y,ang_z) # Publishe the velocity to drone
            cv2.imshow('frame', cv_image)
            # print("Showing Image")
            if cv2.waitKey(1) == ord('q'):
                break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
       main()
    except KeyboardInterrupt:
        pass












