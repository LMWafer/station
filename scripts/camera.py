#! /usr/bin/python3

import argparse
import sys

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv_bridge as br

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    parser.add_argument("-d", "--display", action="store_true", help="Show current processed image with marker detections.")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node("camera")
    if config.verbose:
        rospy.loginfo("Node started with name 'camera'.")

    publisher = rospy.Publisher("color/image_raw", Image, queue_size=1)

    camera = cv2.VideoCapture(0)
    bridge = br.CvBridge()
    seq = 0
    
    def on_shutdown():
        camera.release()
        cv2.destroyAllWindows()
    rospy.on_shutdown(on_shutdown)

    try:
        while camera.isOpened() and not rospy.is_shutdown():
            ret, image = camera.read()
            if not ret:
                continue

            if config.verbose:
                rospy.loginfo(f"---------- Retrieved an image. ----------")

            if config.display:
                cv2.imshow("Color", image)
                cv2.waitKey(1)

            image_msg = bridge.cv2_to_imgmsg(image)
            
            header = Header()
            header.seq = seq
            seq += 1
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_color_optical_frame"
            image_msg.header = header

            if config.verbose:
                rospy.loginfo("Sending image message.")
            publisher.publish(image_msg)
    
    except rospy.ROSInterruptException:
        return


if __name__ == "__main__":
    main()