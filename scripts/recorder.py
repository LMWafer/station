#! /usr/bin/python3

import argparse
import os
import shutil
import sys
import tarfile

import cv2
import cv_bridge as br
import numpy as np
import rospy
from sensor_msgs.msg import Image


config = argparse.Namespace()
bridge = None
mst_images = []
gt_images = []


def mst_image_callback(image_message: Image):
    """Callback for measurments images (from the robot)"""
    global config, bridge, mst_images

    image = bridge.imgmsg_to_cv2(image_message)    
    if config.verbose:
            rospy.loginfo("MST image retrieved.")

    if config.display:
        if config.verbose:
            rospy.loginfo("Displaying MST image.")
        cv2.imshow("Measurement camera", image)
        cv2.waitKey(1)

    mst_images.append(image)
    if config.verbose:
        rospy.loginfo("MST image recorded.")

def gt_image_callback(image_message: Image):
    """Callback for ground truth images (from the station)"""
    global config, bridge, gt_images
    
    image = bridge.imgmsg_to_cv2(image_message)
    if config.verbose:
            rospy.loginfo("GT image retrieved.")

    if config.display:
        if config.verbose:
            rospy.loginfo("Displaying GT image.")
        cv2.imshow("Ground truth camera", image)
        cv2.waitKey(1)

    gt_images.append(image)
    if config.verbose:
        rospy.loginfo("GT image recorded.")

def main():
    global config, bridge, images
    parser = argparse.ArgumentParser()
    parser.add_argument("mst_topic_name", metavar="NAME", type=str, help="Name of the robotlab's measurement images topic.")
    parser.add_argument("gt_topic_name", metavar="NAME", type=str, help="Name of the robotlab's ground truth images topic.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    parser.add_argument("-d", "--display", action="store_true", help="Show recieved images.")
    parser.add_argument("-o", "--output", type=str, default="images", help="Name of the output tarball containing recorded images. Directory is /tmp/robotlab_records/.")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    #-> Initialize node
    rospy.init_node("recorder")
    if config.verbose:
        rospy.loginfo("Started node with name 'recorder'")

    #-> Create message2image converter and visualization windows
    bridge = br.CvBridge()
    if config.display:
        cv2.namedWindow("Measurement camera", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Ground truth camera", cv2.WINDOW_AUTOSIZE)
        if config.verbose:
            rospy.loginfo("Visualization window created.")

    def on_shutdown():
        #-[] Close window
        if config.display:
            cv2.destroyWindow("Measurement camera")
            cv2.destroyWindow("Ground truth camera")
            if config.verbose:
                rospy.loginfo("Window destroyed.")
        
        if config.verbose:
            rospy.loginfo("Creating record tarball.")
        if not os.path.exists("/tmp/robotlab_records"):
            os.mkdir("/tmp/robotlab_records")

        #-> Remove previous tarball if it already exists
        if os.path.exists(config.output):
            if config.verbose:
                rospy.logwarn(f"Previous tarball found with name '{config.output}', it will be replaced.")
            os.remove(config.output)
        
        #-[] Save recorded measurement images
        os.chdir("/tmp/robotlab_records")
        if os.path.exists("mst"):
            shutil.rmtree("mst")
        os.mkdir("mst")
        output = config.output + "_measurement.tar.gz"
        with tarfile.open(output, "w:gz") as tar:
            for idx, image in enumerate(mst_images):
                image_path = f"mst/{idx}.npy"
                np.save(image_path, image)
                tar.add(image_path)
                os.remove(image_path)
        shutil.rmtree("mst")

        #-[] Save recorded ground truth images
        os.chdir("/tmp/robotlab_records")
        if os.path.exists("gt"):
            shutil.rmtree("gt")
        os.mkdir("gt")
        output = config.output + "_ground_truth.tar.gz"
        with tarfile.open(output, "w:gz") as tar:
            for idx, image in enumerate(gt_images):
                image_path = f"gt/{idx}.npy"
                np.save(image_path, image)
                tar.add(image_path)
                os.remove(image_path)
        shutil.rmtree("gt")

        if config.verbose:
            rospy.loginfo(f"{len(mst_images)} recorded MST images were saved.")
            rospy.loginfo(f"{len(gt_images)} recorded GT images were saved.")
    rospy.on_shutdown(on_shutdown)

    #-> Start main subscriber    
    rospy.Subscriber(config.mst_topic_name, Image, mst_image_callback)
    rospy.Subscriber(config.gt_topic_name, Image, gt_image_callback)

    #-> Spin the node
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass