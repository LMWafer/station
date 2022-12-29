#! /usr/bin/python3

import argparse
import sys

import cv2
import cv_bridge as br
import numpy as np
import pytransform3d.transformations as ptf
import rospy
import tf
from sensor_msgs.msg import CameraInfo, Image

config = argparse.Namespace()
bridge = br.CvBridge()
broadcaster = tf.TransformBroadcaster()
expected_agents_ids, expected_structure_ids, expected_objects_ids = [], [], []


def callback_image(image_msg: Image, argv: tuple):
    global config, bridge, broadcaster, agents, structure, objects
    
    aruco_dicts, aruco_params, marker_sizes, K, D = argv
    agents_dict, structure_dict, objects_dict = aruco_dicts
    agents_marker_size, structure_marker_size, objects_marker_size = marker_sizes

    if config.verbose:
        rospy.loginfo(f"---------- Recieved an image. Starting markers detection. ----------")

    image_cv = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    # observed_agents_markers, observed_agents_ids, _ = cv2.aruco.detectMarkers(image_cv, agents_dict, parameters=aruco_params)
    # observed_structure_markers, observed_structure_ids, _ = cv2.aruco.detectMarkers(image_cv, structure_dict, parameters=aruco_params)
    observed_objects_markers, observed_objects_ids, _ = cv2.aruco.detectMarkers(image_cv, objects_dict, parameters=aruco_params)

    #-> Agents pose broadcasting
    # if type(observed_agents_ids) is not type(None):
    #     if config.display:
    #         image_cv = cv2.aruco.drawDetectedMarkers(image_cv, observed_agents_markers, observed_agents_ids)

    #     #-? 'agents_ids' is the list of observed agents ids
    #     observed_agents_ids = [id[0] for id in list(observed_agents_ids)]

    #     if config.verbose:
    #         rospy.loginfo("Found following agents IDs in the image: " + str(observed_agents_ids) + ".")

    #     for idx in range(len(expected_agents_ids)):
    #         #-? 'id' iterates over all expected agents ids
    #         id = expected_agents_ids[idx][0]
            
    #         #-> If expected ID is not found in image, use its last pose
    #         if id not in observed_agents_ids:
    #             if config.verbose:
    #                 rospy.loginfo(f"Agent ID {id} not found in image, sending its last transform.")

    #             TCM = expected_agents_ids[idx][1]
    #             TCM_pq = ptf.pq_from_transform(TCM)
    #             tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
    #             broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/agent_{id}", "/camera_color_optical_frame")

    #         #-> Else, update its pose
    #         else:
    #             marker = observed_agents_markers[observed_agents_ids.index(id)]
    #             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, agents_marker_size, K, D)
    #             R, _ = cv2.Rodrigues(rvec)
    #             TCM = ptf.transform_from(R, tvec)
    #             expected_agents_ids[idx][1] = TCM
            
    #             if config.verbose:
    #                 rospy.loginfo(f"Estimated transformation of agent ID {id}\n" + str(TCM))

    #             if config.display:
    #                 image_cv = cv2.drawFrameAxes(image_cv, K, D, rvec, tvec, agents_marker_size/2)

    #             TCM_pq = ptf.pq_from_transform(TCM)
    #             tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
    #             broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/agent_{id}", "/camera_color_optical_frame")

    #-> Structure pose broadcasting
    # if type(observed_structure_ids) is not type(None):
    #     if config.display:
    #         image_cv = cv2.aruco.drawDetectedMarkers(image_cv, observed_structure_markers, observed_structure_ids)

    #     #-? 'structure_ids' is the list of observed structure ids
    #     observed_structure_ids = [id[0] for id in list(observed_structure_ids)]

    #     if config.verbose:
    #         rospy.loginfo("Found following structure IDs in the image: " + str(observed_structure_ids) + ".")
        
    #     for idx in range(len(expected_structure_ids)):
    #         #-? 'id' iterates over all expected structure ids
    #         id = expected_structure_ids[idx][0]
            
    #         #-> If expected ID is not found in image, use its last pose
    #         if id not in observed_structure_ids:
    #             if config.verbose:
    #                 rospy.loginfo(f"Structure ID {id} not found in image, sending last transform.")

    #             TCM = expected_structure_ids[idx][1]
    #             TCM_pq = ptf.pq_from_transform(TCM)
    #             tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
    #             broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/structure_{id}", "/camera_color_optical_frame")

    #         #-> Else, update its pose
    #         else:
    #             marker = observed_structure_markers[observed_structure_ids.index(id)]
    #             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, structure_marker_size, K, D)
    #             R, _ = cv2.Rodrigues(rvec)
    #             TCM = ptf.transform_from(R, tvec)
    #             expected_structure_ids[idx][1] = TCM
            
    #             if config.verbose:
    #                 rospy.loginfo(f"Estimated transformation of structure ID {id}\n" + str(TCM))

    #             if config.display:
    #                 image_cv = cv2.drawFrameAxes(image_cv, K, D, rvec, tvec, structure_marker_size/2)

    #             TCM_pq = ptf.pq_from_transform(TCM)
    #             tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
    #             broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/structure_{id}", "/camera_color_optical_frame")
    
    #-> Objects pose broadcasting
    if type(observed_objects_ids) is not type(None):
        if config.display:
            image_cv = cv2.aruco.drawDetectedMarkers(image_cv, observed_objects_markers, observed_objects_ids)

        #-? 'objects_ids' is the list of observed objects ids
        observed_objects_ids = [id[0] for id in list(observed_objects_ids)]

        if config.verbose:
            rospy.loginfo("Found following objects IDs in the image: " + str(observed_objects_ids) + ".")

        for idx in range(len(expected_objects_ids)):
            #-? 'id' iterates over all expected objects ids
            id = expected_objects_ids[idx][0]
            
            #-> If expected ID is not found in image, use its last pose
            if id not in observed_objects_ids:
                if config.verbose:
                    rospy.loginfo(f"objects ID {id} not found in image, sending last transform.")

                TCM = expected_objects_ids[idx][1]
                TCM_pq = ptf.pq_from_transform(TCM)
                tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
                broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/object_{id}", "/camera_color_optical_frame")

            #-> Else, update its pose
            else:
                marker = observed_objects_markers[observed_objects_ids.index(id)]
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker, objects_marker_size, K, D)
                R, _ = cv2.Rodrigues(rvec)
                TCM = ptf.transform_from(R, tvec)
                expected_objects_ids[idx][1] = TCM
            
                if config.verbose:
                    rospy.loginfo(f"Estimated transformation of objects ID {id}\n" + str(TCM))

                if config.display:
                    image_cv = cv2.drawFrameAxes(image_cv, K, D, rvec, tvec, objects_marker_size/2)

                TCM_pq = ptf.pq_from_transform(TCM)
                tvec, qvec = TCM_pq[:3], np.roll(TCM_pq[3:], -1)
                broadcaster.sendTransform(tvec, qvec, rospy.Time.now(), f"/object_{id}", "/camera_color_optical_frame")

    if config.display:
        cv2.imshow("Markers", image_cv)
        cv2.waitKey(1)


def main():
    global config, expected_agents_ids, expected_structure_ids, expected_objects_ids

    #-> Parse call arguments
    #-? rospy.myarg to filter out remapping args, [1:] to remove calling script name
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    parser.add_argument("-d", "--display", action="store_true", help="Show current processed image with marker detections.")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    #-> Initialize node
    rospy.init_node("detector", anonymous=True)
    rospy.loginfo("Initializing node with name detector.")
    if config.verbose:
        rospy.loginfo("Node verbose required. Further logs will be outputed.")
    if config.display:
        rospy.loginfo("Node display required. Recieved images will be displayed with overlayed marker detections.")

    #-> Retrieve camera info once
    info_msg = rospy.wait_for_message("color/camera_info", CameraInfo, 3000)
    K = np.reshape(np.array(info_msg.K), (3, 3))
    D = np.array(info_msg.D)
    if config.verbose:
        rospy.loginfo("Camera calibration matrix and distortion vector retrieved.")

    #-> Retrieve parameters for markers detection
    agents_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    structure_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    objects_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    aruco_dicts = (agents_dict, structure_dict, objects_dict)
    aruco_params = cv2.aruco.DetectorParameters_create()
    if config.verbose:
        rospy.loginfo("Retrieved parameters for markers detection.")

    #-> Retrieve parameters for pose estimation
    parameters = rospy.get_param("/robotlab/world")
    agents_marker_size = parameters["agents"]["marker_size"]
    structure_marker_size = parameters["structures"]["marker_size"]
    objects_marker_size = parameters["objects"]["marker_size"]
    marker_sizes = (agents_marker_size, structure_marker_size, objects_marker_size)
    if config.verbose:
        rospy.loginfo("Retrieved parameters for pose estimation.")

    #-> Retrieve lists of expected ids
    expected_agents = parameters["agents"]["expected_ids"]
    expected_agents_ids = [[id, np.identity(4)] for id in expected_agents]
    wall_sequences = parameters["structures"]["wall_sequences"]
    expected_structure_ids = [[id, np.identity(4)] for sequence in wall_sequences for id in sequence]
    objects_shape = parameters["objects"]["shapes"]
    expected_objects_ids = [[int(id), np.identity(4)] for id in objects_shape.keys()]
    if config.verbose:
        rospy.loginfo("Retrieved lists of expected ids.")

    #-> Launch realsense color image subcriber
    rospy.Subscriber("color/image_raw", Image, callback_image, (aruco_dicts, aruco_params, marker_sizes, K, D), queue_size=1)
    if config.verbose:
        rospy.loginfo("Pubs/Subs started.")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return


if __name__ == "__main__":
    main()
