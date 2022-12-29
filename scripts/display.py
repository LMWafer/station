#! /usr/bin/python3

import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import pytransform3d.transformations as ptf
import pytransform3d.plot_utils as ptu
import rospy
import tf


def main():
    #-> Parse call arguments
    #-? rospy.myarg to filter out remapping args, [1:] to remove calling script name
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("display", anonymous=True)
    rospy.loginfo("Initializing node with name 'display'.")
    if config.verbose:
        rospy.loginfo("Node verbose required. Further logs will be outputed.")

    listener = tf.TransformListener()
    if config.verbose:
        rospy.loginfo("Transform listener created.")

    plt.ion()
    ax = ptf.plot_transform(s=0.2)
    fig = ax.get_figure()
    
    expected_ids = rospy.get_param("/robotlab/world")["expected_ids"]
    reference_id = rospy.get_param("/robotlab/world")["reference_id"]
    shapes = rospy.get_param("/robotlab/world")["objects_shape"]
    reference_frame = f"marker_{reference_id}"
    z_offset = shapes[str(reference_id)]["H"]
    T_offset = ptf.transform_from(R=np.identity(3), p=np.array([0, 0, z_offset]))

    #-> Let time to transformations to be created
    time.sleep(5)

    #-> Wait for transformations to be available
    for id in expected_ids:
            listener.waitForTransform(f"marker_{id}", "/map", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        #-> Setup axes for new drawing
        ax.cla()
        ptf.plot_transform(ax, name="R", s=0.2)
        ax.set_xlim(-0.7, 0.7)
        ax.set_ylim(-0.7, 0.7)
        ax.set_zlim(0, 1.4)

        #-> For each marker, get its frame wrt to reference marker and plot both its frame and its box
        for id in expected_ids:
            #-> Get marker frame wrt to reference frame
            T = np.identity(4)
            if id is not reference_id:
                child_frame = f"marker_{id}"
                now = rospy.Time.now()
                try:
                    listener.waitForTransform(reference_frame, child_frame, now, rospy.Duration(4.0))
                    tvec, qvec = listener.lookupTransform(reference_frame, child_frame, now)
                except Exception:
                    rospy.logerr(f"Error during retrieval of frame {id}. Skipping this iteration.")
                    break
                T = np.array(listener.fromTranslationRotation(tvec, qvec))
            T = T @ T_offset

            shape = shapes[str(id)]
            L, W, H = shape["L"], shape["W"], shape["H"]
            T_center = ptf.transform_from(R=np.identity(3), p=np.array([0, 0, -H])/2)
            T_box = T @ T_center
            ptu.plot_box(ax=ax, size=[L, W, H], A2B=T_box, alpha=0.2)
            
            #-> Plot marker transform in reference frame coordinate system
            ptf.plot_transform(ax, T, name=id, s=0.2)

            if config.verbose:
                rospy.loginfo(f"Added id {id} to the 3D plot.")
        fig.canvas.draw()
        fig.canvas.flush_events()

if __name__ == "__main__":
    main()