#! /usr/bin/python3

import argparse
import os
from typing import Tuple

import cv2
import numpy as np


def calibrate(rows:int = 9, cols:int = 6, dir:str = "./"):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Get object points ready
    objp = np.zeros((rows*cols,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:cols].T.reshape(-1,2)
    objp = 38.5*10e-3 * objp

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    path_list = sorted(os.listdir(dir))
    for image_name in path_list:
        image_path = dir + image_name
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("frame", img)
        cv2.waitKey(50)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img, (9,6), None, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_NORMALIZE_IMAGE)
        print(ret)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            img2 = cv2.drawChessboardCorners(img, (9,6), corners2, ret)
            cv2.imshow("frame", img2)
            cv2.waitKey(50)

    cv2.destroyAllWindows()

    _, intrinsics_matrix, distortion_coefficients, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    os.chdir("/app/data/calibration_parameters")
    print("intrinsics\n", intrinsics_matrix)
    print("distortion\n", distortion_coefficients)
    np.save("intrinsics", intrinsics_matrix)
    np.save("distortion", distortion_coefficients)

def capture(nb_images:int = 50, dir:str = "./"):
    camera = cv2.VideoCapture(0)

    images = []
    should_close = False
    i = 0
    while camera.isOpened() and not should_close:
        ret, image = camera.read()
        if not ret:
            continue
    
        cv2.imshow("Image", image)
        key = cv2.waitKey(1)
        print(key)
        if key & 0xFF == 113 or key == ord("q"):
            print("User requested stop, images won't be saved")
            should_close = True
        elif key & 0xFF == 115 or key == ord("s"):
            print("Image saved ! images left:", nb_images-i)
            images.append(image)
            i += 1
    camera.release()

    if not should_close:
        for idx, image in enumerate(images):
            cv2.imwrite(f"image_{idx}.png", image)

def main():
    parser = argparse.ArgumentParser("Calibration", description="An OpenCV-based camera calibration tool.")
    subparsers = parser.add_subparsers()
    capture_parser = subparsers.add_parser("capture", description="Capture images of the camera to calibrate.")
    capture_parser.add_argument("-n", "--nb_images", type=int, default=50, help="The number of images to record. Default is 50.")
    capture_parser.add_argument("-d", "--dir", type=str, default="./", help="The directory to save images to. Default is pwd.")
    capture_parser.set_defaults(func="capture")
    calibrate_parser = subparsers.add_parser("calibrate", description="Calibrate a camera from captured images.")
    calibrate_parser.add_argument("-r", "--rows", type=int, default=9, help="The number of chessboard rows.")
    calibrate_parser.add_argument("-c", "--cols", type=int, default=6, help="The number of chessboard columns.")
    calibrate_parser.add_argument("-d", "--dir", type=str, default="./", help="The directory to save images to. Default is pwd.")
    calibrate_parser.set_defaults(func="calibrate")

    args = parser.parse_args()
    args = args.__dict__
    func_name = args.pop("func")
    globals()[func_name](**args)


if __name__ == "__main__":
    main()