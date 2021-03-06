import argparse

import cv2
import numpy as np
import math
import os
from objloader import *

# Minimum number of matches that have to be found
# to consider the recognition valid
MIN_MATCHES = 10


def main():
    """
    This functions loads the target surface image,
    """
    start_record = False
    stop = False
    # matrix of camera parameters
    camera_parameters = np.array([[800, 0, 320], [0, 800, 120], [0, 0, 1]])

    model = cv2.imread('backgroundcard.jpg', 0)
    print(model.shape)
    # pc camera
    cap = cv2.VideoCapture(0)

    save_f = 'result_1.mp4'
    fps = cap.get(cv2.CAP_PROP_FPS)
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    # DIVX for MPEG-4 coder.
    video_writer = cv2.VideoWriter(
        save_f, cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

    while True:
        # read the current frame
        ret, frame = cap.read(0)
        if ret:

            orb = cv2.ORB_create()  # Fast(Features from Accelerated Segment Test) and rotation brief feature
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Brute-Force matcher, for ORB hamming is best choice
            kp_model, des_model = orb.detectAndCompute(model, None)
            kp_frame, des_frame = orb.detectAndCompute(frame, None)

            matches = bf.match(des_model, des_frame)  # match return most match, knnmatch return top k
            matches = sorted(matches, key=lambda x: x.distance)

            res = cv2.drawMatches(model, kp_model, frame, kp_frame,
                                  matches[: MIN_MATCHES], 0, flags=2)
            cv2.imshow('res', res)
            cv2.waitKey(0)

            obj = OBJ('models/wolf.obj', swapyz=True)
            # compute Homography if enough matches are found
            if len(matches) > MIN_MATCHES:
                # differenciate between source points and destination points
                src_pts = np.float32([kp_model[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
                # compute Homography,mask is online points on destination picture
                homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                if args.rectangle:
                    # Draw a rectangle that marks the found model in the frame
                    h, w = model.shape
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, homography)

                    # connect them with lines
                    frame = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
                    # if a valid homography matrix was found render cube on model plane
                if homography is not None:
                    try:
                        # obtain 3D projection matrix from homography matrix and camera parameters
                        projection = projection_matrix(camera_parameters, homography)
                        # project cube or model
                        frame = render(frame, obj, projection, model, False)
                        # frame = render(frame, model, projection)
                    except Exception as e:
                        print('fond issue: {}'.format(e))
                # draw first 10 matches.
                if args.matches:
                    frame = cv2.drawMatches(model, kp_model, frame, kp_frame, matches[:10], 0, flags=2)
                # show result
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                     stop = True
                if cv2.waitKey(1) & 0xFF == ord('r'):
                    start_record = True
                print("Not enough matches found - %d/%d" % (len(matches), MIN_MATCHES))
                if start_record:
                    print('recording....')
                    video_writer.write(frame)
                if stop:
                    video_writer.close()


def render(img, obj, projection, model, color=True):
    """
    Render a loaded obj model into the current video frame
    """
    vertices = obj.vertices
    scale_matrix = np.eye(3) * 3
    h, w = model.shape

    for face in obj.faces:
        face_vertices = face[0]
        points = np.array([vertices[vertex - 1] for vertex in face_vertices])
        points = np.dot(points, scale_matrix)
        # render model in the middle of the reference surface. To do so,
        # model points must be displaced
        points = np.array([[p[0] + w / 2, p[1] + h / 2, p[2]] for p in points])
        dst = cv2.perspectiveTransform(points.reshape(-1, 1, 3), projection)
        imgpts = np.int32(dst)
        if color is False:
            cv2.fillConvexPoly(img, imgpts, (137, 27, 211))
        else:
            color = hex_to_rgb(face[-1])
            color = color[::-1]  # reverse
            cv2.fillConvexPoly(img, imgpts, color)

    return img


def projection_matrix(camera_parameters, homography):
    """
    From the camera calibration matrix and the estimated homography
    compute the 3D projection matrix
    """
    # Compute rotation along the x and y axis as well as the translation
    homography *= -1
    rot_and_transl = np.dot(np.linalg.inv(camera_parameters), homography)
    col_1 = rot_and_transl[:, 0]
    col_2 = rot_and_transl[:, 1]
    col_3 = rot_and_transl[:, 2]
    # normalise vectors
    l = math.sqrt(np.linalg.norm(col_1, 2) * np.linalg.norm(col_2, 2))
    rot_1 = col_1 / l
    rot_2 = col_2 / l
    translation = col_3 / l
    # compute the orthonormal basis
    c = rot_1 + rot_2
    p = np.cross(rot_1, rot_2)
    d = np.cross(c, p)
    rot_1 = np.dot(c / np.linalg.norm(c, 2) + d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_2 = np.dot(c / np.linalg.norm(c, 2) - d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_3 = np.cross(rot_1, rot_2)
    # finally, compute the 3D projection matrix from the model to the current frame
    projection = np.stack((rot_1, rot_2, rot_3, translation)).T
    return np.dot(camera_parameters, projection)


def hex_to_rgb(hex_color):
    """
    Helper function to convert hex strings to RGB
    """
    hex_color = hex_color.lstrip('#')
    h_len = len(hex_color)
    return tuple(int(hex_color[i:i + h_len // 3], 16) for i in range(0, h_len, h_len // 3))


# Command line argument parsing
# NOT ALL OF THEM ARE SUPPORTED YET
parser = argparse.ArgumentParser(description='Augmented reality application')

parser.add_argument('-r', '--rectangle', help='draw rectangle delimiting target surface on frame',
                    default=False, action='store_true')
parser.add_argument('-mk', '--model_keypoints', help='draw model keypoints', action='store_true')
parser.add_argument('-fk', '--frame_keypoints', help='draw frame keypoints', action='store_true')
parser.add_argument('-ma', '--matches', help='draw matches between keypoints', action='store_true')
parser.add_argument('-mo','--model', help = 'Specify model to be projected', action = 'store_true')

args = parser.parse_args()

if __name__ == '__main__':
    main()
