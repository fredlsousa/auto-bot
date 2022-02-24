import cv2
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray

# Motor speed constants
# RANGE = 75  # medium speed, relative control
# STEP = 25
#
#
# def step(a, b):
#     """ for each element in 'a' step it towards element in 'b'"""
#     s = []
#     for va, vb in zip(a, b):
#         if va == vb:
#             s.append(va)
#         elif va < vb:
#             s.append(va + STEP)
#         else:
#             s.append(va - STEP)
#     return s


publisher = rospy.Publisher('/cmd', Int16MultiArray, queue_size=1, latch=True)
rospy.init_node("autonomous_node")
rate = rospy.Rate(100)


def pub(values):            # method to publish wheel speeds
    values = [-v for v in values]
    rospy.loginfo("publish [%s]" % values)
    msg = Int16MultiArray()
    msg.data = values
    publisher.publish(msg)


# def find_orange_slow():
#     values = [51, -51, -51, 51]
#     rospy.loginfo("publish [%s]" % values)
#     msg = Int16MultiArray()
#     msg.data = values
#     publisher.publish(msg)


def convex_hull_pointing_up(ch):

    points_above_center, points_below_center = [], []

    x, y, w, h = cv2.boundingRect(ch)
    aspect_ratio = w / h

    if aspect_ratio < 0.8:
        vertical_center = y + h / 2

        for point in ch:
            if point[0][
                1] < vertical_center:
                points_above_center.append(point)
            elif point[0][1] >= vertical_center:
                points_below_center.append(point)

        left_x = points_below_center[0][0][0]
        right_x = points_below_center[0][0][0]
        for point in points_below_center:
            if point[0][0] < left_x:
                left_x = point[0][0]
            if point[0][0] > right_x:
                right_x = point[0][0]

        for point in points_above_center:
            if (point[0][0] < left_x) or (point[0][0] > right_x):
                return False
    else:
        return False

    return True


def check_bbox_size(l_bbox, c_bbox):
    # base x altura
    l_bbox_height = np.sqrt(np.power(l_bbox[0][0] - l_bbox[3][0], 2) + np.power(l_bbox[0][1] - l_bbox[3][1], 2))
    c_bbox_height = np.sqrt(np.power(c_bbox[0][0] - c_bbox[3][0], 2) + np.power(c_bbox[0][1] - c_bbox[3][1], 2))

    l_bbox_width = np.sqrt(np.power(l_bbox[1][0] - l_bbox[2][0], 2) + np.power(l_bbox[1][1] - l_bbox[2][1], 2))
    c_bbox_width = np.sqrt(np.power(c_bbox[1][0] - c_bbox[2][0], 2) + np.power(c_bbox[1][1] - c_bbox[2][1], 2))

    l_bbox_area = l_bbox_width * l_bbox_height
    c_bbox_area = c_bbox_width * c_bbox_height

    if c_bbox_area >= l_bbox_area:
        return True
    else:
        return False


def findOrangeCone(hsv, img, debug):

    img_thresh_low = cv2.inRange(hsv, np.array([0, 135, 135]), np.array([15, 255, 255]))
    img_thresh_high = cv2.inRange(hsv, np.array([159, 135, 135]), np.array([179, 255, 255]))
    img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_high)

    # perform bitwise and on the original image arrays using the mask
    # res = cv2.bitwise_and(img, img, mask=mask)

    # contours, _ = cv2.findContours(mask.copy(), 1, 1)  # find the contours on the segmentation mask
    # _, thresh = cv2.threshold(mask, 127, 255, 0)

    kernel = np.ones((5, 5))
    img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
    img_edges = cv2.Canny(img_thresh_blurred, 80, 160)

    # contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours, _ = cv2.findContours(img_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_contours = np.zeros_like(img_edges)
    # cv2.drawContours(img_contours, contours, -1, (255, 255, 255), 2)

    approx_contours = []

    for c in contours:
        approx = cv2.approxPolyDP(c, 10, closed=True)
        approx_contours.append(approx)

    img_approx_contours = np.zeros_like(img_edges)
    # cv2.drawContours(img_approx_contours, approx_contours, -1, (255, 255, 255), 1)

    all_convex_hulls = []
    for ac in approx_contours:
        all_convex_hulls.append(cv2.convexHull(ac))

    img_all_convex_hulls = np.zeros_like(img_edges)
    # cv2.drawContours(img_all_convex_hulls, all_convex_hulls, -1, (255, 255, 255), 2)

    convex_hulls_3to10 = []
    for ch in all_convex_hulls:
        if 3 <= len(ch) <= 10:
            convex_hulls_3to10.append(cv2.convexHull(ch))

    img_convex_hulls_3to10 = np.zeros_like(img_edges)
    # cv2.drawContours(img_convex_hulls_3to10, convex_hulls_3to10, -1, (255, 255, 255), 2)

    cones = []
    bounding_rects = []
    for ch in convex_hulls_3to10:
        if convex_hull_pointing_up(ch):
            cones.append(ch)
            rect = cv2.boundingRect(ch)
            bounding_rects.append(rect)

    img_cones = np.zeros_like(img_edges)
    cv2.drawContours(img_cones, cones, -1, (255, 255, 255), 2)

    img_res = img.copy()
    # cv2.drawContours(img_res, cones, -1, (255, 255, 255), 2)

    for rect in bounding_rects:
        # print(rect)
        cv2.rectangle(img_res, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (1, 255, 1), 3)

    if debug:
        return bounding_rects, img, img_cones
    else:
        return bounding_rects


if __name__ == "__main__":

    # set initial values to [0] and publish them.
    current = [0] * 4
    targets = [0] * 4
    pub(current)
    debug_flag = True

    fast_fps = True         # needs opencv with Gstreamer build

    if fast_fps:
        cap_send = cv2.VideoCapture('v4l2src device=/dev/video0 ! '
                                    'video/x-raw,width=640,height=480,framerate=30/1 ! videorate ! '
                                    'video/x-raw,framerate=30/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

    else:
        cap_send = cv2.VideoCapture(0)

    if not cap_send.isOpened():
        print('VideoCapture not opened')
        exit(0)

    i = 0
    # last_box = [0, 0, 0, 0]
    last_box = np.zeros(shape=4,dtype=np.int8)
    while not rospy.is_shutdown():
        set_immediately = False
        try:
            ret, img = cap_send.read()

            if not ret:
                print('Empty frame! Exiting...')
                break

            # rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # set the lower and upper bounds for the orange hue
            # lower_orange = np.array([10, 100, 20])
            # upper_orange = np.array([70, 255, 255])

            # create a mask for orange colour using inRange function
            # mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Bitwise to maks image with colored detections
            # res = cv2.bitwise_and(img, img, mask=mask)

            # contours, _ = cv2.findContours(mask.copy(), 1, 1)  # find the contours on the segmentation mask
            # contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # find the contours on the segmentation mask

            bounding_rects, img_res, img_cones = findOrangeCone(hsv, img, debug_flag)
            bounding_rects = np.array(bounding_rects)
            if bounding_rects.size > 0:
                rect = bounding_rects[0]
                # box = cv2.boxPoints(rect)
                box = np.int0(rect)  # turn into ints
                print(box, box[0], last_box)

                if box.size > 0:  # if orange point is found, go towards it
                    if check_bbox_size(last_box, box):  # if last bbox is smaller than the current bbox
                        last_box = box
                        pwm_vals = [51, 51, 51, 51]
                        pub(pwm_vals)
                        time.sleep(5.9)
                        pwm_vals = [0, 0, 0, 0]
                        pub(pwm_vals)

                else:   # if orange is not found rotate to find it
                        pwm_vals = [51, -51, -51, 51]
                        pub(pwm_vals)
                        time.sleep(3.4)
                        pwm_vals = [0, 0, 0, 0]
                        pub(pwm_vals)


            # if len(contours) > 0:
            #     # print("Contour")
            #     rect = cv2.minAreaRect(contours[0])
            #     (x, y), (w, h), a = rect
            #
            #     box = cv2.boxPoints(rect)
            #     box = np.int0(box)  # turn into ints
            #     # rect2 = cv2.drawContours(img, [box], 0, (0, 0, 255), 3)
            #
            #     if box:  # if orange point is found, go towards it
            #         if check_bbox_size(last_box, box):  # if last bbox is smaller than the current bbox
            #             last_box = box
            #             pwm_vals = [51, 51, 51, 51]
            #             pub(pwm_vals)
            #             time.sleep(5.9)
            #             pwm_vals = [0, 0, 0, 0]
            #             pub(pwm_vals)
            #
            #     else:   # if orange is not found rotate to find it
            #             pwm_vals = [51, -51, -51, 51]
            #             pub(pwm_vals)
            #             time.sleep(3.4)
            #             pwm_vals = [0, 0, 0, 0]
            #             pub(pwm_vals)

                # print(box)
            if debug_flag:
                cv2.imshow("img_window", img_res)
                cv2.imshow("hsv", hsv)
                cv2.imshow("mask", img_cones)
                s = cv2.waitKey(1)

                if s == ord("q"):
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()


        except KeyboardInterrupt:
            print("Keyboard_interrupt")
            # cv2.destroyAllWindows()
            break

        # ROS Publisher on /cmd topic, publish wheel speeds
        # pub(current)

        # rate.sleep()
        # rospy.spin()
        rospy.sleep(0.01)

    pub([0] * 4)