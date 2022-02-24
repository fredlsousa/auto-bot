import cv2
import numpy as np
from matplotlib.colors import hsv_to_rgb


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


if __name__ == "__main__":

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
    while True:

        try:
            ret, img = cap_send.read()

            if not ret:
                print('Empty frame! Exiting...')
                break

            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

            # set the lower and upper bounds for the orange hue
            # lower_orange = np.array([5, 50, 50])
            # upper_orange = np.array([15, 255, 255])
            # lower_orange = np.array([1, 190, 200])
            # upper_orange = np.array([18, 255, 255])

            # lower_orange = np.array([10, 100, 80])
            # upper_orange = np.array([25, 255, 255])

            # lo_square = np.full((10, 10, 3), lower_orange, dtype=np.uint8) / 255.0
            # do_square = np.full((10, 10, 3), upper_orange, dtype=np.uint8) / 255.0

            # create a mask for orange colour using inRange function
            # mask = cv2.inRange(hsv, lower_orange, upper_orange)

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

            # if len(contours) > 0:
            #     for cont in contours:
            #         # print("Contour: ", contours[0])
            #         rect = cv2.minAreaRect(cont)
            #         (x, y), (w, h), a = rect
            #
            #         box = cv2.boxPoints(rect)
            #         box = np.int0(box)  # turn into ints
            #         rect2 = cv2.drawContours(rgb, [box], 0, (0, 0, 255), 3)
                # print(box)

            # print("Alou")
            # print(img)

            # cv2.imwrite("debug/" + str(i) + "rgb.jpg", img)
            # cv2.imwrite("debug/" + str(i) + "hsv.jpg", hsv)
            # cv2.imwrite("debug/" + str(i) + "mask.jpg", mask)
            i += 1

            # img = cv2.circle(rgb, (30, 50), 10, hsv_to_rgb(lower_orange), 40)
            # img = cv2.circle(rgb, (30, 110), 10, hsv_to_rgb(upper_orange), 40)
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            cv2.imshow("img_window", img_res)
            cv2.imshow("hsv", hsv)
            cv2.imshow("mask", img_cones)
            s = cv2.waitKey(1)

            if s == ord("q"):
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        except KeyboardInterrupt:
            print("Keyboard_interrupt")
            cv2.destroyAllWindows()
            break
        # rate.sleep()
        # rospy.spin()
