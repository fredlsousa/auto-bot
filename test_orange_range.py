import cv2
import numpy as np


if __name__ == "__main__":

    fast_fps = True         # needs opencv with Gstreamer build

    if fast_fps:    # if true, camera framerate is higher but if loop gets stuck in the prediciton, there'll be delays
        # cap_send = cv2.VideoCapture('v4l2src device=/dev/video0 num-buffers=1000 ! '
        #                             'video/x-raw,width=640,height=480,framerate=30/1 ! videorate ! '
        #                             'video/x-raw,framerate=30/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

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

            # rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

            # set the lower and upper bounds for the orange hue
            lower_orange = np.array([5, 50, 50])
            upper_orange = np.array([15, 255, 255])

            # create a mask for orange colour using inRange function
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # perform bitwise and on the original image arrays using the mask
            # res = cv2.bitwise_and(img, img, mask=mask)

            # contours, _ = cv2.findContours(mask.copy(), 1, 1)  # find the contours on the segmentation mask
            # _, thresh = cv2.threshold(mask, 127, 255, 0)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # find the contours on the segmentation mask

            if len(contours) > 0:
                print("Contour")
                rect = cv2.minAreaRect(contours[0])
                (x, y), (w, h), a = rect

                box = cv2.boxPoints(rect)
                box = np.int0(box)  # turn into ints
                rect2 = cv2.drawContours(img, [box], 0, (0, 0, 255), 3)
                print(box)

            # print("Alou")
            # print(img)

            # cv2.imwrite("debug/" + str(i) + "rgb.jpg", img)
            # cv2.imwrite("debug/" + str(i) + "hsv.jpg", hsv)
            # cv2.imwrite("debug/" + str(i) + "mask.jpg", mask)
            i += 1

            cv2.imshow("img_window", img)
            cv2.imshow("hsv", hsv)
            cv2.imshow("mask", mask)
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
