import cv2
import sys
import select
import tty
import termios
import time
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray

# Motor speed constants
RANGE = 75  # medium speed, relative control
STEP = 25


def step(a, b):
    """ for each element in 'a' step it towards element in 'b'"""
    s = []
    for va, vb in zip(a, b):
        if va == vb:
            s.append(va)
        elif va < vb:
            s.append(va + STEP)
        else:
            s.append(va - STEP)
    return s


publisher = rospy.Publisher('/cmd', Int16MultiArray, queue_size=1, latch=True)
rospy.init_node("autonomous_node")
rate = rospy.Rate(100)


def pub(values):            # method to publish wheel speeds
    values = [-v for v in values]
    rospy.loginfo("publish [%s]" % values)
    msg = Int16MultiArray()
    msg.data = values
    publisher.publish(msg)


if __name__ == "__main__":

    # set initial values to [0] and publish them.
    current = [0] * 4
    targets = [0] * 4
    pub(current)

    # set cbreak on stdin (recall original attr to restore later)
    # this is required for the polling select on stdin.
    original_t_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

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
            lower_orange = np.array([10, 100, 20])
            upper_orange = np.array([70, 255, 255])

            # create a mask for orange colour using inRange function
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Bitwise to maks image with colored detections
            # res = cv2.bitwise_and(img, img, mask=mask)

            # contours, _ = cv2.findContours(mask.copy(), 1, 1)  # find the contours on the segmentation mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # find the contours on the segmentation mask

            if len(contours) > 0:
                # print("Contour")
                rect = cv2.minAreaRect(contours[0])
                (x, y), (w, h), a = rect

                box = cv2.boxPoints(rect)
                box = np.int0(box)  # turn into ints
                rect2 = cv2.drawContours(img, [box], 0, (0, 0, 255), 3)
                # print(box)

            # debug imshow
            # s = cv2.waitKey(1)
            #
            # if s == ord("q"):
            #     cv2.waitKey(0)
            #     cv2.destroyAllWindows()

            # Motor Controllers
            # if c == ' ':  # immediate stop
            #     set_immediately = True
            #     targets = [0] * 4
            #     print(targets)
            # elif c == 'w':  # forward
            #     targets = [RANGE] * 4
            #     print(targets)
            # elif c == 's':  # slow stop
            #     targets = [0] * 4
            #     print(targets)
            # elif c == 'x':  # backwards
            #     targets = [-RANGE] * 4
            #     print(targets)
            # elif c == 'f':  # skid steer left
            #     targets = [RANGE, -RANGE, RANGE, -RANGE]
            #     print(targets)
            # elif c == 'g':  # skid steer right
            #     targets = [-RANGE, RANGE, -RANGE, RANGE]  # [right_1, left_1, right_2, left_2]
            #     print(targets)  # 1 - front, 2 - back

            # Omni Directional Commands
            # elif c == 'd':  # mecanum omni right
            #     targets = [RANGE + 10, -RANGE, -RANGE, RANGE + 10]
            # elif c == 'a':  # mecanum omni left
            #     targets = [-RANGE, RANGE + 10, RANGE + 10, -RANGE]
            # elif c == 'e':  # mecanum omni diaconal rigt up
            #     targets = [RANGE + 25, 0, 0, RANGE + 25]
            # elif c == 'z':  # mecanum omni diaconal left down
            #     targets = [-RANGE + (-25), 0, 0, -RANGE + (-25)]
            # elif c == 'q':  # mecanum omni diaconal left up
            #     targets = [0, RANGE + 25, RANGE + 25, 0]
            # elif c == 'c':  # mecanum omni diaconal right down
            #     targets = [0, -RANGE + (-25), -RANGE + (-25), 0]

        except KeyboardInterrupt:
            print("Keyboard_interrupt")
            # cv2.destroyAllWindows()
            break

        # ROS Publisher on /cmd topic, publish wheel speeds
        if current != targets:
            if set_immediately:
                current = targets
            else:
                current = step(current, targets)
            pub(current)

        # rate.sleep()
        # rospy.spin()
        rospy.sleep(0.01)

    pub([0] * 4)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_t_attr)
