import cv2 as cv
import numpy as np

# optional argument for trackbars
def nothing(x):
    pass

# named ites for easy reference
barsWindow = 'Bars'
hl = 'H Low'
hh = 'H High'
sl = 'S Low'
sh = 'S High'
vl = 'V Low'
vh = 'V High'

# set up for video capture on camera 0
cap = cv.VideoCapture(1)

# create window for the slidebars
cv.namedWindow(barsWindow, flags = cv.WINDOW_AUTOSIZE)

# create the sliders
cv.createTrackbar(hl, barsWindow, 0, 179, nothing)
cv.createTrackbar(hh, barsWindow, 0, 179, nothing)
cv.createTrackbar(sl, barsWindow, 0, 255, nothing)
cv.createTrackbar(sh, barsWindow, 0, 255, nothing)
cv.createTrackbar(vl, barsWindow, 0, 255, nothing)
cv.createTrackbar(vh, barsWindow, 0, 255, nothing)

# set initial values for sliders
cv.setTrackbarPos(hl, barsWindow, 0)
cv.setTrackbarPos(hh, barsWindow, 179)
cv.setTrackbarPos(sl, barsWindow, 0)
cv.setTrackbarPos(sh, barsWindow, 255)
cv.setTrackbarPos(vl, barsWindow, 0)
cv.setTrackbarPos(vh, barsWindow, 255)

while(True):
    ret, frame = cap.read()
    # frame = cv.GaussianBlur(frame, (5, 5), 0)
    # frame = cv.medianBlur(frame,11)
    
    # convert to HSV from BGR
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # read trackbar positions for all
    hul = cv.getTrackbarPos(hl, barsWindow)
    huh = cv.getTrackbarPos(hh, barsWindow)
    sal = cv.getTrackbarPos(sl, barsWindow)
    sah = cv.getTrackbarPos(sh, barsWindow)
    val = cv.getTrackbarPos(vl, barsWindow)
    vah = cv.getTrackbarPos(vh, barsWindow)

    # make array for final values
    HSVLOW = np.array([hul, sal, val])
    HSVHIGH = np.array([huh, sah, vah])

    # apply the range on a mask
    mask = cv.inRange(hsv, HSVLOW, HSVHIGH)
    mask = cv.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
    mask = cv.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
    maskedFrame = cv.bitwise_and(frame, frame, mask = mask)

    # display the camera and masked images
    cv.imshow('Masked', maskedFrame)
    cv.imshow('Camera', frame)

	# check for q to quit program with 5ms delay
    if cv.waitKey(5) & 0xFF == ord('q'):
        break

# clean up our resources
cap.release()
cv.destroyAllWindows()