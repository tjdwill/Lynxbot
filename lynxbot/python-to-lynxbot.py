# -*- coding: utf-8 -*-
"""
Created on Thu Nov 10 15:28:32 2022

@title: Python2Lynxbot
@author: Terrance Williams
@date: 10 November 2022
@description: Python program that handles color detection and passes color data
over Serial Port to Arudino.
"""


# Imports
import cv2 as cv
import numpy as np
from time import sleep
import serial
import sys

# Global(s)
global SERIAL_SLEEP


# Functions

def handleSerErr():
    """Allows program to exit properly upon disconnecting from serial line."""

    arduino.__del__()  # close port
    cv.destroyAllWindows()
    cap.release()  # Release Camera
    sys.exit("PYTHON: Connection with LYNX broken.")
    
    
def img_mask(image):
    """ input raw image
        outputs array of masked images (red, yellow, green, blue)
    """
    img_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Generate HSV Thresholds
    red_thresh1 = np.array([[0, SATURATION_LOWER, BRIGHTNESS_LOWER],
                            [7, SATURATION_MAX, BRIGHTNESS_MAX]])
    red_thresh2 = np.array([[165, SATURATION_LOWER, BRIGHTNESS_LOWER],
                            [179, SATURATION_MAX, BRIGHTNESS_MAX]])
    green_thresh = np.array([[40, SATURATION_LOWER, BRIGHTNESS_LOWER],
                             [90, SATURATION_MAX, BRIGHTNESS_MAX]])

    blue_thresh = np.array([[100, SATURATION_LOWER, BRIGHTNESS_LOWER],
                            [140, SATURATION_MAX, BRIGHTNESS_MAX]])
    yellow_thresh = np.array([[20, SATURATION_LOWER, BRIGHTNESS_LOWER],
                              [35, SATURATION_MAX, BRIGHTNESS_MAX]])

    # Generate Masks
    red_mask1 = cv.inRange(img_hsv, red_thresh1[0], red_thresh1[1])
    red_mask2 = cv.inRange(img_hsv, red_thresh2[0], red_thresh2[1])
    # Combine red masks
    red_mask = cv.bitwise_or(red_mask1, red_mask2, mask=None)

    green_mask = cv.inRange(img_hsv, green_thresh[0], green_thresh[1])
    blue_mask = cv.inRange(img_hsv, blue_thresh[0], blue_thresh[1])
    yellow_mask = cv.inRange(img_hsv, yellow_thresh[0], yellow_thresh[1])

    yellow_masked = cv.bitwise_and(image, image, mask=yellow_mask)
    red_masked = cv.bitwise_and(image, image, mask=red_mask)
    blue_masked = cv.bitwise_and(image, image, mask=blue_mask)
    green_masked = cv.bitwise_and(image, image, mask=green_mask)

    output = np.array([red_masked, yellow_masked,
                       green_masked, blue_masked])

    return output


def colorDetect() -> int:
    consec_detections = 0  # Tracks how many consecutive images detect a given color
    while True:
        _, img = cap.read()
        img = cv.flip(img, 1)

        # =============================================================================
        #     Detecting a color in a subregion:
        #     1. Draw rectangle around desired subregion
        #     2. Draw circle in center of subregion. The center of this circle is the
        #     pixel we will analyze.
        #     3. Segment the frame into a variable consisting solely of the subregion
        #     4. Apply relevant masking and filtering
        #     5. Perform bitwise AND operation to isolate the desired color.
        #     6. If center pixel is not black, that color is present.
        #
        # =============================================================================

        # 1. Draw rectangle
        height, width, _ = img.shape
        rect_TpLft = (0, int(7 * height / 8))
        rect_BtmRght = (int(width / 4), height)
        rect_color = (0, 255, 0)
        rect_thickness = 2
        image2 = np.copy(img)  # so we don't modify the original
        image2 = cv.rectangle(image2, rect_TpLft, rect_BtmRght, rect_color,
                              rect_thickness)
        # 2. Draw circle
        rect_center = (int(rect_BtmRght[0] / 2), int((rect_TpLft[1] + height) / 2))
        circle_rad = 20
        circle_color = (255, 0, 255)
        circle_thickness = 5
        image2 = cv.circle(image2, rect_center, circle_rad, circle_color, circle_thickness)
        # print(rect_TpLft, rect_center, rect_BtmRght)
        # print(f'Center BGR is: {img[rect_center[-1], rect_center[0]]}')

        # 3. Define subregion and relevant dimensions

        subregion = img[int(7 * height / 8):height, 0:int(width / 4)]
        sub_height, sub_width, _ = subregion.shape
        # sub_center = subregion[int(sub_height / 2), int(sub_width / 2)]
        # print(sub_center)

        # 4. Apply masks & 5. bitwise AMD
        masked_imgs = img_mask(subregion)

        # 6. Check center pixel and surrounding pixels to determine color sum

        i = 0  # Track which image has the color
        # Define center
        y, x = int(sub_height / 2), int(sub_width / 2)
        color_detected = False
        for im in masked_imgs:
            im_center = im[y, x]
            # reach surrounding pixels from center

            im_area = np.array([im[y - 1, x - 1], im[y - 1, x], im[y - 1, x + 1],
                                im[y, x - 1], im_center, im[y, x + 1],
                                im[y + 1, x - 1], im[y + 1, x], im[y + 1, x + 1]])
            num_detections = 0
            for pixel in im_area:
                if np.sum(pixel) > 0:
                    num_detections += 1
                # print(f'Pixels with color on Image {i}: {num_detections}')
            if num_detections == len(im_area):  # do all pixels have a non-black color?
                # Color found
                color_detected = True
                break
            i += 1
        if color_detected:
            consec_detections += 1
            print(f'PYTHON: Consecutive Detections: {consec_detections}.')
        else:
            # print('PYTHON: No color found.')
            consec_detections = 0
        subregion = cv.circle(subregion, (x, y), 15, circle_color, 5)

        # cv.imshow("Original", img)
        # cv.imshow("Rectangle", image2)
        cv.imshow("Subregion", subregion)
        cv.moveWindow("Subregion", 0, 0)
        cv.waitKey(1)
        if consec_detections >= DETECTION_THRESH:
            print(f'Color detected on Image {i}. Exiting.')
            return i


def send_color(pic_num: int, serial_line):
    
    if pic_num >= 0 and pic_num <= 3:
        byteToSend = str(pic_num).encode()
        try:
            serial_line.write(byteToSend)
        except serial.SerialException:
            handleSerErr()
        sleep(SERIAL_SLEEP)
        print(f"Byte Sent: {byteToSend}\n")
        sleep(3)  # Give human chance to see output
    else:
        print("Invalid data.\n")


#----Program Setup----#

# Camera Variables
SATURATION_LOWER = 50
SATURATION_MAX = 255  # Max 'S' value in HSV
BRIGHTNESS_LOWER = 20
BRIGHTNESS_MAX = 255  # Max 'V' Value in HSV
DETECTION_THRESH = 110  # Number of consecutive detects required to be sure the color is present.

# Set up webcam (laptop)
cap = cv.VideoCapture(0)

# Serial Variables
BAUD_RATE = 19200
SERIAL_TIMEOUT = 5  # (in seconds)
serial_connected = False

#----Begin Program---#
while not serial_connected:
    try:
        arduino = serial.Serial(port='COM10', baudrate=BAUD_RATE, timeout=SERIAL_TIMEOUT)
        serial_connected = True
    except serial.SerialException:
        print("Waiting for COM to open.")
        sleep(2)
       
SERIAL_SLEEP = 0.05  # HOW LONG (in seconds) to delay

# Useful booleans        
arduino_ready = False
program_started = False

# Wait for Arduino to claim readiness
while not program_started:
    print("PYTHON: Awaiting Start signal...")
    try:
        if arduino.in_waiting > 0:
            dummy = arduino.read(24)
            dummy = dummy.decode(encoding='ascii')
            print(dummy)
            if dummy == 'LYNX: Associate Colors.\n':
                program_started = True
                print("PYTHON: Starting Program.")
    except serial.SerialException:
        handleSerErr()
    sleep(2)

# Enter color detection/transmission loop
while True:
    if not arduino_ready:
        # check for serial data
        try:
            if arduino.in_waiting > 0:
                check_data = arduino.read(1)
                check_data = check_data.decode(encoding='ascii')
                if check_data == 'I':
                    arduino_ready = True
                elif check_data == '9':
                    print("PYTHON: Program Exit Received. Exiting...")
                    break
        except serial.SerialException:
            handleSerErr()
    else:
        # color_detect
        print("PYTHON: Entering Color Detection")
        pic_num = colorDetect()
        send_color(pic_num, arduino)
        arduino_ready = False
