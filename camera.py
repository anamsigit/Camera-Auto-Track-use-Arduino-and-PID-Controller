# jika tidak bisa membuat baris baru, gunakan alt + shift + panah saja


# OPENCV
from simple_pid import PID
from cv2 import CascadeClassifier as CC
from collections import deque
from turtle import update
import numpy as np
import argparse
import imutils
import cv2
import serial
import time
import math

# Konfigurasi port Serial
# ser = serial.Serial('COM9', 9600)  # Ganti dengan nama port Serial yang sesuai
# time.sleep(5)

coords = [(1,1), (2,2), (3, 3), (4, 4), (5, 5), (6,6), (7, 7), (8, 8), 
          (9, 9), (10, 10), (11, 11), (12, 12), (13, 13), (14, 14), (15, 15), (16, 16), (17,17), (18, 18),
          (19,19), (20,20), (21,21), (23, 23), (24, 24), (25, 25), (26, 26), (27, 27), (28, 28), (29, 29),
          (30, 30),(31, 31), (32, 32), (33, 33), (34, 34), (35, 35), (36, 36),  (37, 37), (38, 38), (39, 39),
          (40, 40), (41, 41), (42, 42), (43, 43), (44, 44), (45, 45), (46,46), (47, 47), (48, 48), (49,49),
          (50, 50), (51, 51), (52, 52), (53,53), (54,54), (55, 55), (56,56), (57,57), (58,58), (59,59),
          (60,60), (61,61), (62,62), (63,63), (64,64), (65,65), (66,66), (67,67), (68,68), (69,69), (70, 70),
          (71,71), (72,72), (73,73), (74,74), (75,75), (76,76), (77,77), (78,78), (79,79), (80,80), (81,81),
          (82,82), (83,83), (84,84), (85,85), (86,86), (87,87), (88,88), (89,89), (90,90)]
n = 1

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (133, 54, 99)
greenUpper = (255, 255, 255)

# greenLower = (0, 0, 0)
# greenUpper = (0, 0, 0)

pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
else:
	camera = cv2.VideoCapture(args["video"])


# PID

ujisifat = []

# start dimana koordinat menyimpang dari nilai koordinat jalur yang benar
koorpenyimpanganjalurx = 23
koorpenyimpanganjalury = 60

# variabel menyimpan koordinat terkini yang seiring akan dikurang setiap satuan waktu oleh nilaix.update
nilaix = {'evaluasi': koorpenyimpanganjalurx}
nilaiy = {'evaluasi': koorpenyimpanganjalury}


def pidcontrolx(errorx):
    # satu fungsi hanya berlaku untuk 1 error, tidak bisa multi
    pidx = PID(0.25, 0.007, 0.0066, setpointx, sample_time=0.5)
    # pidx = PID(0.5, 0.000, 1.6, setpointx, sample_time=0.5)

    koreksix = pidx(errorx)

    if errorx == setpointx:
        # print(errorx,'==',setpointx,'koordinat x dalam jalur yang benar')
        evaluasi = errorx + koreksix  # disinlah letak pengurangan error oleh koreksi
    else:
        # print(errorx, "akan dikurangi sebanyak", koreksix, "setiap satuan waktu hingga diperoleh nilai", setpointx)
        evaluasi = errorx + koreksix

    return evaluasi


def pidcontroly(errory):
    # satu fungsi hanya berlaku untuk 1 error, tidak bisa multi
    pidy = PID(0.25, 0.007, 0.0066, setpointy, sample_time=0.5)

    koreksiy = pidy(errory)  # pid menghitung kesalahan

    if errory == setpointy:
        # print(errory,'==',setpointy,'koordinat y dalam jalur yang benar')
        evaluasi = errory + koreksiy  # disinlah letak pengurangan error oleh koreksi

    else:
        # print(errory, "akan dikurangi sebanyak", koreksiy, "setiap satuan waktu hingga diperoleh nilai", setpointy)
        evaluasi = errory + koreksiy

    return evaluasi

def HitungJarak(titik1, titik2):
    x1, y1 = titik1
    x2, y2 = titik2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()

	if args.get("video") and not grabbed:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                        cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)

		((x, y), radius) = cv2.minEnclosingCircle(c)

		M = cv2.moments(c)

		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

			# PID
			# tengahx = int((int(x)+int(x2))/2)
			# tengahy = int((int(y)+int(y2))/2)

			tengahx = int(center[0])
			tengahy = int(center[1])

			setpointx = tengahx
			setpointy = tengahy

			# cv2.circle(frame, (tengahx, tengahy), int(20), (0, 255, 255), 2)
			intnilaix = int(nilaix['evaluasi'])
			nilaix.update({'evaluasi': pidcontrolx(intnilaix)})
			updatednilaix = nilaix["evaluasi"]

			intnilaiy = int(nilaiy['evaluasi'])
			nilaiy.update({'evaluasi': pidcontroly(intnilaiy)})
			updatednilaiy = nilaiy["evaluasi"]

			# print(updatednilaix)
			# print(updatednilaiy)
			# print(int(updatednilaiy))
			# print(type(int(updatednilaiy)))

			koordinatPID = int(updatednilaix), int(updatednilaiy)
			centerkoordinat = int(frame.shape[1]/2), int(frame.shape[0]/2)

			# cv2.circle(frame, (434, 3434), int(radius2), (0, 255, 255), 2) #argumen fungsi PID harus dinamis nilainya, cobalah dengan nilai statis dulu untuk pemahaman
			cv2.circle(frame, (koordinatPID), int(15), (30, 154, 70), -1)
			cv2.circle(frame, (centerkoordinat), int(5), (30, 154, 70), -1)
			cv2.line(frame, koordinatPID, centerkoordinat, (30, 154, 70), 1)
			jarak = str(int(HitungJarak(koordinatPID, centerkoordinat)))
			cv2.putText(frame, jarak, centerkoordinat, cv2.FONT_HERSHEY_SIMPLEX, 1, (30, 154, 70), 1)
			ujisifat.append(1)
            
			koorxrescaled = (koordinatPID[0] / 600) * 90
			kooryrescaled = (koordinatPID[1] / 600) * 90
			# print(koorxrescaled)
			koorxrescaled = int(koorxrescaled)
			kooryrescaled = int(kooryrescaled)
			koorxrescaled = str(koorxrescaled)
			kooryrescaled = str(kooryrescaled)
			sendingserial = f"{koorxrescaled},{kooryrescaled}\n"
			# ser.write(sendingserial.encode())
			print(sendingserial)
            # ---------> Test case
			n +=1
			# print(n)
			# message = str(coords[0+n][0]) + ',' + str(coords[0+n][1]) + '\n'
			# print(message)
			# ser.write(message.encode())
            # ---------> Test case

			# time.sleep(0.1)
			

			# print('jumlahnya apakah kontinuuuuuuu',len(ujisifat))

            # RESCALE



	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
print('selesai')