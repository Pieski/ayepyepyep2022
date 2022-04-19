import serial
import time
import threading
import numpy as np
import cv2 as cv
import os
import time
import RPi.GPIO as GPIO

global cam
global net
global frameIndex
global ARMser
global MCUser

CONFIDENCE = 0.2
THRESHOLD = 0.4

workingPath = '/home/pi/zk2022/'
weightsPath = os.path.join(workingPath, 'yolov3-tiny.weights') 
configPath = os.path.join(workingPath, 'yolov3-tiny.cfg')
labelsPath = os.path.join(workingPath, 'voc.names') 
logPath = os.path.join(workingPath, 'log')

frameIndex = 0
def CAMconnect():
	global cam
	cam = cv.VideoCapture(0)
	cam.set(3,1280)
	cam.set(4,720)

def loadNet():
	global net
	net = cv.dnn.readNetFromDarknet(configPath, weightsPath)

def besttest():
	global net
	global frameIndex

	ret, frame = cv.imread("/home/pi/zk2022/test.jpg")
	subframe = [frame[0:720,0:500],frame[0:720,390:890],frame[0:720,780:1280]]
	result = [-1,-1,-1]

	for i in range(0,3):
		(H,W) = subframe[i].shape[:2]
		print("width = " + str(W))
		blob = cv.dnn.blobFromImage(subframe[i], 1.0/255.0, (224, 224), None, True, False)
		net.setInput(blob)

		outInfo = net.getUnconnectedOutLayersNames() 
		start = time.time()
		layerOutputs = net.forward(outInfo)
		end = time.time()
		print("time used: " + str(end-start) + "s")
		confidences = []
		classIDs = []

		for out in layerOutputs:
			for detection in out:
				scores = detection[5:]
				classID = np.argmax(scores)
				confidence = scores[classID]
				if(confidence > CONFIDENCE):
					confidences.append(float(confidence))
					classIDs.append(classID)

		if(len(classIDs) > 0):
			result[i] = classIDs[np.argmax(confidences)]
		else:
			result[i] = -1

	return result

def getFrame():
	global frameIndex
	global cam
	ret, frame = cam.read()
	savepath = os.path.join(workingPath, (str(frameIndex) + '.jpg'))
	cv.imwrite(savepath, frame)

def record(index, img, confidences, classIDs):
	logfile = open(logPath, 'a')
	logfile.write(str(index) + '.jpg :\n')
	for i in range(len(classIDs)):
		logfile.write('\t' + 'ID=' + classIDs[i] + ' confidence=' + confidences[i] + '\n')
	logfile.close()

	savepath = os.path.join(workingPath, (str(index) + 'jpg'))
	cv.imwrite(img, savepath)

loadNet()

print("best = " + str(besttest()))