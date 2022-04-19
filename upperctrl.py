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

WAKE_BYTE = 0x2B
QUIT_BYTE = 0x2C
STAGE_A_BYTE = 0xA0
STAGE_B_BYTE = 0xB0
OVR_BYTE = 0x2D

CONFIDENCE = 0.2
THRESHOLD = 0.4

workingPath = '/home/pi/zk2022/'
weightsPath = os.path.join(workingPath, 'yolov3-tiny.weights') 
configPath = os.path.join(workingPath, 'yolov3-tiny.cfg')
labelsPath = os.path.join(workingPath, 'voc.names') 
logPath = os.path.join(workingPath, 'log')

frameIndex = 0

def ARMconnect():
	global ARMser
	ARMser = serial.Serial('/dev/ttyAMA0',115200,timeout=1)

def ARMwritebyte(s,dat,len=1):
	s.write(dat.to_bytes(len,'big'))

def ARMwriteCMD(_id, _type, dat, d_length):
	global ARMser
	ckc=0
	print("write:",ARMwritebyte(ARMser,0xFF))
	ARMwritebyte(ARMser,0xFF)
	print("id=%d,write:%d",(_id,ARMwritebyte(ARMser,_id)))
	ARMwritebyte(ARMser,d_length+2)
	ARMwritebyte(ARMser,_type)
	ckc += _id + d_length+2 + _type

	for i in range(0,len(dat)):
		if(dat[i] <= 255):
			ARMwritebyte(ARMser,dat[i])
			print("\033[0;33;40 -> Arm:", dat[i], "\033[0m")
			ckc += dat[i]
		else:
			ARMwritebyte(ARMser,dat[i],2)
			print("\033[0;33;40 -> Arm:", dat[i]>>8," ",dat[i]&0xFF, "\033[0m")
			ckc += dat[i]>>8
			ckc += dat[i]&0xFF

	ARMwritebyte(ARMser,(~ckc)&0xFF)

def ARMrotate(id,pos,spd,delay=0):
	ARMwriteCMD(id,0x03,[0x2A,pos,spd],5)
	time.sleep(delay)

def ARMlocate(_id):
	global ARMser
	ARMwriteCMD(_id,0x02,[0x38,0x02],2)
	buf = ARMser.read(8)
	return (buf[2],buf[4],(buf[5] << 8) | buf[6])

def ARMset(pos):
	for i in range(0,len(pos)):
		ARMrotate(pos[i][0],pos[i][1],pos[i][2],0.01)

def ARMenforce(_id,stat,_all=0):
	if(_all != 0):
		for i in range(0,6):
			ARMenforce(i,stat,_all=0)
	else:
		ARMwriteCMD(_id,0x03,[0x28,stat],2)

def ARMblock(_id,_all=0):
	global ARMser
	if(_all != 0):
		for i in range(0,6):
			if(ARMblock(i,_all=0) == 1):
				return 1
		return 0
	else:
		ARMwriteCMD(_id,0x01,[],0)
		buf = ARMser.read(6)
		if(buf[4] &= 0x20 != =):
			return 1
		return 0

def ARMliedown():
	ARMset([(0,2400,1000),(1,2000,1000),(2,1300,2000),(3,2000,2000),(4,2000,2000)])
	time.sleep(2.1)
	ARMenforce(0,0,_all=1)

def MCUconnect():
	global MCUser
	try:
		MCUser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
	except:
		try:
			MCUser = serial.Serial('/dev/ttyUSB1',115200,timeout=1)
		except:
			try:
				MCUser = serial.Serial('/dev/ttyUSB2',115200,timeout=1)
				return True
			except:
				return False

def MCUsendbyte(dat):
	global MCUser
	if(type(dat) == int):
		msg = dat.to_bytes(1,'big')
	else:
		msg = ord(dat).to_bytes(1,'big')

	print("\033[0;32;40m -> MCU:", msg[0], "\033[0m")
	return MCUser.write(msg)

def MCUwaituntil(s):
	global MCUser
	while(True):
		readbyte = MCUser.read()
		if(len(readbyte) > 0):
			print(readbyte[0])
			if(readbyte[0] == s):
				return readbyte
			elif(readbyte[0] == OVR_BYTE):
				return -1

def CAMconnect():
	global cam
	cam = cv.VideoCapture(0)
	cam.set(3,1280)
	cam.set(4,720)

def loadNet():
	global net
	net = cv.dnn.readNetFromDarknet(configPath, weightsPath)

def best():
	global cam
	global net
	global frameIndex

	ret, frame = cam.read()
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

ARMconnect()
MCUconnect()
CAMconnect()
loadNet()
GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.OUT)
GPIO.output(4,False)

#print("best = " + str(best()))

print("input + to turn power on...")
while(input() != '+'):
	pass
GPIO.output(4,True)

print("input '+' to start...")
while(input() != '+'):
	pass

MCUsendbyte(WAKE_BYTE)

while(True):
	layout = [-1,-1,-1,-1,-1,-1]
	if(MCUwaituntil(STAGE_A_BYTE) == -1):
		pass

	print("STAGE_A, input + to continue...")
	while(input() != '+'):
		pass

	#上方拍照
	ARMenforce(0,1,_all=1)
	ARMset([(0,2860,1000),(1,2000,1000),(2,1945,1500),(3,1975,1500),(4,3250,1500)])
	time.sleep(2.2)
	layout[0:3] = best()
	#分割识别1

	#下方拍照
	ARMset([(2,1170,1000),(3,3340,1000),(4,2950,1000)])
	time.sleep(2.2)
	layout[3:6] = best()
	#分割识别2

	print("following placement detected: ",end="")
	for i in range(0,6):
		if(layout[i] != -1):
			print(str(i) + " ",end="")
	print("")

	ARMset([(0,2400,1000),(1,2000,1000),(2,1070,1500),(3,3350,1500),(4,3000,1500)])
	time.sleep(1.6)
	if(layout[4] != -1):
		ARMset([(0,2400,1000),(1,2000,1000),(2,2600,2000),(3,3200,2000),(4,1220,2000)])
		time.sleep(2.1)
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(2,1360,1500),(3,3560,1500),(4,2450,1500)])
		time.sleep(1.6)
		ARMset([(1,1500,1000)])
		time.sleep(1.1)
		ARMset([(1,2000,2500),(3,2000,2000),(4,2000,2000)])
		time.sleep(2.1)
		if(ARMblock(0,_all=1) != 0):
			MCUsendbyte(QUIT_BYTE)
			ARMliedown()
			break
		ARMset([(0,2400,1000)])
		time.sleep(1.1)
		if(layout[3] != -1 or layout[5] != -1):
			ARMset([(1,2000,1000),(2,1070,1500),(3,3350,1500),(4,3000,1500)])
			time.sleep(1.6)

	if(layout[3] != -1):
		ARMset([(0,2400,1000),(1,2300,1000),(2,2600,2000),(3,3200,2000),(4,1220,2000)])
		time.sleep(2.1)
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(1,2000,3500),(2,1360,1500),(3,3560,1500),(4,2450,1500)])
		time.sleep(1.6)
		ARMset([(3,2000,2000),(4,2000,2000)])
		time.sleep(2.1)
		if(ARMblock(0,_all=1) != 0):
			MCUsendbyte(QUIT_BYTE)
			ARMliedown()
			break
		ARMset([(0,2400,1000)])
		time.sleep(1.1)
		if(layout[5] != -1):
			ARMset([(1,2000,1000),(2,1070,1500),(3,3350,1500),(4,3000,1500)])
			time.sleep(1.6)

	if(layout[5] != -1):
		ARMset([(0,2400,1000),(1,1700,1000),(2,2600,2000),(3,3200,2000),(4,1220,2000)])
		time.sleep(2.1)
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(1,2000,3500),(2,1360,1500),(3,3560,1500),(4,2450,1500)])
		time.sleep(1.6)
		ARMset([(3,2000,2000),(4,2000,2000)])
		time.sleep(2.1)
		if(ARMblock(0,_all=1) != 0):
			MCUsendbyte(QUIT_BYTE)
			ARMliedown()
			break
		ARMset([(0,2400,1000)])
		time.sleep(1.1)

	ARMliedown()
	MCUsendbyte(WAKE_BYTE)

	if(MCUwaituntil(STAGE_B_BYTE) == -1):
		pass

	print("STAGE_B, input + to continue...")
	while(input() != '+'):
		pass

	ARMenforce(0,1,_all=1)
	ARMset([(0,2400,1000),(1,2000,1000),(2,1350,1500),(3,2000,1000),(4,3410,1500)])
	time.sleep(1.6)
	if(layout[1] != -1):
		ARMset([(2,1900,2000),(3,2000,2000),(4,3100,2000),(0,2400,1000)])
		time.sleep(2.1)
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(2,1350,1000),(4,3410,1000)])
		time.sleep(1.1)
		ARMset([(2,1150,1500),(4,2000,1500)])
		time.sleep(1.6)
		ARMset([(0,2400,1000)])
		time.sleep(1.1)
		if(layout[0] != -1 or layout[2] != -1):
			ARMset([(2,1350,1500),(4,3410,1500)])
			time.sleep(1.6)
			if(ARMblock(0,_all=1) != 0):
				MCUsendbyte(QUIT_BYTE)
				ARMliedown()
				break

	if(layout[0] != 1):
		ARMset([(1,2300,2000),(2,1900,2000),(4,3100,2000)])
		time.sleep(2.1)
		if(ARMblock(0,_all=1) != 0):
			MCUsendbyte(QUIT_BYTE)
			ARMliedown()
			break
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(1,2000,1500),(2,1350,1000),(4,3410,1000)])
		time.sleep(1.1)
		ARMset([(2,1150,1500),(4,2000,1500)])
		time.sleep(1.6)
		ARMset([(0,2400,1000)])
		time.sleep(1.1)
		if(layout[2] != -1):
			ARMset([(2,1350,1500),(4,3410,1500)])
			time.sleep(1.6)
			if(ARMblock(0,_all=1) != 0):
				MCUsendbyte(QUIT_BYTE)
				ARMliedown()
				break

	if(layout[2] != -1):
		ARMset([(1,1700,2000),(2,1900,2000),(4,3100,2000)])
		time.sleep(2.1)
		if(ARMblock(0,_all=1) != 0):
			MCUsendbyte(QUIT_BYTE)
			ARMliedown()
			break
		ARMset([(0,2750,1000)])
		time.sleep(1.1)
		ARMset([(1,2000,1500),(2,1350,1000),(4,3410,1000)])
		time.sleep(1.1)
		ARMset([(2,1150,1500),(4,2000,1500)])
		time.sleep(1.6)
		ARMset([(0,2400,1000)])
		time.sleep(1.1)
	
	ARMliedown()
	MCUsendbyte(WAKE_BYTE)