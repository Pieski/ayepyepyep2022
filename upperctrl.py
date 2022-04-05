import serial
import time
import threading
import numpy as np
import cv2 as cv
import os
import time

global cam
global net
global frameIndex
global ARMser
global MCUser

WAKE_BYTE = 0x2B
STAGE_A_BYTE = 0xA0
STAGE_B_BYTE = 0xB0
OVR_BYTE = 0x2D

CONFIDENCE = 0.5  # 过滤弱检测的最小概率
THRESHOLD = 0.4  # 非最大值抑制阈值

cmd_high_center = ((1,2000),(2,1529),(3,2833),(4,2595),(5,1502))
cmd_high_left = ((1,2457),(2,1646),(3,2684),(4,2704))
cmd_high_right = ((1,1486),(2,1762),(3,2681),(4,2604))
cmd_high_exce = ((1,2041),(2,1392),(3,2513),(4,2446))

workingPath = '/home/pi/zk2022/'
weightsPath = os.path.join(workingPath, 'yolov3.weights') 
configPath = os.path.join(workingPath, 'yolov3-voc.cfg')
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

def ARMrotate(cmd,spd,delay=0):
	for i in range(0,len(cmd)):
		ARMwriteCMD(cmd[i][0],0x03,[0x2A,cmd[i][1],spd],5)
		time.sleep(delay)

def ARMlocate(_id):
	global ARMser
	ARMwriteCMD(_id,0x02,[0x38,0x02],2)
	buf = ARMser.read(8)
	return (buf[2],buf[4],(buf[5] << 8) | buf[6])

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

def loadNet():
	global net
	net = cv.dnn.readNetFromDarknet(configPath, weightsPath)

def best():
	global cam
	global net
	ret, frame = cam.read()
	(H,W) = frame.shape[:2]
	blob = cv.dnn.blobFromImage(frame, 1.0/255.0, (224, 224), None, True, False)
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

	return classIDs[np.argmax(confidences)]

def besttest():
	global cam
	global net
	frame = cv.imread("/home/pi/zk2022/test.jpg")
	(H,W) = frame.shape[:2]
	blob = cv.dnn.blobFromImage(frame, 1.0/255.0, (320, 320), None, True, False)
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

	return classIDs[np.argmax(confidences)]

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


#ARMconnect()
#MCUconnect()
CAMconnect()
loadNet()

print("best = " + str(best()))

print("input '+' to start...")
while(input() != '+'):
	pass

MCUsendbyte(WAKE_BYTE)

while(True):
	if(MCUwaituntil(STAGE_A_BYTE) == -1):
		pass

	print("STAGE_A, input + to continue...")
	while(input() != '+'):
		pass

	#STAGE A
	MCUsendbyte(WAKE_BYTE)

	if(MCUwaituntil(STAGE_B_BYTE) == -1):
		pass

	print("STAGE_B, input + to continue...")
	while(input() != '+'):
		pass

	#STAGE B
	MCUsendbyte(WAKE_BYTE)