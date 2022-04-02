import serial
import time
import threading
import RPi.GPIO

cmd_high_center = ((1,2000),(2,1529),(3,2833),(4,2595),(5,1502))
cmd_high_left = ((1,2457),(2,1646),(3,2684),(4,2704))
cmd_high_right = ((1,1486),(2,1762),(3,2681),(4,2604))
cmd_high_exce = ((1,2041),(2,1392),(3,2513),(4,2446))

wbuf = []

threadLock = threading.Lock()

def writebyte(s,dat,len=1):
	s.write(dat.to_bytes(len,'big'))

def write_cmd(_id, _type, dat, d_length):
	ckc=0
	print("write:",writebyte(ser,0xFF))
	writebyte(ser,0xFF)
	print("id=%d,write:%d",(_id,writebyte(ser,_id)))
	writebyte(ser,d_length+2)
	writebyte(ser,_type)
	ckc += _id + d_length+2 + _type

	for i in range(0,len(dat)):
		if(dat[i] <= 255):
			print("write ", dat[i])
			writebyte(ser,dat[i])
			ckc += dat[i]
		else:
			writebyte(ser,dat[i],2)
			print("write ", dat[i]>>8," ",dat[i]&0xFF)
			ckc += dat[i]>>8
			ckc += dat[i]&0xFF

	writebyte(ser,(~ckc)&0xFF)

def rotate(cmd,spd,delay=0):
	for i in range(0,len(cmd)):
		write_cmd(cmd[i][0],0x03,[0x2A,cmd[i][1],spd],5)
		time.sleep(delay)

def writebyte(s,dat):
	if(type(dat) == int):
		return s.write(dat.to_bytes(1,'big'))
	else:
		print("char written: ",ord(dat))
		return s.write(ord(dat).to_bytes(1,'big'))

class monitor_thread(threading.Thread):

	def __init__(self,ser,lock):
		threading.Thread.__init__(self)
		self.ser = ser
		self.lock = lock

	def run(self):
		while(True):
			self.lock.acquire()
			if(self.ser.in_waiting != 0):
				print("\033[0;33;40m",end="")
				print(self.ser.read(self.ser.in_waiting),end="")
				print("\033[0m",end="")
			if(len(wbuf) > 0):
				print(" -> ",end="")
				for i in range(len(wbuf)):
					print(writebyte(self.ser, wbuf[0]),"bytes written")
					print(wbuf[0]," ",end="")
					del wbuf[0]
				print("\n",end="")
			self.lock.release()
			time.sleep(1)

ser = 0
try:
	ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
except:
	ser = serial.Serial('/dev/ttyUSB1',115200,timeout=1)
mthread = monitor_thread(ser,threadLock)
mthread.start()

while(True):
	cmd = input()
	try:
		cmd = int(cmd)
	except:
		pass
	threadLock.acquire()
	if(type(cmd) == str):
		for i in range(len(cmd)):
			wbuf.append(cmd[i])
	else:
		wbuf.append(cmd)
	threadLock.release()