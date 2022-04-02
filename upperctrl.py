import serial
import time
import threading

cmd_high_center = ((1,2000),(2,1529),(3,2833),(4,2595),(5,1502))
cmd_high_left = ((1,2457),(2,1646),(3,2684),(4,2704))
cmd_high_right = ((1,1486),(2,1762),(3,2681),(4,2604))
cmd_high_exce = ((1,2041),(2,1392),(3,2513),(4,2446))

class Arm:
	def connect():
		self.ser = serial.Serial('/dev/ttyAMA0',115200,timeout=1)

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
				writebyte(ser,dat[i])
				print("\033[0;33;40 -> Arm:", dat[i], "\033[0m")
				ckc += dat[i]
			else:
				writebyte(ser,dat[i],2)
				print("\033[0;33;40 -> Arm:", dat[i]>>8," ",dat[i]&0xFF, "\033[0m")
				ckc += dat[i]>>8
				ckc += dat[i]&0xFF

		writebyte(ser,(~ckc)&0xFF)

	def rotate(cmd,spd,delay=0):
		for i in range(0,len(cmd)):
			write_cmd(cmd[i][0],0x03,[0x2A,cmd[i][1],spd],5)
			time.sleep(delay)

	def locate(_id):
		write_cmd(_id,0x02,[0x38,0x02],2)
		buf = ser.read(8)
		return (buf[2],buf[4],(buf[5] << 8) | buf[6])

class MCU:
	def connect():
		try:
			self.ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
		except:
			try:
				self.ser = serial.Serial('/dev/ttyUSB1',115200,timeout=1)
			except:
				try:
					self.ser = serial.Serial('/dev/ttyUSB2',115200,timeout=1)
					return True
				except:
					return False

	def sendbyte(dat):
		if(type(dat) == int):
			msg = dat.to_bytes(1,'big')
		else:
			msg = ord(dat).to_bytes(1,'big')

		print("\033[0;32;40 -> MCU:", msg[0], "\033[0m")
		return self.ser.write(msg)

	def waituntil(s):
		while(True):
			readbyte = self.ser.read()
			if(readbyte == s or readbyte == ocd(s)):
				return readbyte
			elif(readbyte == '-' or readbyte == ocd('-')):
				return -1

Arm.connect()
MCU.connect()

while(input() != '+'):
	pass

MCU.sendbyte('+')
time.sleep(0.2)
print("\033[0;37;41 -> MCU:", self.ser.read(self.ser.in_waiting), "\033[0m")

while(True):
	if(MCU.waituntil(0xA0) == -1):
		break

	if(MCU.waituntil(0xB0) == -1):
		break