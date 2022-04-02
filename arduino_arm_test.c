#include <SoftwareSerial.h>
#include <stdlib.h>
#include <string.h>
#define LOCK 1
#define UNLOCK 0
#define TX 4
#define RX 5
#define ROTATE 0xEE
#define LOCATE 0xDD
#define R_ROTATE 0xCC
#define REG 0xBB
#define TORQUE 0XAA
#define HIGH8(V) (V>>8&0xFF)
#define LOW8(V) (V&0xFF)

void reg();
void reg_rotate(byte,uint16_t,uint16_t);
void rotate(byte,uint16_t,uint16_t);
void locate(byte);
void write_cmd(byte,byte,byte*,uint16_t);

SoftwareSerial ws = SoftwareSerial(RX,TX);
byte buffer[2];

void setup(){
	pinMode(RX,INPUT_PULLUP);
  	Serial.begin(115200);
  	Serial.println("begin");
	ws.begin(115200);
}

void loop(){
	if(Serial.available() > 0){
		delay(100);
		byte head = Serial.read();
		if(head == ROTATE){
			byte id = Serial.read();
			Serial.readBytes(buffer,2);
			uint16_t dest = convert_byte(buffer);
			Serial.readBytes(buffer,2);
			uint16_t delay = convert_byte(buffer);
			rotate(id,dest,delay);
			Serial.write(0x01);
      		ws.flush();
		}
		else if(head == LOCATE){
			byte id = Serial.read();
			locate(id);
      		ws.flush();
		}
		else if(head == R_ROTATE){
			byte id = Serial.read();
			Serial.readBytes(buffer,2);
			uint16_t dest = convert_byte(buffer);
			Serial.readBytes(buffer,2);
			uint16_t delay = convert_byte(buffer);
			reg_rotate(id,dest,delay);
			Serial.write(0x01);
      		ws.flush();
		}
		else if(head == REG){
			reg();
			Serial.write(0x01);
			ws.flush();
		}
		else if(head == TORQUE){
			Serial.readBytes(buffer,2);
			set_torque(buffer[0],buffer[1]);
			Serial.write(0x01);
			ws.flush();
		}
	}
}

void set_torque(byte id,byte stat){
	byte cmd[1] = {stat};
	write_cmd(id,
		0x03,
		cmd,
		1);
}

void reg(){
	write_cmd(0xFE, 0x05, NULL, 0);
}

void reg_rotate(byte id, uint16_t dest, uint16_t delay){
	byte cmd[5] = {0x2A,HIGH8(dest),LOW8(dest),HIGH8(delay),LOW8(delay)};
	write_cmd(id,
		0x04,
		cmd,
		5);
}

void rotate(byte id, uint16_t dest, uint16_t delay){
	byte cmd[5] = {0x2A,HIGH8(dest),LOW8(dest),HIGH8(delay),LOW8(delay)};
	write_cmd(id,
		0x03,
		cmd,
		5);
}

void locate(byte id){
	byte cmd[2] = {0x38, 0x02};
	write_cmd(id,
		0x02,
		cmd,
		2);

	byte buf[8];
	int i=0;
	//while(ws.available() <= 0);
	ws.readBytes(buf,8);
	Serial.write(LOCATE);
	Serial.write(buf,8);
  	Serial.write(0x01);
}

void write_cmd(byte id, byte type, byte *dat, uint16_t d_length){
	pinMode(TX,OUTPUT);
	uint32_t ckc = 0;
	ws.write(0xFF);
	ws.write(0xFF);
	ws.write(id);
	ws.write(d_length+2);
	ws.write(type);
	ckc += id + d_length+2 + type;
	for(int i=0;i<d_length;++i){
		ws.write(dat[i]);
		ckc += dat[i];
	}
	ws.write((~ckc)&0xFF);
	pinMode(TX,INPUT_PULLUP);
}


uint16_t convert_byte(byte buf[]){
	return (buf[1] << 8) | buf[0];
}