#include <math.h>
#define DEBUG 0
//总状态机 可用于串口通信
#define STOP 0xAA
#define PULSE 0xBB
#define RUN 0xCC

//串口通信字节
#define WAKE_BYTE '+'
#define QUIT_BYTE 0x2C
#define STAGE_A_BYTE 0xA0
#define STAGE_B_BYTE 0xB0
#define OVR_BYTE '-'

//串口调试命令头
#define C_FWD 0x0A
#define C_REV 0x0B
#define C_LFT 0x0C
#define C_RGT 0x0D

//强制回程时间 ms
#define TIME_MAX 480000			//8min

//循迹导航模式
#define COURSE_FWD 0
#define COURSE_FWD_IGNORE 1
#define COURSE_REV 2

//左右调速PIN
#define PWM_LEFT_A 4
#define PWM_RGHT_A 5
#define PWM_LEFT_B 2
#define PWM_RGHT_B 3

//电机状态PIN
#define LEFT_POS_A 52
#define LEFT_NEG_A 53
#define RGHT_POS_A 51
#define RGHT_NEG_A 50
#define LEFT_POS_B 25
#define LEFT_NEG_B 24
#define RGHT_POS_B 26
#define RGHT_NEG_B 27

//红外PIN
#define LEFTSIDE 35
#define LEFT 34
#define CENTER 33
#define RIGHT 32
#define RIGHTSIDE 30
#define REV_LEFTSIDE 37
#define REV_LEFT 47
#define REV_RIGHT 46
#define REV_RIGHTSIDE 39

//超声PIN
#define HCL_TRIG 43
#define HCL_ECHO 42
#define HCR_TRIG 45
#define HCR_ECHO 44

//电磁铁继电器PIN
#define RELAY 41

//强制刹车时间
#define BRAKING 800

//左侧动力补偿
#define SPD_COMP 50

//基准速度
int SPD_STAND = 140;

//修正减速
int CORR_REDUCE = 110;

//基准倒行速度
int SPD_STAND_REV = 130;

//倒行修正减速
int CORR_REDUCE_REV = 120;

//转向速度
#define SPD_TURN_POS 180
#define SPD_TURN_NEG 80

//强制转向时间
#define TURNNING 600

//强制回正时间 170
#define TURNING_RETURN 50

//转向刹车时间
#define TURNING_BRAKE 1000

//步间刹车时间
#define BRAKE_STABLE 1000

//强制倒车时间（静止离开路口）
#define REVERSE 600

//强制倒车时间（初速度非零）
#define REVERSE_KEEP 300

//启动直行时间
#define START_FORWARD 500
 
//终止倒车时间
#define END_REVERSE 1000

//刹车距离(cm)
#define STOP_DIST_A 20.00
#define STOP_DIST_B 5.00

//吸合倒车时间
#define ATTACH_TIME 8000

//超声平滑次数
#define SOR_SMOOTH 3

//超声超时时间
#define TIMEOUT 10000

//超声允许误差
#define SONAR_DIFF_MAX 0.2

//超声修正转速
#define SOR_CORRECT_SPD 180

//超声刹车时间
#define SOR_BRAKE 100

//超声单步时间
#define SOR_STEP_TIME 80

/*状态机
	FWD：循迹直行
	STP：货架前动作
	LFT：左转
	RGT：右转
	OVR：终止（final step）
	REV：循迹倒车
	WAT：等待
	PUL：进入PULSE模式
	REF：基准航向设置（刹车）
	SOR：超声校准
	KEYNODE：关键节点标志
*/
#define FWD 0
#define STP 1
#define LFT 2
#define RGT 3
#define OVR 4
#define REV 5
#define WAT 6
#define PUL 7
#define REF 8
#define MAG 9
#define SOR 10
#define TREV 11
#define KEYNODE 0xFF

#define START_PROCEDURE {FWD,1},{RGT,1},{FWD,2},{REV,2},{LFT,1},{FWD,2},{REV,1},{TREV,3}
#define STORAGE_ON_RIGHT {RGT,1},{STP,1},{REV,2},{LFT,1}
#define STORAGE_ON_LEFT {LFT,1},{STP,1},{REV,2},{RGT,1}

void correct(int);
void forward();
void turn(int);						//阻塞
void brake(int);					//阻塞
void setlogic(int,int,int,int);
void writespeed(int,int);

int course(int);
void course_block(int);				//阻塞
void course_time(int,int);			//阻塞，非精确时间，非常不精确

double s_dist(int);					//读超声距离
void s_cali();						//超声对齐

//总状态机
int stat = RUN;

//比赛开始时间
long zerotime = 0;

//任务流程。第一索引标记一级步骤，第二索引标记步骤类型及重复次数，下面称一级步骤中重复执行的部分为二级步骤
//short procedure[][2] = {START_PROCEDURE,{FWD,1},{LFT,1},{FWD,2},STORAGE_ON_RIGHT,{FWD,1},{SOR,1},{REV,3},{FWD,1},{RGT,1},{FWD,2},{OVR,1}};
short procedure[][2] = {START_PROCEDURE,{FWD,1},{LFT,1},{FWD,3},
STORAGE_ON_LEFT,{FWD,2},
STORAGE_ON_LEFT,{FWD,2},
STORAGE_ON_LEFT,{FWD,1},{REV,1},{RGT,1},{FWD,2},{OVR,1}};
//short procedure[][2] = {{TREV,1},{OVR,1}};
short step_rp = 0;
short step = 0;
int procedure_len = 0;

//红外状态
short ir_map[5] = {1,1,1,1,1};
short ir_map_old[5] = {0,0,0,0,0};
short ir_map_rev[4] = {1,1,1,1};

//修正状态
short corr_stat = 0;

//倒车状态
short rev_flag = 0;

//货架前停车状态
short stop_flag = 0;

//全局速度
int speed = SPD_STAND;

//目标距离
double dist = 0.00;

void setup(){
	pinMode(LEFT_POS_A, OUTPUT);
	pinMode(LEFT_NEG_A, OUTPUT);
	pinMode(RGHT_NEG_A, OUTPUT);
	pinMode(RGHT_POS_A, OUTPUT);
	pinMode(LEFT_POS_B, OUTPUT);
	pinMode(LEFT_NEG_B, OUTPUT);
	pinMode(RGHT_NEG_B, OUTPUT);
	pinMode(RGHT_POS_B, OUTPUT);
	pinMode(PWM_LEFT_A, OUTPUT);
	pinMode(PWM_RGHT_A, OUTPUT);
	pinMode(LEFTSIDE, INPUT);
	pinMode(LEFT, INPUT);
	pinMode(CENTER, INPUT);
	pinMode(RIGHT, INPUT);
	pinMode(RIGHTSIDE, INPUT);
	pinMode(REV_LEFT,INPUT);
	pinMode(REV_RIGHT,INPUT);
	pinMode(REV_LEFTSIDE,INPUT);
	pinMode(REV_RIGHTSIDE,INPUT);
	pinMode(HCL_ECHO, INPUT);
	pinMode(HCL_TRIG, OUTPUT);
	pinMode(HCR_ECHO, INPUT);
	pinMode(HCR_TRIG, OUTPUT);
	pinMode(RELAY, OUTPUT);
	digitalWrite(HCL_TRIG, 0);
	digitalWrite(HCR_TRIG, 0);
	digitalWrite(RELAY, 0);
	setlogic(0,0,0,0);

	if(DEBUG == 0){
		Serial.begin(115200);
		while(Serial.read() != WAKE_BYTE);
	}

	//zerotime = millis();
	setlogic(1,0,1,0);

	for(int i=0;i<3;++i){
	    ir_map[0] = (short)digitalRead(LEFTSIDE);
	    ir_map[1] = (short)digitalRead(LEFT);
	    ir_map[2] = (short)digitalRead(CENTER);
	    ir_map[3] = (short)digitalRead(RIGHT);
	    ir_map[4] = (short)digitalRead(RIGHTSIDE);
  	}

  	writespeed(SPD_STAND,SPD_STAND);
  	delay(START_FORWARD);
}

void loop(){
	if(DEBUG == 1){
		Serial.print(digitalRead(REV_LEFTSIDE));
		Serial.print(digitalRead(REV_LEFT));
		Serial.print(digitalRead(REV_RIGHT));
		Serial.print(digitalRead(REV_RIGHTSIDE));
		return;
	}

	if(stat == RUN){
		//读红外状态
		ir_map[0] = (short)digitalRead(LEFTSIDE);
		ir_map[1] = (short)digitalRead(LEFT);
		ir_map[2] = (short)digitalRead(CENTER);
		ir_map[3] = (short)digitalRead(RIGHT);
		ir_map[4] = (short)digitalRead(RIGHTSIDE);

		if(rev_flag){
			ir_map_rev[0] = (short)digitalRead(REV_LEFTSIDE);
			ir_map_rev[1] = (short)digitalRead(REV_LEFT);
			ir_map_rev[2] = (short)digitalRead(REV_RIGHT);
			ir_map_rev[3] = (short)digitalRead(REV_RIGHTSIDE);
		}
	
		if(rev_flag == 1)
			setlogic(0,1,0,1);
		else
			setlogic(1,0,1,0);

		//如果在左转去货架的关键节点超时，则取消左转
		/*if(procedure[step][0] == KEYNODE){
			if(millis() - zerotime >= TIME_MAX){
				procedure_fold(step,5,procedure_len);
				procedure_len -= 5;
			}
		}*/

		//检测当前一级步骤是否已经重复执行完毕
		if(step_rp >= procedure[step][1]){

			//在开始下一步前进行特殊步骤的过渡
			//倒行 -> 转向：继续倒车离开路口，然后重新直行循迹到这个路口，再转向
			if(procedure[step][0] == REV && (procedure[step+1][0] == LFT || procedure[step+1][0] == RGT)){
				course_time(COURSE_REV, 600);
				brake(BRAKE_STABLE);
				setlogic(1,0,1,0);
				rev_flag = 0;
				course_block(COURSE_FWD);
			}
			//转向 -> 倒行：直行到下一个路口，再倒车回到这个路口，强制倒车离开该路口后再执行倒车
			if((procedure[step][0] == LFT || procedure[step][0] == RGT) && procedure[step+1][0] == REV){	
				course_block(COURSE_FWD);
				brake(BRAKE_STABLE);
				setlogic(0,1,0,1);
				rev_flag = 1;
				forward();
				delay(REVERSE);
				course_block(COURSE_REV);
				forward();
				delay(REVERSE_KEEP);
			}
			//直行转倒车 或 倒车转直行：刹车等待一段时间，防止机器倾覆
			else if((procedure[step][0] == REV && procedure[step+1][0] == FWD) ||
				(procedure[step][0] == FWD && procedure[step+1][0] == REV)){
				brake(BRAKE_STABLE);
			}
			step++;
			step_rp = 0;
		}

		//流程判断
		switch(procedure[step][0]){
			case FWD:
			rev_flag = 0;
			if(course(COURSE_FWD))		//检测到路口则step_rp++，下同
				step_rp ++;
			break;

			case LFT:
			rev_flag = 0;
			turn(1);
			step_rp++;
			break;

			case RGT:
			rev_flag = 0;
			turn(0);
			step_rp++;
			break;

			case STP:
			digitalWrite(HCR_TRIG,HIGH);
			delayMicroseconds(10);
			digitalWrite(HCR_TRIG,LOW);
			dist = (double)pulseIn(HCR_ECHO, HIGH, TIMEOUT) / 58.0;
			if(stop_flag == 0){
				if(dist <= STOP_DIST_A && dist > 0.10){
					brake(BRAKING);
					s_cali();
					Serial.write(STAGE_A_BYTE);
					char byte = Serial.read();
					while(byte != WAKE_BYTE && byte != QUIT_BYTE)
						byte = Serial.read();
					if(byte = QUIT_BYTE){		//接受QUIT信号后直接退出
						stop_flag = 0;
						step_rp ++;
						break;
					}
					stop_flag = 1;
					break;
				}
			}else if(stop_flag == 1){
				if(dist <= STOP_DIST_B && dist > 0.10){
					brake(BRAKING);
					s_cali();
					Serial.write(STAGE_B_BYTE);
					char byte = Serial.read();
					while(byte != WAKE_BYTE && byte != QUIT_BYTE)
						byte = Serial.read();
					stop_flag = 0;
					step_rp ++;
					break;
				}
			}
			course(COURSE_FWD_IGNORE);
			break;

		    case OVR:
		    rev_flag = 0;
		    stat = STOP;
		    break;

			case REV:
			if(!rev_flag)
				setlogic(0,1,0,1);
			rev_flag = 1;
			if(course(COURSE_REV))
				step_rp ++;
			break;

			case WAT:
			brake(BRAKE_STABLE);
			step_rp ++;
			break;

			case SOR:
			s_cali();
			step_rp++;
			break;

			case TREV:
			SPD_STAND = 170;
			CORR_REDUCE = 140;
			SPD_STAND_REV = 150;
			CORR_REDUCE_REV = 130;
			setlogic(0,1,0,1);
			rev_flag = 1;
			digitalWrite(RELAY,1);
			course_time(COURSE_REV,2000);
			step_rp++;
			break;
		}

		for(int i=0;i<5;++i)
			ir_map_old[i] = ir_map[i];
	}

	else if(stat == PULSE){
		delay(1000);
		return;
	}

	//如果当前状态非运转也非暂停，则默认状态为停机。刹车，关闭电磁铁，进入死循环。
	else{
		setlogic(0,0,0,0);
		speed=0;
		while(1)
			delay(1000);
	}
}

/*	Serial.print("detected = ");
	int recv = Serial.read();
	Serial.print(recv);
	if(recv == RUN || recv == PULSE || recv == STOP){
		stat = recv;
		Serial.print("status changed");
		return;
	}

	if(recv == C_FWD || recv == C_REV || recv == C_LFT || recv == C_RGT)
		stat = PULSE;
	else 
		return;

	if(recv == C_FWD)
		course_block(COURSE_FWD);
	else if(recv == C_REV)
		course_block(COURSE_REV);
	else if(recv == C_LFT)
		turn(0);
	else if(recv == C_RGT)
		turn(1);

	brake(500);
}*/

//设定轨迹修正 stat=0：向左修正
void correct(int stat){
	if(!rev_flag){
		if(stat == 0){
			writespeed(speed - CORR_REDUCE, speed);
			return;
		}
		writespeed(speed, speed - CORR_REDUCE);
	}else{
		if(stat == 0){
			writespeed(SPD_STAND_REV - CORR_REDUCE, SPD_STAND_REV);
			return;
		}
		writespeed(SPD_STAND_REV, SPD_STAND_REV - CORR_REDUCE);
	}
}

void forward(){
	if(!rev_flag)
		writespeed(speed,speed);
	else
		writespeed(SPD_STAND_REV,SPD_STAND_REV);
}

//转向（堵塞） stat=1：左转
void turn(int stat){
	writespeed(0,0);

	//m_update(stat);

	if(stat){
		setlogic(0,1,1,0);
		writespeed(SPD_TURN_NEG, SPD_TURN_POS);
	}else{
		setlogic(1,0,0,1);
		writespeed(SPD_TURN_POS, SPD_TURN_NEG);
	}

	delay(TURNNING);

	while(digitalRead(CENTER))
		delay(1);

	delay(TURNING_RETURN);

	writespeed(0,0);
	brake(TURNING_BRAKE);

	writespeed(0,0);
	setlogic(1,0,1,0);
}

//刹车
void brake(int time){
	writespeed(0,0);
	setlogic(0,0,0,0);
	delay(time);
	if(!rev_flag)
		setlogic(1,0,1,0);
	else
		setlogic(0,1,0,1);
}

void setlogic(int lp, int ln, int rp, int rn){
	digitalWrite(LEFT_POS_A,lp);
	digitalWrite(LEFT_POS_B,lp);
	digitalWrite(LEFT_NEG_A,ln);
	digitalWrite(LEFT_NEG_B,ln);
	digitalWrite(RGHT_POS_A,rp);
	digitalWrite(RGHT_POS_B,rp);
	digitalWrite(RGHT_NEG_A,rn);
	digitalWrite(RGHT_NEG_B,rn);
} 

void writespeed(int l, int r){
	analogWrite(PWM_LEFT_A,l);
	analogWrite(PWM_LEFT_B,l);
	analogWrite(PWM_RGHT_A,r);
	analogWrite(PWM_RGHT_B,r);
}

//循迹导航，返回1：检测到路口
int course(int mode){
	if(mode!=COURSE_FWD_IGNORE)
		if((ir_map[0] == 0 || ir_map[4] == 0) && (ir_map_old[0] == 1 && ir_map_old[4] == 1))
			return 1;

	if(mode != COURSE_REV){
		if(ir_map[1] == 0 && ir_map[3] == 0 /*&& ir_map[2] == 0*/)
			correct(corr_stat);
		else if(ir_map[1] == 0 || ir_map[3] == 0){
			correct(ir_map[1]);
			corr_stat = ir_map[1];
		}
		else
			forward();
		return 0;
	}

	else if(mode == COURSE_REV){
		if(ir_map_rev[1] == 0 && ir_map_rev[2] == 0)
			forward();
		if(ir_map_rev[1] == 0 || ir_map_rev[2] == 0)
			correct(ir_map_rev[2]);
		else if(ir_map_rev[0] == 0 || ir_map_rev[3] == 0)
			correct(ir_map_rev[3]);
		else
			forward();
	}
	return 0;
}

//阻塞导航
void course_block(int mode){
	while(1){
		ir_map[0] = (short)digitalRead(LEFTSIDE);
		ir_map[1] = (short)digitalRead(LEFT);
		ir_map[2] = (short)digitalRead(CENTER);
		ir_map[3] = (short)digitalRead(RIGHT);
		ir_map[4] = (short)digitalRead(RIGHTSIDE);

		if(rev_flag){
			ir_map_rev[0] = (short)digitalRead(REV_LEFTSIDE);
			ir_map_rev[1] = (short)digitalRead(REV_LEFT);
			ir_map_rev[2] = (short)digitalRead(REV_RIGHT);
			ir_map_rev[3] = (short)digitalRead(REV_RIGHTSIDE);
		}

		if(course(mode))		//检测到路口
			return;

		for(int i=0;i<5;++i)
			ir_map_old[i] = ir_map[i];
	}
}

void course_time(int mode, int time){
	for(int i=0;i<time;++i){
		ir_map[0] = (short)digitalRead(LEFTSIDE);
		ir_map[1] = (short)digitalRead(LEFT);
		ir_map[2] = (short)digitalRead(CENTER);
		ir_map[3] = (short)digitalRead(RIGHT);
		ir_map[4] = (short)digitalRead(RIGHTSIDE);

		if(rev_flag){
			ir_map_rev[0] = (short)digitalRead(REV_LEFTSIDE);
			ir_map_rev[1] = (short)digitalRead(REV_LEFT);
			ir_map_rev[2] = (short)digitalRead(REV_RIGHT);
			ir_map_rev[3] = (short)digitalRead(REV_RIGHTSIDE);
		}

		if(course(mode))		//检测到路口
			return;

		for(int i=0;i<5;++i)
			ir_map_old[i] = ir_map[i];

		delay(1);
	}
}

//超声测距，side = 0：左侧
double _dist = 0.0;
double s_dist(int side,int smooth){
	_dist = 0.0;
	if(side == 0){
		for(int i=0;i<smooth;++i){
			digitalWrite(HCL_TRIG,HIGH);
			delayMicroseconds(10);
			digitalWrite(HCL_TRIG,LOW);
			_dist += (double)pulseIn(HCL_ECHO, HIGH, TIMEOUT) / 58.0;
		}
	}else{
		for(int i=0;i<smooth;++i){
			digitalWrite(HCR_TRIG,HIGH);
			delayMicroseconds(10);
			digitalWrite(HCR_TRIG,LOW);
			_dist += (double)pulseIn(HCR_ECHO, HIGH, TIMEOUT) / 58.0;
		}
	}
	return _dist/(double)smooth;
}

double distl,distr;
void s_cali(){
	distl = s_dist(0,SOR_SMOOTH);
	distr = s_dist(1,SOR_SMOOTH);
	while((double)abs(distl - distr) >= SONAR_DIFF_MAX){
		distl = s_dist(0,SOR_SMOOTH);
		distr = s_dist(1,SOR_SMOOTH);
		if(distl > distr)
			setlogic(1,0,1,0);
		else
			setlogic(0,1,1,0);
		writespeed(SOR_CORRECT_SPD,0);
		delay(SOR_STEP_TIME);
		brake(SOR_BRAKE);
	}
	brake(BRAKE_STABLE);
}

void procedure_fold(int start, int num, int len){
	for(int i=start;i<len-num;++i){
		procedure[i][0] = procedure[i+num][0];
		procedure[i][1] = procedure[i+num][1];
	}
	for(int i=len-num;i<len;++i){
		procedure[i][0] = 0;
		procedure[i][1] = 0;
	}
}