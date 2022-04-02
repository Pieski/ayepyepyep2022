#include <math.h>
#define DEBUG 1
//总状态机 可用于串口通信
#define STOP 0xAA
#define PULSE 0xBB
#define RUN 0xCC

#define WAKE_BYTE '+'
#define STAGE_A_BYTE 0xA0
#define STAGE_B_BYTE 0xB0
#define OVR_BYTE '-'

//串口调试命令头
#define C_FWD 0x0A
#define C_REV 0x0B
#define C_LFT 0x0C
#define C_RGT 0x0D

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
#define REV_LEFTSIDE 40
#define REV_LEFT 46
#define REV_RIGHT 47
#define REV_RIGHTSIDE 41

//超声PIN
#define HCL_TRIG 48
#define HCL_ECHO 49
#define HCR_TRIG 45
#define HCR_ECHO 44

//强制刹车时间
#define BRAKING 1000

//左侧动力补偿
#define SPD_COMP 50

//基准速度
#define SPD_STAND 150

//修正减速
#define CORR_REDUCE 130

//基准倒行速度
#define SPD_STAND_REV 150

//倒行修正减速
#define CORR_REDUCE_REV 130

//转向速度
#define SPD_TURN_POS 190
#define SPD_TURN_NEG 130

//强制转向时间
#define TURNNING 600

//强制回正时间
#define TURNING_RETURN 170

//转向刹车时间
#define TURNING_BRAKE 1000

//步间刹车时间
#define BRAKE_STABLE 1000

//强制倒车时间（静止离开路口）
#define REVERSE 600

//强制倒车时间（初速度非零）
#define REVERSE_KEEP 300

//启动直行时间
#define START_FORWARD 1000

//终止倒车时间
#define END_REVERSE 1000

//刹车距离(cm)
#define STOP_DIST_A 10.00
#define STOP_DIST_B 5.00

//吸合距离(cm)
#define ATTACH_DIST 5.00

//超声超时时间
#define TIMEOUT 10000

//超声允许误差
#define SONAR_DIFF_MAX 3.0

//超声修正转速
#define SOR_CORRECT_SPD 200

//超声单步时间
#define SOR_STEP_TIME 100

//磁航向修正转速
#define MAG_CORRECT_SPD 200

//磁航向修正单步时间(ms)
#define MAG_STEP_TIME 100

//读磁航向超时
#define MAG_ERROR 99999

//读磁航向置信阈值
#define MAG_DIFF_MAX 360.0

//磁校准置信阈值
#define MAG_CALI_DIFF_MIX 5.0

//磁航向修正允许误差
#define MAG_DIFF_MIN 3.0

//读磁航向滤波次数
#define MAG_DETECT_TIME 5

//磁校准滤波次数
#define MAG_CALI_TIME 20

//磁校准读取次数上限
#define MAG_ERROR_TIME 10


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

#define STORAGE_ON_LEFT {FWD,1},{LFT,1},{STP,1},{REV,2},{RGT,1},{MAG,1}

void correct(int);
void forward();
void turn(int);						//阻塞
void brake(int);					//阻塞
void setlogic(int,int,int,int);
void writespeed(int,int);

int course(int);
void course_block(int);				//阻塞
void course_time(int,int);			//阻塞，非精确时间，非常不精确
									
double m_angle();					//获取当前磁角度 还没写
double m_update(int);				//计算转向后的航向
double m_diff(double,double);		//计算两个磁角间的劣角
void m_recali();					//传感器重置

double s_dist(int);					//读超声距离
void s_cali();						//超声对齐

//总状态机
int stat = RUN;

//任务流程。第一索引标记一级步骤，第二索引标记步骤类型及重复次数，下面称一级步骤中重复执行的部分为二级步骤
short procedure[][2] = {{REF,1},{FWD,1},{LFT,1},{FWD,3},STORAGE_ON_LEFT,STORAGE_ON_LEFT,STORAGE_ON_LEFT,{OVR,1}};
short step_rp = 0;
short step = 0;

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

//基准航向
double ref_heading = 0.00;

double cali = 0.00;

//当前正确航向
double yaw_set = 0.00;

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

	delay(3000);

	Serial.begin(115200);
  	Serial.print(0xAA);

  	Serial2.begin(9600);
  	Serial2.println("AT+PRATE=100");

  	while(Serial.read() != WAKE_BYTE);
  	Serial.println("Start initializing");

	setlogic(1,0,1,0);
	digitalWrite(HCL_TRIG, 0);
	digitalWrite(HCR_TRIG, 0);

	for(int i=0;i<3;++i){
	    ir_map[0] = (short)digitalRead(LEFTSIDE);
	    ir_map[1] = (short)digitalRead(LEFT);
	    ir_map[2] = (short)digitalRead(CENTER);
	    ir_map[3] = (short)digitalRead(RIGHT);
	    ir_map[4] = (short)digitalRead(RIGHTSIDE);
  	}
}

double last_read,read;
void loop(){
	if(DEBUG == 1){
		m_recali();
		delay(20000);
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

		//检测当前一级步骤是否已经重复执行完毕
		if(step_rp >= procedure[step][1]){

			//在开始下一步前进行特殊步骤的过渡
			//倒行 -> 转向：继续倒车离开路口，然后重新直行循迹到这个路口，再转向
			if(procedure[step][0] == REV && (procedure[step+1][0] == LFT || procedure[step+1][0] == RGT)){
				course_time(COURSE_REV, 1000);
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
			Serial.println(dist);
			if(stop_flag == 0){
				if(dist <= STOP_DIST_A && dist > 0.10){
					Serial.println("stopped at position A");
					brake(BRAKING);
					s_cali();
					Serial.print(STAGE_A_BYTE);
					while(Serial.read() != WAKE_BYTE);
					stop_flag = 1;
					break;
				}
			}else if(stop_flag == 1){
				if(dist <= STOP_DIST_B && dist > 0.10){
					Serial.println("stopped at position B");
					brake(BRAKING);
					s_cali();
					Serial.print(STAGE_B_BYTE);
					while(Serial.read() != WAKE_BYTE);
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

			case REF:
			brake(BRAKE_STABLE);
			Serial2.println("AT+INIT");
			delay(500);
			cali = 0.0;
		    last_read = 10000.0;
		    read = 0.0;
			for(int i=0;i<MAG_CALI_TIME;++i){
				read = m_angle();
				if(last_read != 10000.0 && abs(read - last_read) > MAG_CALI_DIFF_MIX){
					i = 0;
					cali = 0;
					read = 0.0;
					last_read = 10000.0;
					continue;
				}
				cali += read;
				delay(10);
			}
			yaw_set = cali/MAG_CALI_TIME;
			ref_heading = yaw_set;
			Serial.print("yaw_set = ");
			Serial.print(yaw_set);
			Serial.print("\n");
			step_rp ++;
			break;

			case MAG:
			m_cali();
			step_rp++;
			break;
		}

		for(int i=0;i<5;++i)
			ir_map_old[i] = ir_map[i];
	}

	else if(stat == PULSE){
		Serial.println("pulsed");
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

//若缓冲区有数据，则在loop()间隙触发
void serialEvent(){
	Serial.print("detected = ");
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
}

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

	m_update(stat);

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
		if(ir_map_rev[0] == 0 || ir_map_rev[3] == 0)
			correct(ir_map_rev[3]);
		else if(ir_map_rev[1] == 0 || ir_map_rev[2] == 0)
			correct(ir_map_rev[2]);
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

//读取当前磁角度
double m_angle(){
	int readcount = 0;
	while(Serial2.read()!='w' && readcount <= MAG_ERROR)
		readcount++;

	if(readcount >= MAG_ERROR)
		return 720.0;

	char buf[10];
	Serial2.readBytes(buf,10);

	double r = 0.0;
	int minflag = 1;
	byte potflag = 0;
	int ncount_a = 0, ncount_b = 0;
	int num[6] = {0};

	for(int i=0;i<10;++i){
		if(buf[i] == '.')
			potflag = 1;
		if(buf[i] >= 48 && buf[i] <= 57){
			num[ncount_a+ncount_b] = buf[i] - 48;
			if(potflag)
				ncount_b++;
			else
				ncount_a++;
		}
		if(buf[i] == '-')
			minflag = -1;
	}

	/*for(int i=0;i<6;++i)
		Serial.print(num[i]);

	Serial.print("   ncount_a = ");
	Serial.print(ncount_a);
	Serial.print("   minflag = ");
	Serial.print(minflag);
	Serial.print("\n");*/

	int a = ncount_a + ncount_b - 1;
	for(int i=0;i<ncount_b;++i)
		r += pow(10,-1*ncount_b+i) * num[a-i];
	for(int i=0;i<ncount_a;++i)
		r += pow(10,i) * num[a-ncount_b-i];
	
	Serial2.println("AT+INIT");
	return r * (double)minflag;
}

//计算转向后的设定航向，mode=1：左转
double m_update(int mode){
	if(mode == 1)
		yaw_set += 90.0;
	else if(mode == 0)
		yaw_set -= 90.0;
	if(yaw_set > 180.0)
		yaw_set -= 360.0;
	else if(yaw_set < -180.0)
		yaw_set += 360.0;

	Serial.print("yaw set to");
	Serial.print(yaw_set);
	Serial.print("\n");
	return yaw_set;
}

double m_diff(double a, double b){
	if(abs(a-b) > 180)
		return abs(360 - abs(a-b));
	return abs(a-b);
}

//超声测距，side = 0：左侧
double s_dist(int side){
	if(side == 0){
		digitalWrite(HCL_TRIG,HIGH);
		delayMicroseconds(10);
		digitalWrite(HCL_TRIG,LOW);
		return (double)pulseIn(HCL_ECHO, HIGH, TIMEOUT) / 58.0;
	}else{
		digitalWrite(HCR_TRIG,HIGH);
		delayMicroseconds(10);
		digitalWrite(HCR_TRIG,LOW);
		return (double)pulseIn(HCR_ECHO, HIGH, TIMEOUT) / 58.0;
	}
}

void s_cali(){
	double distl,distr;
	do{
		distl = s_dist(0);
		distr = s_dist(1);
		Serial.print("distl = ");
		Serial.print(distl);
		Serial.print("   distr = ");
		Serial.print(distr);
		Serial.print("\n");
		if(distl > distr)
			setlogic(1,0,1,0);
		else
			setlogic(0,1,1,0);
		writespeed(SOR_CORRECT_SPD,0);
		delay(SOR_STEP_TIME);
	}while((double)abs(distl - distr) >= SONAR_DIFF_MAX);
}

void m_cali(){
	Serial.print("ref_heading = ");
	Serial.print(ref_heading);
	Serial.print("diff = ");
	Serial.print(m_diff(yaw_set, ref_heading));
	Serial.print("\n");

	double current_yaw = 0.0;
	for(int i=0;i<MAG_DETECT_TIME;++i){
		current_yaw += m_angle();
		delay(20);
	}
	current_yaw /= MAG_DETECT_TIME;

	Serial.print("current_yaw = ");
	Serial.print(current_yaw);
	Serial.print("      yaw_set = ");
	Serial.print(yaw_set);
	Serial.print("      diff = ");
	Serial.print(m_diff(current_yaw, yaw_set));
	Serial.print("\n");

	int trytime = 0;
	for(;trytime<3;++trytime){
		if(m_diff(current_yaw, yaw_set) <= MAG_DIFF_MAX)
			break;
		current_yaw = m_angle();
		delay(20);
	}

	if(trytime <= 3){
		if(m_diff(current_yaw, yaw_set) <= MAG_DIFF_MAX){
			while(m_diff(current_yaw, yaw_set) > MAG_DIFF_MIN){
				Serial.print("current_yaw = ");
				Serial.print(current_yaw);
				Serial.print("\n");
				if(current_yaw < yaw_set){
					writespeed(0,MAG_CORRECT_SPD);
					delay(MAG_STEP_TIME);
				}else{
					writespeed(MAG_CORRECT_SPD,0);
					delay(MAG_STEP_TIME);
				}
				brake(TURNING_BRAKE);

				current_yaw = 0.0;
				for(int i=0;i<MAG_DETECT_TIME;++i){
					current_yaw += m_angle();
					delay(20);
				}
				current_yaw /= MAG_DETECT_TIME;
			}
		}
	}
}

void m_recali(){
	Serial2.println("AT+INIT");
	delay(500);
	Serial2.println("AT+PRATE=0");
	delay(500);
	Serial2.println("AT+CALI=1");
	setlogic(1,0,0,1);
	writespeed(SPD_STAND,SPD_STAND);
	delay(20000);
	writespeed(0,0);
	setlogic(0,0,0,0);
	Serial2.println("AT+CALI=0");
}