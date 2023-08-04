#pragma once
// ver. 2017/06/12
// ver. 2017/06/20
// ver. 2017/07/76

#ifndef __CAR_ORIENTAL_MOTOR__
#define __CAR_ORIENTAL_MOTOR__

#include <stdio.h>   
#include <stdlib.h>  
#include <unistd.h>    
#include <sys/types.h> 
#include <sys/stat.h>  
#include <fcntl.h>     
#include <termios.h>   
#include <errno.h>     
#include <sys/time.h>

#include "def.hpp"

#define MotorID(N) N
#define JOG_PLUS 0x10
#define JOG_MINUS 0x20
#define MOTOR_MOVE_FWD 0x40
#define MOTOR_MOVE_REV 0x80
#define MOTOR_DELAY_TIME 18000
#define MOTOR_DELAY_TIME_CAST 19000
#define MOTOR_TYPE_AR 'R'
#define MOTOR_TYPE_AZ 'Z'
#define FALSE false
#define TRUE true
#define u_char unsigned char
#define Int32 int

#define MOTOR_STATE_RETURN_READY 5
#define MOTOR_STATE_RETURN_MOVE 13
#define MOTOR_STATE_RETURN_ALM 7

#define MOTOR_MOVE_ABS 1
#define MOTOR_MOVE_INC 0
#define MOTOR_MOVE_INC_AZ 2

#define Orient_Rbuffer_SIZE 64
#define Orient_ReadTryMax 30
namespace RS485_Oriental_Motor {

	class Motor_Control {

	public:
		int Speed_Multiple;
		bool WR_Working;

	private:

		int Status;
		int fd;
		struct termios options;

		struct timeval start, end;
		long mtime, seconds, useconds;

		u_char Motor_Sel[16];
		bool WR_SWITCH;
		int speed_arr[2] = { B9600,B115200 };
		int name_arr[2] = { 9600, 115200 };
	public:
		void Motor_Setting(char Motor_Type, int Motor_ID) {
			Motor_Sel[Motor_ID] = Motor_Type;
		}
		void Motor_Type_Set(char Motor_Type, ...) // Use 0 to end setting
		{
			va_list ap;
			int arg;
			va_start(ap, Motor_Type);
			while ((arg = va_arg(ap, int)) != 0) {
				Motor_Sel[arg] = Motor_Type;
			}
			va_end(ap);
		}

		bool Motor_Init(std::string ComName) {
			return SerialPortInitial(ComName);
		}
		void Motor_Disconnect() {
		//	CloseHandle(serialPort);
		}
	public:
		Motor_Control() {
			Speed_Multiple = 1;
			for (int i = 0; i < 16; i++)
				Motor_Sel[i] = MOTOR_TYPE_AR;
		}

	public:
		~Motor_Control(void) {
			//CloseHandle(serialPort);
		}

	private:
		bool SerialPortInitial(std::string ComName) {
			Speed_Multiple = 1;

			//SerialPortInitial(ComName);
			fd = OpenDev(ComName);
			printf("OpenDev sucessful\n");
			set_speed(fd, 115200);
			printf("set_speed sucessful\n");
			if (set_Parity(fd, 8, 1, 'E') == FALSE) {
				printf("Set Parity Error\n");
				//exit(0);
				return false;
			}
			else {
				printf("set_Parity sucessful\n");
			}
			for (int i = 0; i < 16; i++)
				Motor_Sel[i] = MOTOR_TYPE_AR;

			WR_Working = false;
			return true;
		}
		int OpenDev(std::string ComName) {
			ComName;
			const char* Dev = ComName.c_str();
			int fd = open(Dev, O_RDWR | O_NOCTTY | O_NDELAY);         //| O_NOCTTY | O_NDELAY 
			if (-1 == fd) {
				perror("Can't Open Serial Port");
				return -1;
			}
			else {
				return fd;
			}

		}
		void set_speed(int fd, int speed) {
			int   i;
			int   status;
			//struct termios options;
			tcgetattr(fd, &options);
			for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
				if (speed == name_arr[i]) {
					tcflush(fd, TCIOFLUSH);
					cfsetispeed(&options, speed_arr[i]);
					cfsetospeed(&options, speed_arr[i]);
					status = tcsetattr(fd, TCSANOW, &options);
					if (status != 0)
						perror("tcsetattr fd1");
					return;
				}
				tcflush(fd, TCIOFLUSH);
			}
		}

		int set_Parity(int fd, int databits, int stopbits, int parity) {
			
			if (tcgetattr(fd, &options) != 0) {
				perror("SetupSerial 1");
				return(FALSE);
			}

			//options.c_cflag = 0;
			//options.c_oflag = 0;
			//options.c_iflag = 0;
			//options.c_lflag = 0;
			printf("Init _ C= %x ,L= %x ,O = %x ,I = %x \n", options.c_cflag, options.c_lflag, options.c_oflag, options.c_iflag);

			options.c_cflag &= ~CSIZE;
			switch (databits) /*Ý’uÉ?ˆÊŒ³É*/
			{
			case 7:
				options.c_cflag |= CS7;
				break;
			case 8:
				options.c_cflag |= CS8;
				break;
			default:
				fprintf(stderr, "Unsupported data size\n"); return (FALSE);
			}
			switch (parity)
			{
			case 'n':
			case 'N':
				options.c_cflag &= ~PARENB;   /* Clear parity enable */
				options.c_iflag &= ~INPCK;     /* Enable parity checking */
				break;
			case 'o':
			case 'O':
				options.c_cflag |= (PARODD | PARENB); /* Ý’uˆ×ŠïÁ?*/
				options.c_iflag |= INPCK;             /* Disnable parity checking */
				break;
			case 'e':
			case 'E':
				options.c_cflag |= PARENB;     /* Enable parity */
				options.c_cflag &= ~PARODD;   /* çzŠ·ˆ×‹ôÁ?*/
				options.c_iflag |= INPCK;       /* Disnable parity checking */
				break;
			case 'S':
			case 's':  /*as no parity*/
				options.c_cflag &= ~PARENB;
				options.c_cflag &= ~CSTOPB; break;
			default:
				fprintf(stderr, "Unsupported parity\n");
				return (FALSE);
			}

			/* Ý’u’âŽ~ˆÊ*/
			switch (stopbits)
			{
			case 1:
				options.c_cflag &= ~CSTOPB;
				break;
			case 2:
				options.c_cflag |= CSTOPB;
				break;
			default:
				fprintf(stderr, "Unsupported stop bits\n");
				return (FALSE);
			}

			/* Set input parity option */
			if (parity != 'n')
				options.c_iflag |= INPCK;

			tcflush(fd, TCIFLUSH);
			//options.c_lflag &= 0;
			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //Input
			options.c_lflag &= ~0xf0;  //Input
			printf("c_lflag _ C= %x , %x \n", options.c_lflag, ECHOE);

			options.c_oflag &= ~OPOST;   //Output
			options.c_oflag &= ~(ONLCR | OCRNL);

			options.c_iflag &= ~(IXON | IXOFF | IXANY);
			options.c_iflag &= ~(IGNBRK | INLCR | IGNCR | ICRNL);
			options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

			options.c_cflag |= (CLOCAL | CREAD);
			options.c_cc[VTIME] = 30; // Ý’u’´?3 seconds
			options.c_cc[VMIN] = 30; // Update the options and do it NOW

			printf("SET _ C= %x ,L= %x ,O = %x ,I = %x \n", options.c_cflag, options.c_lflag, options.c_oflag, options.c_iflag);


			if (tcsetattr(fd, TCSANOW, &options) != 0)
			{
				perror("SetupSerial 3");
				return (FALSE);
			}
			return (TRUE);
		}

		void CRC_16_calculate(char* data, int inputNum) {
			int init_reg = 0xFFFF;

			for (int byte_i = 0; byte_i < inputNum; byte_i++) {
				init_reg = init_reg ^ (data[byte_i] & 0xff);
				for (int bit_i = 0; bit_i < 8; bit_i++) {
					if (init_reg & 0x01)
						init_reg = (init_reg >> 1) ^ 0xa001;
					else
						init_reg = init_reg >> 1;
				}
			}
			init_reg &= 0xFFFF;
			data[inputNum] = init_reg & 0xff;
			data[inputNum + 1] = (init_reg >> 8) & 0xff;
		}
		void Serial_Input(char* Wdata, char* Rdata) {
			CRC_16_calculate(Wdata, 6);

			write(fd, Wdata, 8);
			usleep(MOTOR_DELAY_TIME);

			SerialRead(fd, Rdata, 8);
			usleep(MOTOR_DELAY_TIME);

		}
		void Serial_Input_read(char* Wdata, char* Rdata, int RegNum) {
			CRC_16_calculate(Wdata, 6);

			write(fd, Wdata, 8);
			usleep(MOTOR_DELAY_TIME);

			SerialRead(fd, Rdata, 5 + (RegNum * 2));
			usleep(MOTOR_DELAY_TIME);

		}
		void Serial_Input_mul(char* Wdata, char* Rdata, int num) {
			CRC_16_calculate(Wdata, num);
			write(fd, Wdata, num + 2);
			usleep(MOTOR_DELAY_TIME);
			SerialRead(fd, Rdata, 8);
			usleep(MOTOR_DELAY_TIME);
		}
		void Serial_Input_cast(char* Wdata, int num) {
			CRC_16_calculate(Wdata, num);
			write(fd, Wdata, num + 2);
			usleep(MOTOR_DELAY_TIME_CAST);
		}
		bool SerialRead(int fd, char* Rdata, int RegNum) {
			int res = 0;
			int trytime = 0;
			//gettimeofday(&start, NULL);

			do {
				res = read(fd, Rdata, RegNum);
				trytime++;
			} while ((res < 0) && (trytime < 20000));

			if ((res < 0) || (trytime < 20000)) {
				return false;
			}
			else
				return true;
		}


	public:
		void JOG_Input(int speed, int value, char direction, char Motor_ID) {
			if (Motor_Sel[Motor_ID] == 'R') {
				JOG_Input_AR(speed, value, direction, Motor_ID);
			}
			else if (Motor_Sel[Motor_ID] == 'Z') {
				JOG_Input_AZ(speed, value, direction, Motor_ID);
			}
		}
		void JOG_Acc_Setting(int acc, char Motor_ID) {
			if (Motor_Sel[Motor_ID] == 'R') {
				JOG_Acc_Setting_AR(acc, Motor_ID);
			}
			else if (Motor_Sel[Motor_ID] == 'Z') {
				JOG_Acc_Setting_AZ(acc, Motor_ID);
			}
		}
		void JOG_Input_AR(int speed, int value, char direction, char Motor_ID) {
			speed = speed * Speed_Multiple;
			JOG_Speed_Setting_AR(speed, Motor_ID);
			JOG_Step_Setting_AR(value, Motor_ID);
			JOG_Move(direction, Motor_ID);
		}
		void JOG_Speed_Setting_AR(int speed, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0, 0, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x02; Wbuffer[3] = 0x86;
			Wbuffer[7] = speed >> 24 & 0xff;; Wbuffer[8] = speed >> 16 & 0xff;
			Wbuffer[9] = speed >> 8 & 0xff; Wbuffer[10] = speed & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		void JOG_Step_Setting_AR(int value, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0, 0, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x10; Wbuffer[3] = 0x48;
			Wbuffer[7] = value >> 24 & 0xff;; Wbuffer[8] = value >> 16 & 0xff;
			Wbuffer[9] = value >> 8 & 0xff; Wbuffer[10] = value & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		void JOG_Acc_Setting_AR(int acc, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0, 0, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x02; Wbuffer[3] = 0x88;
			Wbuffer[7] = acc >> 24 & 0xff;; Wbuffer[8] = acc >> 16 & 0xff;
			Wbuffer[9] = acc >> 8 & 0xff; Wbuffer[10] = acc & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		/////
		void JOG_Input_AZ(int speed, int value, char direction, char Motor_ID) {
			speed = speed * Speed_Multiple;
			JOG_Speed_Setting_AZ(speed, Motor_ID);
			JOG_Step_Setting_AZ(value, Motor_ID);
			JOG_Move(direction, Motor_ID);
		}
		void JOG_Speed_Setting_AZ(int speed, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x02, 0xa2, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[7] = speed >> 24 & 0xff;; Wbuffer[8] = speed >> 16 & 0xff;
			Wbuffer[9] = speed >> 8 & 0xff; Wbuffer[10] = speed & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		void JOG_Step_Setting_AZ(int value, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x02, 0xa0, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Wbuffer[7] = value >> 24 & 0xff;; Wbuffer[8] = value >> 16 & 0xff;
			Wbuffer[9] = value >> 8 & 0xff; Wbuffer[10] = value & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}

		void JOG_Acc_Setting_AZ(int acc, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x02, 0xa4, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x02; Wbuffer[3] = 0xa4;
			acc = 1000000000 / acc;
			Wbuffer[7] = acc >> 24 & 0xff;; Wbuffer[8] = acc >> 16 & 0xff;
			Wbuffer[9] = acc >> 8 & 0xff; Wbuffer[10] = acc & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		/////
		void RUN_Input(int speed, char direction, char Motor_ID) {
			speed = speed * Speed_Multiple;
			Motor_Work_Para_Setting(0x0480, speed, Motor_ID);

			if (direction == 'F')
				direction = 0x40;
			else if (direction == 'B')
				direction = 0x80;
			int directiontmp = ((direction & 0xff) << 8);

			Motor_BasicCMD_0x007d(directiontmp, Motor_ID); // Use Data 1
			Motor_BasicCMD_0x007d(directiontmp+1, Motor_ID);// change to Use Data 2
		}
		void RUN_Input_Group(int speed, int speed_2, char direction) {
			speed = speed * Speed_Multiple;
			
			Motor_Work_Para_Setting(0x0480, speed, 1);
			if (speed_2 != 0) {
				speed_2 = speed_2 * Speed_Multiple;
				Motor_Work_Para_Setting(0x0480, speed_2, 2);
			}
			else {
				Motor_Work_Para_Setting(0x0480, speed, 2);
			}

			int directiontmp = 0;
			if (direction == 'B')
				directiontmp = 0x00400;
			else if (direction == 'F')
				directiontmp = 0x00200;
			else if (direction == 'L')
				directiontmp = 0x08000;
			else if (direction == 'R')
				directiontmp = 0x04000;

			Motor_BasicCMD_0x007d(directiontmp, 0, true); // Use Data 1
			Motor_BasicCMD_0x007d(directiontmp + 1, 0, true);// change to Use Data 2
		}

		void RUN_Input_Group(int speed, char direction) {
			RUN_Input_Group(speed, 0, direction);
		}

		void RUN_Acc_Setting(int acc, char direction, char Motor_ID) {
			if (Motor_Sel[Motor_ID] == 'Z') {
				acc = 1000000000 / acc;
		}
			if (direction == 'P')
				Motor_Work_Para_Setting(0x0600, acc, Motor_ID);
			else if (direction == 'N')
				Motor_Work_Para_Setting(0x0680, acc, Motor_ID);
		}
		void Motor_Move_Program(int value, int spd,int acc, int decacc, int INC_ABS, char Motor_ID) {
			spd = spd * Speed_Multiple;
			if (Motor_Sel[Motor_ID] == 'Z') {
				acc = 1000000000 / acc;
				decacc = 1000000000 / decacc;
				if (MOTOR_MOVE_INC == INC_ABS) {
					INC_ABS = MOTOR_MOVE_INC_AZ;
				}
			}
			Motor_Work_Para_Setting_One(0x0400, value, Motor_ID,2);
			Motor_Work_Para_Setting_One(0x0480, spd, Motor_ID, 2);
			Motor_Work_Para_Setting_One(0x0500, INC_ABS, Motor_ID, 2);
			Motor_Work_Para_Setting(0x0600, acc, Motor_ID);
			Motor_Work_Para_Setting(0x0680, decacc, Motor_ID);
			//Motor_Work_Para_Setting_One(0x0600, acc, Motor_ID, 2);
			//Motor_Work_Para_Setting_One(0x0680, decacc, Motor_ID, 2);
			Motor_BasicCMD_Click((0x08 | 0x02), 0x00, Motor_ID);
		}
		void Motor_Move_Program_AccSet(int acc, int decacc,char Motor_ID) {
			if (Motor_Sel[Motor_ID] == 'Z') {
				acc = 1000000000 / acc;
				decacc = 1000000000 / decacc;
			}
			Motor_Work_Para_Setting(0x0600, acc, Motor_ID);
			Motor_Work_Para_Setting(0x0680, decacc, Motor_ID);
			//Motor_Work_Para_Setting_One(0x0600, acc, Motor_ID, 2);
			//Motor_Work_Para_Setting_One(0x0680, decacc, Motor_ID, 2);
		}
		void Motor_Move_Program(int value, int spd, int INC_ABS, char Motor_ID) {
			if (Motor_Sel[Motor_ID] == 'Z') {
				if (MOTOR_MOVE_INC == INC_ABS) {
					INC_ABS = MOTOR_MOVE_INC_AZ;
				}
			}
			spd = spd * Speed_Multiple;
			Motor_Work_Para_Setting_One(0x0400, value, Motor_ID, 2);
			Motor_Work_Para_Setting_One(0x0480, spd, Motor_ID, 2);
			Motor_Work_Para_Setting_One(0x0500, INC_ABS, Motor_ID, 2);
			Motor_BasicCMD_Click((0x08 | 0x02), 0x00, Motor_ID);
			}

		void JOG_Move(char direction, char Motor_ID) {
			if (direction == 'P')
				direction = JOG_PLUS;
			else if
				(direction == 'N')
				direction = JOG_MINUS;
			int directiontmp = ((direction & 0xff) << 8);
			Motor_BasicCMD_Click(directiontmp, 0x0000, Motor_ID);
		}

		void Motor_Stop(char Motor_ID) {
			Motor_BasicCMD_Click(0x0020,0x0000, Motor_ID);
		}

		void Motor_Error_Reset(char Motor_ID) {
			Motor_Stop(Motor_ID);

			char Wbuffer[] = { Motor_ID, 0x06, 0x01, 0x81, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x01; Wbuffer[3] = 0x81;
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x01;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}

		void Pos_Reset(char Motor_ID) {
			Motor_Stop(Motor_ID);

			char Wbuffer[] = { Motor_ID, 0x06, 0x01, 0x8b, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}

		bool Motor_Free(char Motor_ID) {
			//Motor_Stop(Motor_ID);
			bool tmp;
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0x7d, 0x00, 0x01 , 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			Serial_Input_read(Wbuffer, Rbuffer, 1);
			if (Rbuffer[4] & 0x40) {
				tmp = false;
				//Wbuffer[5] = 0x00;
			}
			else {
				Wbuffer[5] = 0x40;
			Wbuffer[1] = 0x06;
			Serial_Input(Wbuffer, Rbuffer);
				tmp = true;
			}
			return tmp;

		}
		//////

		Int32 Motor_Pos_Return(char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0xc6, 0x00, 0x02, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Int32 tmp = 0;
			Serial_Input_read(Wbuffer, Rbuffer, 2);
			tmp = ((Rbuffer[3] & 0xff) << 24) + ((Rbuffer[4] & 0xff) << 16) + ((Rbuffer[5] & 0xff) << 8) + (Rbuffer[6] & 0xff);
			return tmp;
			}
		int Motor_State_Return(char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0x7f, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input_read(Wbuffer, Rbuffer, 1);
			int tmp = ((Rbuffer[3] & 0xff) << 8) + ((Rbuffer[4] & 0xff));
			return tmp;
		}
		bool Motor_Err_Detector(char Motor_ID) {
			int tmp = Motor_State_Return(Motor_ID) ;
			return ((tmp & (0x01 << MOTOR_STATE_RETURN_ALM)) !=0);
		}
		bool Motor_Ready_Detector(char Motor_ID) {
			int tmp = Motor_State_Return(Motor_ID);
			return ((tmp & (0x01 << MOTOR_STATE_RETURN_READY)) != 0);
		}
		bool Motor_Move_Detector(char Motor_ID) {
			int tmp = Motor_State_Return(Motor_ID);
			return ((tmp & (0x01 << MOTOR_STATE_RETURN_MOVE)) != 0);
		}
		int MotorStateSameTECO(char Motor_ID) {
			int tmp = Motor_State_Return(Motor_ID);
			if (tmp & (0x01 << MOTOR_STATE_RETURN_READY)) {
				return 3;
			}
			else if (tmp & (0x01 << MOTOR_STATE_RETURN_ALM)) {
				return 8;
			}
			else if (tmp & (0x01 << MOTOR_STATE_RETURN_MOVE)) {
				return 5;
			}
			else {
				return 0;
			}
		
		}

		Int32 Motor_Speed_Return(char Motor_ID) { //RPM
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0xc8, 0x00, 0x02, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Int32 tmp = 0;
			Serial_Input_read(Wbuffer, Rbuffer, 2);
			tmp = ((Rbuffer[3] & 0xff) << 24) + ((Rbuffer[4] & 0xff) << 16) + ((Rbuffer[5] & 0xff) << 8) + (Rbuffer[6] & 0xff);
			return tmp;
		}
		void Motor_Pos_Move(int speed, int value, char Motor_ID) {
			Int32 tmp = Motor_Pos_Return(Motor_ID);
			tmp = value - tmp;
			if (tmp > 0)
				JOG_Input(speed, tmp, JOG_PLUS, Motor_ID);
			else if (tmp < 0)
				JOG_Input(speed, -tmp, JOG_MINUS, Motor_ID);
		}

		void Motor_Breaker_Lock() {

			char Wbuffer[] = { 0x02, 0x06, 0x02, 0x01, 0x00, 0x03, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[1] = 0x06;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;			///address
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x20;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);
		}
		void Motor_Breaker_Free() {

			char Wbuffer[] = { 0x02, 0x06, 0x02, 0x01, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[1] = 0x06;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;				///address
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);
		}

	private:
		void Motor_Work_Para_Setting(int address, int data, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = address >> 8 & 0xff;
			Wbuffer[3] = address & 0xff;

			Wbuffer[7] = data >> 24 & 0xff;
			Wbuffer[8] = data >> 16 & 0xff;
			Wbuffer[9] = data >> 8 & 0xff;
			Wbuffer[10] = data & 0xff;

			Wbuffer[11] = Wbuffer[7];
			Wbuffer[12] = Wbuffer[8];
			Wbuffer[13] = Wbuffer[9];
			Wbuffer[14] = Wbuffer[10];

			Wbuffer[15] = Wbuffer[7];
			Wbuffer[16] = Wbuffer[8];
			Wbuffer[17] = Wbuffer[9];
			Wbuffer[18] = Wbuffer[10];
			Serial_Input_mul(Wbuffer, Rbuffer, 19);
		}
		void Motor_Work_Para_Setting_One(int address, int data, char Motor_ID,int Dir) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			address = address + (Dir * 2);
			Wbuffer[2] = address >> 8 & 0xff;
			Wbuffer[3] = address & 0xff;
			Wbuffer[7] = data >> 24 & 0xff;
			Wbuffer[8] = data >> 16 & 0xff;
			Wbuffer[9] = data >> 8 & 0xff;
			Wbuffer[10] = data & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
		}
		void Motor_BasicCMD_0x007d(int value, char Motor_ID, bool cast = false) {
			char Wbuffer[] = { Motor_ID, 0x06, 0x00, 0x7d, 0x00, 0x00, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[4] = value >> 8 & 0xff;
			Wbuffer[5] = value & 0xff;
			if (cast) {
				Serial_Input_cast(Wbuffer, 6);
			}
			else {
				Serial_Input(Wbuffer, Rbuffer);
			}
			
		}
		void Motor_BasicCMD_Click(int value_1, int value_2, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x06, 0x00, 0x7d, 0x00, 0x00, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[4] = value_1 >> 8 & 0xff;
			Wbuffer[5] = value_1 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[4] = value_2 >> 8 & 0xff;
			Wbuffer[5] = value_2 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);
		}
	public:



		/* //NotWork
		void Motor_group_run_setting(int speed, char Motor_ID) {  
			char Wbuffer[] = { Motor_ID, 0x10, 0x04, 0x80, 0x00, 0x04, 0x08,  ///[0]ID ///address
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			speed = speed * Speed_Multiple;

			//Wbuffer[7] = 0; Wbuffer[8] = 0;
			Wbuffer[9] = speed >> 8 & 0xff; Wbuffer[10] = speed & 0xff;
			//Wbuffer[11] = 0; Wbuffer[12] = 0;
			Wbuffer[13] = speed >> 8 & 0xff; Wbuffer[14] = speed & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 15);
			Wbuffer[0] = Motor_ID + 1;
			Serial_Input_mul(Wbuffer, Rbuffer, 15);


		}
		void Motor_group_run(char Motor_ID, char dir, int speed) {
			Motor_group_run_setting(speed, Motor_ID);

			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0 };
			char Wbuffer[] = { Motor_ID, 0x10,
				0x00, 0x30,
				0x00, 0x02, 0x04,
				0x00, 0x00, 0x00, 0x0a,
				0, 0 };
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
			Wbuffer[0] = Motor_ID + 1;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);

			if (dir == 'F')
				Wbuffer[7] = 0x02;
			if (dir == 'B')
				Wbuffer[7] = 0x04;
			if (dir == 'R')
				Wbuffer[7] = 0x80;
			if (dir == 'L')
				Wbuffer[7] = 0x40;


			Wbuffer[0] = 0x0a;
			Wbuffer[1] = 0x10;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x01;
			Wbuffer[6] = 0x02;
			//Wbuffer[7] = 0x02; 
			Wbuffer[8] = 0x00;
			Serial_Input_cast(Wbuffer, 9);
			Wbuffer[8] = 0x01;
			Serial_Input_cast(Wbuffer, 9);
		}
		void Motor_group_JOG_setting(char Motor_ID, Int32 value, int speed) {
			char Wbuffer[] = { Motor_ID, 0x06, 0x02, 0x87, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			speed = speed * Speed_Multiple;

			Wbuffer[0] = Motor_ID;
			Wbuffer[2] = 0x02; Wbuffer[3] = 0x87;
			Wbuffer[4] = speed >> 8 & 0xff; Wbuffer[5] = speed & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[2] = 0x10; Wbuffer[3] = 0x49;
			Wbuffer[4] = value >> 8 & 0xff; Wbuffer[5] = value & 0xff;
			Serial_Input(Wbuffer, Rbuffer);
		}
		void Motor_group_JOG(char Motor_ID, char dir, Int32 value, int speed) {
			Motor_group_JOG_setting(Motor_ID, value, speed);
			Motor_group_JOG_setting(Motor_ID + 1, value, speed);
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Wbuffer[] = { Motor_ID, 0x10,
				0x00, 0x30,
				0x00, 0x02, 0x04,
				0x00, 0x00, 0x00, 0x0a,
				0, 0 };
			Serial_Input_mul(Wbuffer, Rbuffer, 11);
			Wbuffer[0] = Motor_ID + 1;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);

			if (dir == 'F') {
				Wbuffer[7] = 0x00; Wbuffer[8] = 0x80;
			}
			if (dir == 'B') {
				Wbuffer[7] = 0x01; Wbuffer[8] = 0x00;
			}
			if (dir == 'R') {
				Wbuffer[7] = 0x20; Wbuffer[8] = 0x00;
			}
			if (dir == 'L') {
				Wbuffer[7] = 0x10; Wbuffer[8] = 0x00;
			}

			Wbuffer[0] = 0x0a;
			Wbuffer[1] = 0x10;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x01;
			Wbuffer[6] = 0x02;
			//Wbuffer[7] = 0x02; Wbuffer[8] = 0x00;
			Serial_Input_cast(Wbuffer, 9);
			Wbuffer[7] = 0x00; Wbuffer[8] = 0x00;
			Serial_Input_cast(Wbuffer, 9);
		}
		*/

		void Motor_group_stop(void) {
			//char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 ,0,0,0,0 };
			//char Wbuffer[] = { 0x00, 0x10,
			//	0x00, 0x7d,
			//	0x00, 0x01,
			//	0x02,
			//	0x00, 0x20, 0, 0 };

			//Serial_Input_cast(Wbuffer, 9);
			//Wbuffer[8] = 0x00;
			//Serial_Input_cast(Wbuffer, 9);

			Motor_BasicCMD_0x007d(0x0020, 0, true);
			Motor_BasicCMD_0x007d(0x0000, 0, true);
		}

		void LDR_Move(char Motor_ID, char direction) {
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			char Wbuffer[] = { Motor_ID, 0x06, 0x00, 0x1e, 0x00, 0x00, 0, 0 };

			if (direction == 'U')
				Wbuffer[5] = 0x01;
			else if (direction == 'D')
				Wbuffer[5] = 0x02;

			Wbuffer[4] = 0x20; // C-ON On
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[4] = 0x21; // Start On 
			Serial_Input(Wbuffer, Rbuffer);

			//Sleep(2000);

			//Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;
			//Serial_Input(Wbuffer, Rbuffer);

		}

		void LDR_Move(char Motor_ID, Int32 speed, Int32 value) {
			int address;
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			char Wbuffer[] = { Motor_ID, 0x06,
				0x00, 0x00, //address
				0x00, 0x00, //value
				0, 0 };

			Wbuffer[2] = 0x04;
			if (value > 0) {
				Wbuffer[3] = 0x02;
			}
			else if (value < 0) {
				Wbuffer[3] = 0x04;
			}

			Wbuffer[4] = value >> 24 & 0xff; Wbuffer[5] = value >> 16 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[3]++;
			Wbuffer[4] = value >> 8 & 0xff; Wbuffer[5] = value & 0xff;
			Serial_Input(Wbuffer, Rbuffer);


			Wbuffer[2]++; Wbuffer[3]--;
			Wbuffer[4] = speed >> 24 & 0xff; Wbuffer[5] = speed >> 16 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);
			Wbuffer[3]++;
			Wbuffer[4] = speed >> 8 & 0xff; Wbuffer[5] = speed & 0xff;
			Serial_Input(Wbuffer, Rbuffer);


			Wbuffer[2] = 0x00; Wbuffer[3] = 0x1e;
			if (value > 0)
				Wbuffer[5] = 0x01;
			else if (value < 0)
				Wbuffer[5] = 0x02;

			Wbuffer[4] = 0x20; // C-ON On
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[4] = 0x21; // Start On 
			Serial_Input(Wbuffer, Rbuffer);

			//Sleep(2000);

			Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;
			//Serial_Input(Wbuffer, Rbuffer);
		}
	};
		}


#endif