#pragma once

#ifndef __CAR_TECO_MOTOR__
#define __CAR_TECO_MOTOR__

#include     <stdio.h>      /*標準輸入輸出定義*/
#include     <stdlib.h>     /*標準函數庫定義*/
#include     <unistd.h>     /*Unix 標準函數定義*/
#include     <sys/types.h> 
#include     <sys/stat.h>  
#include     <fcntl.h>      /*檔控制定義*/
#include     <termios.h>    /*PPSIX 終端控制定義*/
#include     <errno.h>      /*錯誤號定義*/
#include	<sys/time.h>
//#include "txtWR.h"

#define MotorID(N) N
#define JOG_PLUS 0x10
#define JOG_MINUS 0x20
#define MOTOR_MOVE_FWD 0x40
#define MOTOR_MOVE_REV 0x80
#define MOTOR_DELAY_TIME 18000
#define MOTOR_DELAY_TIME_CAST 19000

#define FALSE false
#define TRUE true

#define Rbuffer_SIZE 64
#define failCounter_Retry_Limit 5
#define ReadTryMax 8000



#ifdef __TXT_OUTPUT__
TXT_ReadWrite ReadErrorTxt_O("/home/pi", "/Desktop/Log/Log_RS485.cig", "RS485_Log_", ".txt", 25);
#else
#define TXT_FUNC_TAG(TAG,STRING,format,...) printf(format,##__VA_ARGS__)
#define SHOWFNC(format,...) printf(format,##__VA_ARGS__)
#endif



namespace RS485_TECO_Motor {


	class Motor_Control {
		struct MotorInfo
		{
			int EX=0;
			int FLT=0;
			int MST=0;
			int State=0;
			double VX=0;
			double CURQA=0;
			double CURDA=0;
			int POSD = 0;
			int PERR = 0;
		};

	public:
		int Speed_Multiple;
		bool WR_Working;
		int Pos_Pre[2];
		bool Debug_ShowReadError = false;

		MotorInfo Info[2];

		int failCounter = 3;
		int Serial_Input_timeout_Count = 0;
		int Serial_Input_Fail_Count = 0;
		int MotorPosError[2] = {0,0};

		int InputTryTimeMax = 0;
		int InputTryTimeMin = ReadTryMax;
		
	private:
		char resBuff[Rbuffer_SIZE];

		int Status;
		int fd;
		const char ErrorString_C[7] = "COMERR";
		std::string ErrorString_S = "COMERR";
		struct timeval start, end;
		long mtime, seconds, useconds;
		int failCounter_continous = 0;
		bool WR_SWITCH;
		

	private:
		char TxtTag[9] = "TECO_MOR";
		bool TXT_Output_flag = false;
		char TxtOutBuffer[200];
		int ACC_SAVE[2] = { 0,0 };

#ifdef __TXT_OUTPUT__
		TXT_ReadWrite *OutputText;
	public:
		void TXT_Output_Setting(TXT_ReadWrite *TXT_Set) {
			OutputText = TXT_Set;
			TXT_Output_flag = true;
		}

		TXT_ReadWrite *ReadErrorTxt = &ReadErrorTxt_O;
	public:
		void TXT_DEBUG_Setting(TXT_ReadWrite *TXT_Set) {
			ReadErrorTxt = TXT_Set;
			//ReadErrorTxt_O.TXT_Write("TXT_DEBUG_Setting");
			//TXT_Output_flag = true;
		}
#endif

	public:
		Motor_Control(std::string Dev) {
			Speed_Multiple = 1;

			//SerialPortInitial(ComName);
			fd = OpenDev(Dev);
			printf("Serial Port:OpenDev sucessful\n");
			set_speed(fd, B115200);
			printf("Serial Port:set_speed sucessful\n");
			if (set_Parity(fd, 8, 1, 'n') == FALSE) {
				printf("Serial Port:Set Parity Error\n");
				//exit(0);
			}
			else {
				printf("Serial Port:set_Parity sucessful\n");
			}

			WR_Working = false;
#ifdef __TXT_OUTPUT__
			std::string bbb = "TXT_Output_Setting\n";
			ReadErrorTxt_O.TXT_Write(bbb);
#endif
		}

	public:
		~Motor_Control(void) {
			//CloseHandle(serialPort);
		}

	private:
		int OpenDev(std::string Dev) {
			int fd = open(Dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);         //| O_NOCTTY | O_NDELAY 
			if (-1 == fd) {
				perror("Serial Port:Can't Open Serial Port");
				return -1;
			}
			else {
				return fd;
			}

		}
		void set_speed(int fd, int speed) {
			int   i;
			int   status;
			struct termios   Opt;
			tcgetattr(fd, &Opt);

			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed);
			cfsetospeed(&Opt, speed);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if (status != 0)
				perror("tcsetattr fd1");
			return;
			tcflush(fd, TCIOFLUSH);

		}

		int set_Parity(int fd, int databits, int stopbits, int parity) {
			struct termios options;
			if (tcgetattr(fd, &options) != 0) {
				perror("SetupSerial 1");
				return(FALSE);
			}

			options.c_cflag &= ~CSIZE;
			switch (databits) /*設置數據位元數*/
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
				options.c_cflag |= (PARODD | PARENB); /* 設置為奇效驗*/
				options.c_iflag |= INPCK;             /* Disnable parity checking */
				break;
			case 'e':
			case 'E':
				options.c_cflag |= PARENB;     /* Enable parity */
				options.c_cflag &= ~PARODD;   /* 轉換為偶效驗*/
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

			/* 設置停止位*/
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
			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //Input
			options.c_oflag &= ~OPOST;   //Output
			//options.c_oflag |= ~OPOST;
			//options.c_lflag |= ~(ICANON | ECHO | ECHOE | ISIG);

			options.c_iflag |= CREAD;
			options.c_cflag |= (CLOCAL | CREAD);
			options.c_cc[VTIME] = 30; // 設置超時3 seconds
			options.c_cc[VMIN] = 30; // Update the options and do it NOW

			if (tcsetattr(fd, TCSANOW, &options) != 0)
			{
				perror("SetupSerial 3");
				return (FALSE);
			}
			return (TRUE);
		}

		void Serial_Input(char *Wdata, int size, char *Rdata) {
			if (failCounter_continous > failCounter_Retry_Limit) {
				if (failCounter_continous < 999) {
					TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "failCounter_continous reach %d, Serial Port Break;\n", failCounter_continous);
					failCounter_continous = 1000;
				}
				return;
			}

			failCounter = 6;
			bool ReadCorrect = true;
			bool SerialRead_Success = true;
			do {
				write(fd, Wdata, size);
				SerialRead_Success = SerialRead(fd, Rdata);
				ReadCorrect = CheckReadCorrect(Rdata);

				if (ReadCorrect && SerialRead_Success) {
					failCounter = 0;
					failCounter_continous = 0;
				}
				else {
					MotorErrorRecord(Wdata, Rdata, failCounter, ReadCorrect, SerialRead_Success);

					if (!ReadCorrect) {
						Serial_Input_Fail_Count++;
					}
					if (!SerialRead_Success) {
						Serial_Input_timeout_Count++;
					}
					if (Debug_ShowReadError) {
						ShowCharString(Wdata);
						TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "ShowReadError,COMERR(%d)_Timeout(%d)_FailCount(%d) \n",  Serial_Input_Fail_Count, Serial_Input_timeout_Count, failCounter);
					}
					if (failCounter % 2 == 1) {
						usleep(int(1000));
						RGB_READ(((Wdata[2] - '0')) % 2);
						//printf("Serial_Input , failCounter %d , RGB_READ\n", failCounter);
					}
					failCounter--;
					//printf("Serial_Input ,failCounter = %d ,Timeout = %d,Input_Fail = %d\n", failCounter, Serial_Input_timeout_Count, Serial_Input_Fail_Count);
					if (failCounter == 0) {
						TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "Serial_Input ,failCounter = %d ,Timeout = %d,Input_Fail = %d\n", failCounter, Serial_Input_timeout_Count, Serial_Input_Fail_Count);

						ShowCharString(Wdata);
						failCounter_continous++;
					}
				}
			} while (failCounter);
		}

		bool SerialRead(int fd, char *Rdata) {
			int res = 0;
			int trytime = 0;
			char RdataBuff[Rbuffer_SIZE];

			//gettimeofday(&start, NULL);
			char tmp;
			std::string ReadString;
			//memset(RdataBuff, ';' , Rbuffer_SIZE);
			memset(Rdata, 0, Rbuffer_SIZE);
			memset(resBuff, '_', Rbuffer_SIZE);

			int count = 0;
			bool startw = false;
			int hitcount = 0;

			do {
				memset(RdataBuff, ';', Rbuffer_SIZE);
				res = read(fd, &RdataBuff, sizeof(RdataBuff));
				if (res > 0) {
					ReadString = RdataBuff;
					hitcount = ReadString.find('#');
					if (hitcount != -1) {
						startw = true;
					}
					if (startw) {
						hitcount = ReadString.find(0x0a);
						for (int i = 0; i < res; i++) {
							Rdata[count + i] = RdataBuff[i];
						}
						count = count + res;
						if (hitcount != -1) {
					break;
				}
				}
				}
				else {
				trytime++;
				}

				//res = read(fd, &tmp, 1);
				//if ((tmp == 0x0a && count != 0) || (count >= Rbuffer_SIZE) ){
				//	RdataBuff[count] = 0x0a;
				//	resBuff[0] = res + '0';
				//	break;
				//}
				//if (tmp == '#') {
				//	startw = true;
				//}
				//if (startw) {
				//	if (res > 0) {
				//		RdataBuff[count] = tmp;
				//		count++;
				//	}
				//	else {
				//		RdataBuff[count] = 0x0a;
				//		resBuff[0] = res + '0';
				//		break;
				//	}
				//}
				//if (count==0) {
				//	resBuff[0] = res + '0';
				//}
				//else {
				//	resBuff[count] = res + '0';
				//}
				//trytime++;
				
			} while (trytime < ReadTryMax);

			if (trytime < ReadTryMax) {
				if (InputTryTimeMin > trytime) {
					InputTryTimeMin = trytime;

					std::string tmpStringA =
						"InputTryTimeMin(" + std::to_string(InputTryTimeMin) + ")\n";
#ifdef __TXT_OUTPUT__
					ReadErrorTxt_O.TXT_Write(tmpStringA);
					ReadErrorTxt_O.Write_CharPtr_N((char*)"R", Rdata);
#endif
				}
				if (InputTryTimeMax < trytime) {
					InputTryTimeMax = trytime;
					std::string tmpStringA =
						"InputTryTimeMax(" + std::to_string(InputTryTimeMax) + ")\n";
#ifdef __TXT_OUTPUT__
					ReadErrorTxt_O.TXT_Write(tmpStringA);
					ReadErrorTxt_O.Write_CharPtr_N((char*)"R", Rdata);
#endif
				}
				return true;
			}
			else {
				if (failCounter == 0) {
					TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "Fault , trytime = %d, count = %d \n", trytime, count);
				}

				return false;
			}

		}

		void Serial_Cast(char *Wdata, int size) {
			write(fd, Wdata, size);
		};

		bool CheckReadCorrect(char *Rdata) {
			int hitcount = 0;
			bool res = true;

			std::string ReadString = Rdata;
			hitcount = ReadString.find(ErrorString_S);

			if (hitcount != -1) {
				res = false;
			}
			hitcount = ReadString.find("\r\n");
			//if (hitcount == -1 ) {
			//	TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "DEBUG, Rdata without 0d0a, res = %d, Rdata = [%s] \n", res, Rdata);
			//}

			return res;
		}
		int CharToInt(char *Rdata, char *Tag) {
			std::string OriString = Rdata;
			std::string CutString;
			std::string TagS = Tag ;
			int StartPtr = OriString.find(Tag);
			if (StartPtr == -1) {
				return 0;
			}
			else {
				StartPtr = StartPtr + TagS.size();
			}
			int EndPtr = (int)OriString.find(";", StartPtr);
			if (EndPtr == -1) {
				EndPtr = OriString.size();
			}
			CutString.append(OriString, StartPtr, EndPtr - StartPtr);
			int x;
			try {
				x = atoi(CutString.c_str());
			}
			catch (...) {
				TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "CharToInt Error, Rdata = [%s] \n", Rdata);
				x = 0;
			}
			return x;
		}
		double CharToDouble(char *Rdata, char *Tag) {
			std::string OriString = Rdata;
			std::string CutString;
			std::string TagS = Tag;
			int StartPtr = OriString.find(Tag);
			if (StartPtr == -1) {
				return 0;
			}
			else {
				StartPtr = StartPtr + TagS.size();
			}
			int EndPtr = (int)OriString.find(";", StartPtr);
			if (EndPtr == -1) {
				EndPtr = OriString.size();
			}
			CutString.append(OriString, StartPtr, EndPtr - StartPtr);
			return std::stod(CutString, NULL);
		}
		int CharHexToInt(char *Rdata, char *Tag) {
			std::string OriString = Rdata;
			std::string CutString;
			std::string TagS = Tag;
			int StartPtr = OriString.find(Tag);
			if (StartPtr == -1) {
				return 0;
			}
			else {
				StartPtr = StartPtr + TagS.size();
			}
			int EndPtr = (int)OriString.find(";", StartPtr);
			if (EndPtr == -1) {
				EndPtr = OriString.size();
			}
			CutString.append(OriString, StartPtr, EndPtr - StartPtr);
			int x;
			try {
				x = std::stoi(CutString, nullptr, 16);
			}
			catch (...) {
				TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "CharHexToInt Error, Rdata = [%s] \n", Rdata);
				x = 0;
			}
			return x;
		}
		int MotorPosCheck(char Motor_ID,int Value) {

			if ((Motor_ID == 1) || (Motor_ID == 2)) {
				int Pos_Pre_Dir = Motor_ID - 1;
				if ((abs(Pos_Pre[Pos_Pre_Dir] - Value)) < 100000) {
					Pos_Pre[Pos_Pre_Dir] = Value;
					MotorPosError[Pos_Pre_Dir] = 0;
				}
				else {
					MotorPosError[Pos_Pre_Dir]++;
					if (MotorPosError[Pos_Pre_Dir] > 16) {
						MotorPosError[Pos_Pre_Dir] = 16;
						//Pos_1 = tmpValue;
						SHOWFNC("Motor(%d) Pos_Read Error, Pre = %d, Get = %d, ErrorCount = %d \r", Motor_ID, Pos_Pre[Pos_Pre_Dir], Value, MotorPosError[Pos_Pre_Dir]);
						Value = Pos_Pre[Pos_Pre_Dir];
					}
					else {
						if (MotorPosError[Pos_Pre_Dir] % 5 == 1) {
							TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "Motor(%d) Pos_Read Error, Pre = %d, Get = %d, ErrorCount = %d \n", Motor_ID, Pos_Pre[Pos_Pre_Dir], Value, MotorPosError[Pos_Pre_Dir]);
						}
						else {
							SHOWFNC("Motor(%d) Pos_Read Error, Pre = %d, Get = %d, ErrorCount = %d \n", Motor_ID, Pos_Pre[Pos_Pre_Dir], Value, MotorPosError[Pos_Pre_Dir]);
						}
						Value = Pos_Pre[Pos_Pre_Dir];
					}
				}
			}
			return Value;
		}
		void ShowCharString(char *Rdata) {
			std::string OriString = Rdata;
			int StartPtr = OriString.find('\n');
			if (StartPtr != -1) {
				OriString.at(StartPtr) = ' ';
			}
			StartPtr = OriString.find('\r');
			if (StartPtr != -1) {
				OriString.at(StartPtr) = ' ';
			}
			TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "[%s]\n", OriString.c_str());
		}

		int RGB_READ(char Motor_ID) {
			char Wbuffer[] = "@00:RGB\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			write(fd, Wbuffer, sizeof(Wbuffer));
			bool SerialRead_Success = SerialRead(fd, Rbuffer);
			bool ReadCorrect = CheckReadCorrect(Rbuffer);
			//char SHOW_buffer[Rbuffer_SIZE] = { 0 };
			if (!ReadCorrect) {
				//Serial_Input_Fail_Count++;
				ShowCharString(Wbuffer);
				//for (int gg = 0; Wbuffer[gg] != 0x0a && Wbuffer[gg] != 0x0d; gg++) {
				//	SHOW_buffer[gg] = Wbuffer[gg];
				//}
				TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "RGB_READ_Input_Fail_Count_%d \n", Serial_Input_Fail_Count);
			}

			return Rbuffer[8];
		}
		void MotorErrorRecord(char *Wdata , char *Rdata,int FC,bool RC,bool RS) {
			std::string tmpStringA =
				"failCounter(" + std::to_string(FC)
				+ ")_ReadCorrect(" + std::to_string(RC)
				+ ")_Read_Success(" + std::to_string(RS)
				+ ")\n";
#ifdef __TXT_OUTPUT__

			ReadErrorTxt_O.TXT_Write(tmpStringA);
#endif
			Motor_CMD_Record(Wdata, Rdata);
		}
		void Motor_CMD_Record(char *Wdata, char *Rdata) {
#ifdef __TXT_OUTPUT__

			ReadErrorTxt_O.Write_CharPtr((char*)"W", Wdata);
			ReadErrorTxt_O.Write_CharPtr_N((char*)"G", resBuff);
			ReadErrorTxt_O.Write_CharPtr_N((char*)"R", Rdata);
#endif
		}

	public:

		int Motor_Pos_Set(char Motor_ID, int Value) {
			char Wbuffer[] = "@00:EX=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			int tmpValue = Value;
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (Value < 0) {
				Value = -Value;
				Wbuffer[7] = '-';
			}
			for (int i = 0; i < 10; i++) {
				Wbuffer[17 - i] = Value % 10 + '0';
				Value /= 10;
				if (Value == 0)
					break;
			}

			if (Motor_ID == 1) {
				Pos_Pre[0] = tmpValue;
			}
			else if (Motor_ID == 2) {
				Pos_Pre[1] = tmpValue;
			}

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			return 0;
		}

		int Motor_Pos_Read(char Motor_ID) {
			char Wbuffer[] = "@00:EX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			int tmpValue = 0;
			bool startNum = false;

			tmpValue = CharToInt(Rbuffer, (char*) "EX=");

			if (!CheckReadCorrect(Rbuffer)) {
				TXT_FUNC_TAG(TxtTag, TxtOutBuffer, "Motor(%d) CheckRead Error, Value = %d \n", Motor_ID, tmpValue);
			}
			tmpValue = MotorPosCheck(Motor_ID, tmpValue);
			return tmpValue;
		}

		double Motor_Speed_RPM_Read(char Motor_ID) {
			char Wbuffer[] = "@00:VX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			double tmpValue = 0;
			bool startNum = false;
			bool minus = false;
			bool dotAdd = false;
			float dotNum = 1;
			
			tmpValue = CharToDouble(Rbuffer,(char*)"VX=");
			return tmpValue;

		}

		double Motor_Speed_Plus_Read(char Motor_ID) {
			return (Motor_Speed_RPM_Read(Motor_ID) * 10000.0 / 60.0);
		}

		void Motor_ENAFC_Read(char Motor_ID) {
			char Wbuffer[] = "@00:ENAFC\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			ShowCharString(Rbuffer);
			Motor_CMD_Record(Wbuffer, Rbuffer);
		}
		void Motor_CurrentLimit_Read(char Motor_ID) {
			char Wbuffer[] = "@00:CEMS;CEVAL\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			ShowCharString(Rbuffer);
			Motor_CMD_Record(Wbuffer, Rbuffer);
		}
		void Motor_PositionLimit_Read(char Motor_ID) {
			char Wbuffer[] = "@00:PEMS;PEVAL\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			ShowCharString(Rbuffer);
			Motor_CMD_Record(Wbuffer, Rbuffer);
		}
		int Motor_MST_Read(char Motor_ID) {
			char Wbuffer[] = "@00:MST\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			int mst = 0;
			mst = CharHexToInt(Rbuffer, (char*)"MST=");
			return mst;

		}
		int Motor_FLT_Read(char Motor_ID) {
			char Wbuffer[] = "@00:FLT\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			int startNum = 0;
			int flt = 0;
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			flt = CharHexToInt(Rbuffer, (char*)"FLT=");
			return flt;

		}

		int Motor_State_Read(char Motor_ID) {
			char Wbuffer[] = "@00:MST;FLT\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			int startNum = 0;

			int mst = 0, flt = 0;

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			mst = CharHexToInt(Rbuffer, (char*)"MST=");
			flt = CharHexToInt(Rbuffer, (char*)"FLT=");
			return mst + (flt * 100);
		}
		int Motor_Info_Read_3(char Motor_ID) {
			char Wbuffer1[] = "@00:PERR;MST;FLT\r\n";
			char Wbuffer2[] = "@00:VX;EX\r\n";
			//char Wbuffer3[] = "@00:CURQA;CURDA\r\n";
			char Wbuffer3[] = "@00:CURQA\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer1[2] = (Motor_ID % 10) + '0';
			Wbuffer2[2] = (Motor_ID % 10) + '0';
			Wbuffer3[2] = (Motor_ID % 10) + '0';
			int dir = Motor_ID - 1;

			Serial_Input(Wbuffer1, sizeof(Wbuffer1), Rbuffer);
			Info[dir].PERR = CharToInt(Rbuffer, (char*)"PERR=");
			Info[dir].FLT = CharHexToInt(Rbuffer, (char*)"FLT=");
			Info[dir].MST = CharHexToInt(Rbuffer, (char*)"MST=");
			Info[dir].State = Info[dir].MST + (Info[dir].FLT * 100);

			Serial_Input(Wbuffer2, sizeof(Wbuffer2), Rbuffer);
			int GetEX = CharToInt(Rbuffer, (char*)"EX=");
			Info[dir].EX = MotorPosCheck(Motor_ID, GetEX);
			Info[dir].VX = CharToDouble(Rbuffer, (char*)"VX=");

			Serial_Input(Wbuffer3, sizeof(Wbuffer3), Rbuffer);
			Info[dir].CURQA = CharToDouble(Rbuffer, (char*)"CURQA=");
			//Info[dir].CURDA = CharToDouble(Rbuffer, (char*)"CURDA=");
			Info[dir].CURDA = 0;
			//Info[dir].POSD = Motor_POSD_Read(Motor_ID);
			return 0;
		}

		void Motor_Info_Read(char Motor_ID) {
				int dir = Motor_ID - 1;
				Info[dir].EX = Motor_Pos_Read(Motor_ID);
				Info[dir].VX = Motor_Speed_RPM_Read(Motor_ID);
				Info[dir].CURQA = Motor_Ampre_Read(Motor_ID,'Q');
				Info[dir].CURDA = 0;
				//Info[dir].CURDA = Motor_Ampre_Read(Motor_ID, 'D');
				//Info[dir].State = Motor_State_Read(Motor_ID);
				//Info[dir].FLT = Info[dir].State / 100;
				//Info[dir].MST = Info[dir].State % 100;
				Info[dir].FLT = Motor_FLT_Read(Motor_ID);
				Info[dir].MST = Motor_MST_Read(Motor_ID);
				Info[dir].State = Info[dir].MST + (Info[dir].FLT * 100);	
				//Info[dir].POSD = Motor_POSD_Read(Motor_ID);
				Info[dir].PERR = Motor_PERR_Read(Motor_ID);
		}
		void Motor_Info_Read_2(char Motor_ID) {
			int dir = Motor_ID - 1;
			Info[dir].EX = Motor_Pos_Read(Motor_ID);
			Info[dir+1].EX = Motor_Pos_Read(Motor_ID+1);
			Info[dir].VX = Motor_Speed_RPM_Read(Motor_ID);
			Info[dir + 1].VX = Motor_Speed_RPM_Read(Motor_ID + 1);
			Info[dir].CURQA = Motor_Ampre_Read(Motor_ID, 'Q');
			Info[dir + 1].CURQA = Motor_Ampre_Read(Motor_ID + 1, 'Q');
			Info[dir].CURDA = 0;
			Info[dir + 1].CURDA = 0;
			//Info[dir].CURDA = Motor_Ampre_Read(Motor_ID, 'D');
				//Info[dir].State = Motor_State_Read(Motor_ID);
				//Info[dir].FLT = Info[dir].State / 100;
				//Info[dir].MST = Info[dir].State % 100;
				Info[dir].FLT = Motor_FLT_Read(Motor_ID);
			Info[dir + 1].FLT = Motor_FLT_Read(Motor_ID+1);
				Info[dir].MST = Motor_MST_Read(Motor_ID);
			Info[dir + 1].MST = Motor_MST_Read(Motor_ID+1);
				Info[dir].State = Info[dir].MST + (Info[dir].FLT * 100);	
			Info[dir+1].State = Info[dir+1].MST + (Info[dir+1].FLT * 100);
				//Info[dir].POSD = Motor_POSD_Read(Motor_ID);
				Info[dir].PERR = Motor_PERR_Read(Motor_ID);
			Info[dir + 1].PERR = Motor_PERR_Read(Motor_ID+1);
		}
		int Motor_POSD_Read(char Motor_ID) {
			char Wbuffer[] = "@00:POSD\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			int tmpValue = 0;
			tmpValue = CharToInt(Rbuffer, (char*)"POSD=");
			return tmpValue;
		}
		int Motor_PERR_Read(char Motor_ID) {
			char Wbuffer[] = "@00:PERR\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			int tmpValue = 0;
			bool startNum = false;
			for (int i = 0; i < Rbuffer_SIZE; i++) {
				if (Rbuffer[i] == '=') {
					startNum = true;
					i++;
				}
				else if (Rbuffer[i] == ';' || Rbuffer[i] == 0x0a || Rbuffer[i] == 0x0d)
					break;
				if (startNum) {
					if (Rbuffer[i] == '-') {
						i++;
						tmpValue = -(Rbuffer[i] - '0');
					}
					else {
						if (tmpValue >= 0)
							tmpValue = (10 * tmpValue) + (Rbuffer[i] - '0');
						else
							tmpValue = (10 * tmpValue) - (Rbuffer[i] - '0');
					}
				}

			}
			tmpValue = CharToInt(Rbuffer, (char*)"PERR=");
			return tmpValue;
		}

		double Motor_Ampre_Read(char Motor_ID, char type) {
			char Wbuffer[] = "@00:CURQA;\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Wbuffer[7] = type;
			char SearchTag[] = "CURQA=";
			SearchTag[3] = type;

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			double tmpValue = 0;
			tmpValue = CharToDouble(Rbuffer, SearchTag);

			return tmpValue;
		}

		void ServoOn(char Motor_ID) {
			char Wbuffer[] = "@00:SVON\r\n";
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void ServoOFF(char Motor_ID) {
			char Wbuffer[] = "@00:SVOFF\r\n";
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}


		void ACC_Setting(double acc, char Motor_ID, bool forceInput) {
			if (forceInput) {
				int acc_int = acc;
				ACC_SAVE[Motor_ID - 1] = acc_int;
				char Wbuffer[] = "@00:ACC=00000\r\n";
				char Rbuffer[Rbuffer_SIZE] = { 0 };
				for (int i = 0; i < 5; i++) {
					Wbuffer[12 - i] = acc_int % 10 + '0';
					acc_int /= 10;
				}
				Wbuffer[1] = (Motor_ID / 10) + '0';
				Wbuffer[2] = (Motor_ID % 10) + '0';
				Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			}
			else {
				ACC_Setting(acc, Motor_ID);
			}
		}
		void ACC_Setting(double acc, char Motor_ID) {
			int acc_int = acc;
			if (ACC_SAVE[Motor_ID - 1] != acc_int) {
				ACC_SAVE[Motor_ID - 1] = acc_int;
				char Wbuffer[] = "@00:ACC=00000\r\n";
				char Rbuffer[Rbuffer_SIZE] = { 0 };
				for (int i = 0; i < 5; i++) {
					Wbuffer[12 - i] = acc_int % 10 + '0';
					acc_int /= 10;
				}
				Wbuffer[1] = (Motor_ID / 10) + '0';
				Wbuffer[2] = (Motor_ID % 10) + '0';
				Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			}
		}

		void JOG_Setting(double speed, double acc, char Motor_ID) {
			ACC_Setting(acc, Motor_ID);
			HSPD_Setting(speed, Motor_ID);
		}
		void HSPD_Setting(double speed, char Motor_ID) {
			char Wbuffer[] = "@00:HSPD=00000.000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			int speed_int = speed * 1000;
			for (int i = 0; i < 9; i++) {
				if (i == 3) {
					i++;
				}
				Wbuffer[17 - i] = speed_int % 10 + '0';
				speed_int /= 10;
			}
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void JOG_Move_JOGV(double speed , char Motor_ID) {
			char Wbuffer[] = "@00:JOGV=00000.000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			int speed_int = speed * 1000;
			if (speed < 0) {
				Wbuffer[9] = '-';
				speed_int = -speed_int;
			}
			for (int i = 0; i < 8; i++) {
				if (i == 3) {
					i++;
				}
				Wbuffer[17 - i] = speed_int % 10 + '0';
				speed_int /= 10;
			}
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}

		void JOG_Move(char Motor_ID, double acc, double speed) {
			ACC_Setting(acc, Motor_ID);
			JOG_Move_JOGV(speed, Motor_ID);
		}
		void Motor_JOGX(char Motor_ID, char dir) {
			char Wbuffer[] = "@00:JOGXN\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (dir == 'P') {
				Wbuffer[8] = dir;
			}

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}

		void Motor_Step_Move(char Motor_ID, int Value) {
			int now = Motor_Pos_Read(Motor_ID);
			int tag = now + Value;

			char Wbuffer[] = "@00:X=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (tag < 0) {
				Wbuffer[6] = '-';
				tag = -tag;
			}
			for (int i = 0; tag; i++) {
				Wbuffer[16 - i] = tag % 10 + '0';
				tag /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Step_Move_Both(char Motor_ID_1, int Value1, char Motor_ID_2, int Value2) {

			int tag1 = Motor_Pos_Read(Motor_ID_1) + Value1;
			int tag2 = Motor_Pos_Read(Motor_ID_2) + Value2;
			char Wbuffer1[] = "@00:X=00000000000\r\n";
			char Wbuffer2[] = "@00:X=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer1[1] = (Motor_ID_1 / 10) + '0';
			Wbuffer1[2] = (Motor_ID_1 % 10) + '0';
			Wbuffer2[1] = (Motor_ID_2 / 10) + '0';
			Wbuffer2[2] = (Motor_ID_2 % 10) + '0';
			if (tag1 < 0) {
				Wbuffer1[6] = '-';
				tag1 = -tag1;
			}
			for (int i = 0; tag1; i++) {
				Wbuffer1[16 - i] = tag1 % 10 + '0';
				tag1 /= 10;
			}
			if (tag2 < 0) {
				Wbuffer2[6] = '-';
				tag2 = -tag2;
			}
			for (int i = 0; tag2; i++) {
				Wbuffer2[16 - i] = tag2 % 10 + '0';
				tag2 /= 10;
			}
			Serial_Input(Wbuffer1, sizeof(Wbuffer1), Rbuffer);
			Serial_Input(Wbuffer2, sizeof(Wbuffer2), Rbuffer);
		}
		void Motor_Both_Setting(double speed, double acc) {
			char Wbuffer[] = "@00:ACC=00000;HSPD=00000.000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			int speed_int = speed * 1000;
			int acc_int = acc;

			for (int i = 0; i < 5; i++) {
				Wbuffer[12 - i] = acc_int % 10 + '0';
				acc_int /= 10;
			}
			for (int i = 0; i < 9; i++) {
				if (i == 3) {
					i++;
				}
				Wbuffer[27 - i] = speed_int % 10 + '0';
				speed_int /= 10;
			}

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_STOP(char Motor_ID) {
			char Wbuffer[] = "@00:STOPX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Error_Limit() {
			char Wbuffer[] = "@00:CEMS=3000;CEVAL=3000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}
		void Motor_Error_Limit(int CEMS, int CEVAL) {
			Motor_Error_Limit_CEMS(1,CEMS);
			Motor_Error_Limit_CEVAL(1,CEVAL);
			Motor_Error_Limit_CEMS(2,CEMS);
			Motor_Error_Limit_CEVAL(2,CEVAL);
		}
		void Motor_Position_Limit(int PEMS, int PEVAL) {
			Motor_Error_Limit_PEMS(1, PEMS);
			Motor_Error_Limit_PEVAL(1, PEVAL);
			Motor_Error_Limit_PEMS(2, PEMS);
			Motor_Error_Limit_PEVAL(2, PEVAL);
		}
		void Motor_Current_Limit(int CEMS, int CEVAL) {
			Motor_Error_Limit_CEMS(1, CEMS);
			Motor_Error_Limit_CEVAL(1, CEVAL);
			Motor_Error_Limit_CEMS(2, CEMS);
			Motor_Error_Limit_CEVAL(2, CEVAL);
		}
		void Motor_Error_Limit_CEMS(char Motor_ID, int CEMS) {
			char Wbuffer[] = "@00:CEMS=00000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			for (int i = 0; CEMS; i++) {
				Wbuffer[13 - i] = CEMS % 10 + '0';
				CEMS /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Error_Limit_CEVAL(char Motor_ID, int CEVAL) {
			char Wbuffer[] = "@00:CEVAL=00000\r\n";
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			for (int i = 0; CEVAL; i++) {
				Wbuffer[14 - i] = CEVAL % 10 + '0';
				CEVAL /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Error_Limit_PEMS(char Motor_ID, int PEMS) {
			char Wbuffer[] = "@00:PEMS=00000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			for (int i = 0; PEMS; i++) {
				Wbuffer[13 - i] = PEMS % 10 + '0';
				PEMS /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Error_Limit_PEVAL(char Motor_ID, int PEVAL) {
			char Wbuffer[] = "@00:PEVAL=00000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			for (int i = 0; PEVAL; i++) {
				Wbuffer[14 - i] = PEVAL % 10 + '0';
				PEVAL /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void Motor_Error_Limit_ENAFC(char Motor_ID, int ENAFC) {
			char Wbuffer[] = "@00:ENAFC=0\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			//int Input = (Current * 2) + (Position * 1);
			Wbuffer[10] = ENAFC % 10 + '0';
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

		}

		//Group
		void ServoOn_Cast() {
			char Wbuffer[] = "@00:SVON\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}
		void ServoOff_Cast() {
			char Wbuffer[] = "@00:SVOFF\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}

		void Motor_STOP_Group() {
			char Wbuffer[] = "@00:STOPX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };


			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}
		void Motor_JOGX_Group(char dir) {
			char Wbuffer[] = "@00:JOGXN\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			if (dir == 'P') {
				Wbuffer[8] = dir;
			}
			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}
	};
}

#endif