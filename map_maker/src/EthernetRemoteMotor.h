#pragma once
#ifndef __ETHERNET_REMOTE_COMMUN__
#define __ETHERNET_REMOTE_COMMUN__

#include <stdio.h>
#include <sys/types.h>
#include <iostream>
#include <vector>
#include <string> 
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "txtWR.h"
#include "ConvenientTool.h"

#ifdef _WIN32
#include <ws2tcpip.h>
#include <time.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
typedef int socklen_t;
#else
#define closesocket(A) close(A) 
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "txtWR.h"
#define Sleep(A) usleep(1000 *A) 
#endif

#define ETH_REMOTE_SERVER_PORT 8999
#define ETH_REMOTE_BUFF_LEN 24
#define RECV_CMD_LIST_MAX 128

//#ifndef MOTOR_MAX_NUM
#define MOTOR_MAX_NUM 20
#define MOTOR_MAX_NUM_2 40
//#endif


#define LOCAL_SET_ACC 0
#define LOCAL_SET_SPD 1




struct RECV_CMD_LIST_TYPE{
	int ID;
	int CMD;
	int Value;
};

struct ETHER_MOTOR_STATE {
	int ID = -1;
	int SPD_READ = -1;
	int ACC_READ = -1;
	int POS = 0;
	int READY = -1;
	int MOVING = -1;
	int ERR = -1;
	int STAMP = 0;
};
struct ETHER_DATE_RECV {
	int ID = -1;
	int STAMP = 0;
	int DATE[5] = {-1};
};
//struct ETHER_SENSOR_RECV {
//	int stamp = 0;
//	int DATE[4] = { -1 };
//};
struct MOTOR_LOCAL_SETTING {
	int Type = 0;
	int SPD_SETTING = 0;
	int ACC_SETTING = 0;
};


enum EtherMotorCmd {
	LASER_SWITCH = 1,
	LASER_UP,
	LASER_DOWN,
	LASER_LEFT,
	LASER_RIGHT,
	LASER_STOP,
	MOVE_FORWARD,
	MOVE_BACK,
	SPIN_LEFT,
	SPIN_RIGHT,
	MOVE_STOP,
	ERR_RESET,
	SPD_SET,
	FORCE_STOP,
	FORCE_STOP_CLEAR,
	OP_END,
	SET_STAMP,
	SET_STAMP_MACRO,
	ASK_STATE,
	ASK_SENSOR_STATE,
	ASK_MACRO_RES,
	MACRO_PGM,
	CMD_CUT,
	CMD_END,
};

const std::string EtherMotorCmdString[25]{
	"non",
	"LASER_SWITCH",
	"LASER_UP",
	"LASER_DOWN",
	"LASER_LEFT",
	"LASER_RIGHT",
	"LASER_STOP",
	"MOVE_FORWARD",
	"MOVE_BACK",
	"SPIN_LEFT",
	"SPIN_RIGHT",
	"MOVE_STOP",
	"ERR_RESET",
	"SPD_SET",
	"FORCE_STOP",
	"FORCE_STOP_CLEAR",
	"OP_END",
	"SET_STAMP",
	"SET_STAMP_MACRO",
	"ASK_STATE",
	"ASK_SENSOR_STATE",
	"ASK_MACRO_RES",
	"MACRO_PGM",
	"CMD_CUT",
	"CMD_END",
};


class Remote_Motor_Server {
public:
	Remote_Motor_Server() {
#ifdef _WIN32
		WSAData wsaData;
		WORD  DLLVSERION = 0x102; // MAKEWORD(2, 1);0x102
		WSAStartup_res = WSAStartup(DLLVSERION, &wsaData);
#endif
	}
	~Remote_Motor_Server() {

	}
//parameter
public:

	MOTOR_LOCAL_SETTING MotorLocalSetting[MOTOR_MAX_NUM];
	bool Ether_Work = true;
private:
	RECV_CMD_LIST_TYPE RecvCmdList[RECV_CMD_LIST_MAX];
	int SentCMD_Stamp[MOTOR_MAX_NUM];
	int SentMacro_Stamp[MOTOR_MAX_NUM];
	ETHER_MOTOR_STATE MotorDate[MOTOR_MAX_NUM];
	ETHER_DATE_RECV MacroDate[MOTOR_MAX_NUM];
	ETHER_DATE_RECV SensorDate;
	char buf[ETH_REMOTE_BUFF_LEN] = { 0 };
	char buf_Replay[ETH_REMOTE_BUFF_LEN] = { 0 };
	int sListen, ret = 0;
	int sockfd = 0;
	int forClientSockfd = 0;
	struct sockaddr_in serverInfo;
	socklen_t addrlen = sizeof(serverInfo);
	int AutoKillCount = 0;
	int AutoKillCountMax = 499;
	int recv_function_Fail = 0;
	fd_set rfd;
	int selectReturn;
	timeval timeout = { 0,300 };
	bool EthGetData = false;
	int WSAStartup_res;
	bool ClientAskReturn = false;
	bool MacroDateAskReturn = false;
	bool SensorAskReturn = false;
	int ClientAskReturn_Dir;
	int MacroDateAsReturn_Dir;
	int CmdListDir_Save = 0;
	int CmdListDir_Now = 0;
	ConvenientTool Tool;
//function
public:


	void Socket_init() {
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			printf("create socket fail!\n");
			system("pause");
			return;
		}
		memset(&serverInfo, 0, sizeof(serverInfo));

		serverInfo.sin_family = AF_INET;
		serverInfo.sin_port = htons(ETH_REMOTE_SERVER_PORT);

		ret = ::bind(sockfd, (struct sockaddr*)&serverInfo, sizeof(serverInfo));
		int option = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&option, sizeof(option));

		if (ret < 0)
		{
			perror("bind");
			printf("socket bind fail!\n");
			return;
		}
		if (listen(sockfd, 5) == -1) {
			perror("listen");
			printf("socket listen fail!\n");
			return;

		}
		printf("recieve start! \n");
	}

	void UpdateMotorDate(ETHER_MOTOR_STATE Input) {
		int dir = Input.ID;
		Input.STAMP = SentCMD_Stamp[dir];
		MotorDate[dir] = Input;
	}
	void UpdateMotorMotionStamp(int ID,int Input) {
		SentCMD_Stamp[ID] = Input;
	}
	void UpdateMacroDateStamp(int ID, int Input) {
		//printf("SentMacro_Stamp[%d] %d <= Input %d\n", ID, SentMacro_Stamp[ID], Input);
		SentMacro_Stamp[ID] = Input;
	}
	void UpdateSensor(int *date_tmp) {
		SensorDate.ID = 0;
		SensorDate.DATE[0] = date_tmp[0];
		SensorDate.DATE[1] = date_tmp[1];
		SensorDate.DATE[2] = date_tmp[2];
		SensorDate.DATE[3] = date_tmp[3];
		SensorDate.DATE[4] = date_tmp[4];
		//SensorDate.STAMP++;
	}
	void UpdateMacroDate(int id, int* date_tmp) {
		MacroDate[id].ID = id;
		MacroDate[id].DATE[0] = date_tmp[0];
		MacroDate[id].DATE[1] = date_tmp[1];
		MacroDate[id].DATE[2] = date_tmp[2];
		MacroDate[id].DATE[3] = date_tmp[3];
		MacroDate[id].DATE[4] = date_tmp[4];
		MacroDate[id].STAMP = SentMacro_Stamp[id];
		//MacroDate[id].STAMP++;
	}
	void UpdateMacroDate(int id) {
		MacroDate[id].ID = id;
		MacroDate[id].STAMP = SentMacro_Stamp[id];
	}
	int ether_main(void)
	{
		while (Ether_Work) {
			forClientSockfd = accept(sockfd, (struct sockaddr*)&serverInfo, &addrlen);
			printf("Server %d accept %d \n", sockfd, forClientSockfd);
			if (forClientSockfd != -1){
				printf("forClientSockfd %d, sockfd %d \n", forClientSockfd, sockfd);
			}
			else {

			}
			handle_udp_msg(forClientSockfd);
			printf("handle_udp_msg End %d, sockfd %d \n", forClientSockfd, sockfd);
			closesocket(forClientSockfd);
			printf("closesocket End %d, sockfd %d \n", forClientSockfd, sockfd);

			closesocket(sockfd);
			Socket_init();
			Sleep(1000);
		}

		closesocket(sockfd);
		return 1;
	}
	ETHER_MOTOR_STATE MotorDateReturn(int ID) {
		return MotorDate[ID];
	}
	bool Recv_New_CMD(void) {
		if (CmdListDir_Now != CmdListDir_Save) {
			//printf("Recv_New_CMD %d,%d \n", CmdListDir_Now,CmdListDir_Save);
			return true;
		}
		else {
			return false;
		}	
	}
	RECV_CMD_LIST_TYPE Recv_Cmd_RETURN() {
		int Dir = CmdListDir_Now;
		CmdListDir_Now = (CmdListDir_Now + 1) % RECV_CMD_LIST_MAX;
		return RecvCmdList[Dir];
	}

	int ReturnLocalSetting(int ID ,int type) {
		if (type == LOCAL_SET_ACC) {
			return MotorLocalSetting[ID].ACC_SETTING;
		}
		else { //type == LOCAL_SET_SPD
			return MotorLocalSetting[ID].SPD_SETTING;
		}		
	}
	void LocalParaSet(int ID,int type, int Value) {
		if (type == LOCAL_SET_ACC) {
			MotorLocalSetting[ID].ACC_SETTING = Value;
		}
		else { //type == LOCAL_SET_SPD
			MotorLocalSetting[ID].SPD_SETTING = Value;
		}
	}

private:
	int Recv_Cmd_List_RETURN() {

		if (CmdListDir_Now != CmdListDir_Save) {
			return CmdListDir_Now;
		}
		else {
			return -1;
		}
	}

	void handle_udp_msg(int fd)
	{
		socklen_t len;
		int count;
		int coooo = 0;

		printf( "recieve start! \n");
		while (1) {

			coooo++;
			//if (EthGetData) {
			//	printf("==========[ %d ]=========== \n", coooo);
			//}

			memset(buf, 0, ETH_REMOTE_BUFF_LEN);
			
			EthGetData = false;

			int recv_function_res = recv_function(buf);
			if (recv_function_res == (-1)) {
				printf("recv_function Get %d, \n", recv_function_res);
				break;
			}
			int send_function_res = sent_function(buf_Replay);

		}
	}
	int sent_function(char* mat) {
		//memset(buf_Replay, 0, ETH_REMOTE_BUFF_LEN);
		if (ClientAskReturn) {
			ClientAskReturn = false;
			ETHER_MOTOR_STATE tmpS = MotorDate[ClientAskReturn_Dir];

			mat[0] = tmpS.ID & 0x0ff;
			mat[1] = tmpS.READY & 0x0ff;
			mat[2] = tmpS.MOVING & 0x0ff;
			mat[3] = tmpS.ERR & 0x0ff;

			Tool.cutIntToChar(mat + 4, tmpS.POS, 4);
			//mat[7] = tmpS.POS & 0x0ff;
			//tmpS.POS = tmpS.POS >> 8;
			Tool.cutIntToChar(mat + 8, tmpS.SPD_READ, 4);
			Tool.cutIntToChar(mat + 12, tmpS.STAMP, 4);

		}
		else if (SensorAskReturn) {
			SensorAskReturn = false;
			
			Tool.cutIntToChar(mat, SensorDate.ID +  MOTOR_MAX_NUM_2, 1);
			Tool.cutIntToChar(mat + 1, SensorDate.STAMP, 3);
			Tool.cutIntToChar(mat + 4, SensorDate.DATE[0], 4);
			Tool.cutIntToChar(mat + 8, SensorDate.DATE[1], 4);
			Tool.cutIntToChar(mat + 12, SensorDate.DATE[2], 4);
			Tool.cutIntToChar(mat + 16, SensorDate.DATE[3], 4);
			Tool.cutIntToChar(mat + 20, SensorDate.DATE[4], 4);

		}
		else if (MacroDateAskReturn) {
			MacroDateAskReturn = false;
			int tmpid = MacroDateAsReturn_Dir;
			Tool.cutIntToChar(mat, MacroDate[tmpid].ID + MOTOR_MAX_NUM, 1);
			Tool.cutIntToChar(mat + 1, MacroDate[tmpid].STAMP, 3);
			Tool.cutIntToChar(mat + 4, MacroDate[tmpid].DATE[0], 4);
			Tool.cutIntToChar(mat + 8, MacroDate[tmpid].DATE[1], 4);
			Tool.cutIntToChar(mat + 12, MacroDate[tmpid].DATE[2], 4);
			Tool.cutIntToChar(mat + 16, MacroDate[tmpid].DATE[3], 4);
			Tool.cutIntToChar(mat + 20, MacroDate[tmpid].DATE[4], 4);
		}
		int send_count = send(forClientSockfd, mat, sizeof(buf_Replay), 0);
		return send_count;
	}
	int recv_function(char* mat) {
		int recv_count = recv(forClientSockfd, mat, sizeof(buf), 0);
		if (recv_count <= 0) {
			printf("recieve data fail! count = %d\n", recv_count);
			recv_function_Fail = 1;
			return -1;
		}
		else {
			//printf("recieve recvfrom! count = %d \n", recv_count);
			EthGetData = true;
			AutoKillCount = 0;
			recv_function_Fail = 0;
			int tmp_cmdValue = 0;

			RECV_CMD_LIST_TYPE tmpList;
		//Tool.ShowMat(mat, ETH_REMOTE_BUFF_LEN);

			for (int i = 0; i < ETH_REMOTE_BUFF_LEN; i+=6) {
				if (mat[i] != 0) {

					tmpList.ID = mat[i] & 0x0ff;
					tmpList.CMD = mat[i+1] & 0x0ff;
				
					if (tmpList.CMD == EtherMotorCmd::ASK_STATE) {
						ClientAskReturn = true;
						ClientAskReturn_Dir = tmpList.ID;
					}
					else if (tmpList.CMD == EtherMotorCmd::ASK_SENSOR_STATE) {
						SensorAskReturn = true;
					}
					else if (tmpList.CMD == EtherMotorCmd::ASK_MACRO_RES) {
						MacroDateAskReturn = true;
						MacroDateAsReturn_Dir = tmpList.ID - MOTOR_MAX_NUM;
					}
					else {
						if(tmpList.ID >= MOTOR_MAX_NUM) {
							tmpList.ID = tmpList.ID - MOTOR_MAX_NUM;
						}
						tmp_cmdValue = Tool.CombineCharToInt(mat + i + 2, 4);
						//printf("tmp_cmdValue %d [%d,%d,%d]" , tmp_cmdValue , mat[i+2], mat[i+3], mat[i+4]);
						tmpList.Value = tmp_cmdValue;
						RecvCmdList[CmdListDir_Save] = tmpList;
						CmdListDir_Save = (CmdListDir_Save + 1) % RECV_CMD_LIST_MAX;
					}
				}
				else {
					break;
				}
			}


		}
		return recv_count;
	}


};

class Remote_Motor_Client {
public:
	Remote_Motor_Client() {
#ifdef _WIN32
		WSAData wsaData;
		WORD  DLLVSERION = 0x102; // MAKEWORD(2, 1);0x102
		WSAStartup_res = WSAStartup(DLLVSERION, &wsaData);
#endif
	}
	~Remote_Motor_Client() {

	}
	//parameter
public:

	MOTOR_LOCAL_SETTING MotorLocalSetting[MOTOR_MAX_NUM];
	bool Ether_Work = true;
	ETHER_DATE_RECV MacroDate[MOTOR_MAX_NUM];
	ETHER_DATE_RECV SensorDate;
	
private:
	ETHER_MOTOR_STATE MotorDate[MOTOR_MAX_NUM];
	int MacroStamp[MOTOR_MAX_NUM] = { 0 };
	int SensorStamp[1] = { 0 };
	int SentCMD_Stamp[MOTOR_MAX_NUM] = { 0 };
	int SentMacro_Stamp[MOTOR_MAX_NUM] = { 0 };
	char buf[ETH_REMOTE_BUFF_LEN] = { 0 };
	char buf_Replay[ETH_REMOTE_BUFF_LEN] = { 0 };
	int sListen, ret;
	int sockfd = 0;
	int forClientSockfd = 0;
	int AutoKillCount = 0;
	int AutoKillCountMax = 149;
	int recv_function_Fail = 0;
	fd_set rfd;
	int selectReturn;
	timeval timeout = { 0,300 };
	bool EthGetData = false;
	std::string SERVER_IP;
	int SERVER_PORT;
	int WSAStartup_res;
	bool Client_WantDateSend = false;
	ConvenientTool Tool;
	int buf_Write_dir_Now = 0;

	
	//function
public:

private:
public:
	void Socket_init(std::string IP, int PORT) {
		SERVER_IP = IP;
		SERVER_PORT = PORT;
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			printf("create socket fail!\n");
			system("pause");
			return;
		}

		struct sockaddr_in clientInfo;
		memset(&clientInfo, 0, sizeof(clientInfo));
		clientInfo.sin_family = AF_INET;
		clientInfo.sin_port = htons(SERVER_PORT);
		//clientInfo.sin_addr.s_addr = inet_addr(SERVER_IP.c_str());

		int err = connect(sockfd, (struct sockaddr*)&clientInfo, sizeof(clientInfo));
		if (err == -1) {
			printf("Connection error");
		}
		printf("Connect start! \n");
	}
	bool LastMotionDone(int ID) {
		if (SentCMD_Stamp[ID] == MotorDate[ID].STAMP) {
			return true;
		}
		return false;
	}
	bool LastMacroDone(int ID) {
		if (SentMacro_Stamp[ID] == MacroDate[ID].STAMP) {
			//printf("\rLastMacroDone [%3d:%3d] checked\n", SentMacro_Stamp[ID], MacroDate[ID].STAMP);
			return true;
		}
		//printf("\rLastMacroDone [%3d:%3d] non",SentMacro_Stamp[ID] , MacroDate[ID].STAMP);
		return false;
	}
	
	int ether_send(void) {
		int GetSendCount = sent_function(buf);
		int GetRecvCount = recv_function(buf_Replay);
		return GetRecvCount;
	}

	void Send_MOV_CMD(int ID, int Type, int SPD, int ACC, int Value) {
		Send_Program_CMD(ID, Type, SPD, ACC, Value);	
	}
	void Send_MOV_CMD(int ID, int Type, int Value) {
		MOTOR_LOCAL_SETTING* tmpMLS = MotorLocalSetting + ID;
		Send_Program_CMD(ID, Type, tmpMLS->SPD_SETTING, tmpMLS->ACC_SETTING, Value);
	}
	void Send_MOTION_CMD(int ID, int Type) {
		Send_Program_CMD(ID, Type, 0, 0, 1);
	}
	void Send_SETTING_CMD(int ID, int Type,int Value) {
		Send_Program_CMD(ID, Type, Value, Value, 1);
	}
	void Send_ASK_STATE_CMD(int ID) {
		Send_Program_CMD(ID, EtherMotorCmd::ASK_STATE,0,0,1);
	}
	void Send_ASK_SENSOR_CMD() {
		Send_Program_CMD(MOTOR_MAX_NUM_2, EtherMotorCmd::ASK_SENSOR_STATE, 0, 0, 1);
	}
	void Send_ASK_MACRO_RES_CMD(int ID) {
		Send_Program_CMD(ID + MOTOR_MAX_NUM, EtherMotorCmd::ASK_MACRO_RES, 0, 0, 1);
	}
	void Send_MACRO_CMD(int ID, int macro_type) {
		Send_Program_CMD(ID + MOTOR_MAX_NUM, EtherMotorCmd::MACRO_PGM, 0, 0, macro_type);
	}

	ETHER_MOTOR_STATE MotorDateReturn(int ID) {
		return MotorDate[ID];
	}

	int ether_main(void)
	{
		while (Ether_Work) {
			if (Client_WantDateSend) {
				handle_udp_msg(sockfd);
				printf("handle_udp_msg End %d, sockfd %d \n", forClientSockfd, sockfd);
				closesocket(sockfd);
				Socket_init(SERVER_IP, SERVER_PORT);
			}
			Sleep(1000);
		}
		closesocket(sockfd);
		return 1;
	}
private:
	int SetPara(int ID, int Cmd, int Value) {

		
		if (buf_Write_dir_Now == 0) {
			memset(buf, 0, ETH_REMOTE_BUFF_LEN);
		}
		int tmp = buf_Write_dir_Now;
		buf_Write_dir_Now += 6;
		if (buf_Write_dir_Now <= ETH_REMOTE_BUFF_LEN) {
			buf[tmp] = ID & 0x0ff;
			buf[tmp + 1] = Cmd & 0x0ff;

			Tool.cutIntToChar(buf + tmp + 2, Value, 4);
		}
		
		//Tool.ShowMat(buf, ETH_REMOTE_BUFF_LEN);
		return buf_Write_dir_Now;
	}
	void SetPara_Stamp(int ID, int type = EtherMotorCmd::SET_STAMP) {
		if (type == EtherMotorCmd::SET_STAMP) {
			SentCMD_Stamp[ID] = (1 + SentCMD_Stamp[ID]) % 0x0fff;
			SetPara(ID, type, SentCMD_Stamp[ID]);
		}
		else {
			int ID_Dir = ID - MOTOR_MAX_NUM;
			//printf("SentMacro_Stamp[%d] = %d\n", ID_Dir, SentMacro_Stamp[ID_Dir]);
			SentMacro_Stamp[ID_Dir] = (1 + SentMacro_Stamp[ID_Dir]) % 0x0fff;
			//printf("SentMacro_Stamp[%d] = %d after\n", ID_Dir, SentMacro_Stamp[ID_Dir]);
			SetPara(ID, type, SentMacro_Stamp[ID_Dir]);
		}
	}
	void Send_Program_CMD(int ID, int Type, int SPD, int ACC, int Value) {
		MOTOR_LOCAL_SETTING* tmpMLS = MotorLocalSetting + ID;
		switch (Type) {
		case EtherMotorCmd::LASER_SWITCH:
		case EtherMotorCmd::LASER_UP:
		case EtherMotorCmd::LASER_DOWN:
		case EtherMotorCmd::LASER_LEFT:
		case EtherMotorCmd::LASER_RIGHT:
		case EtherMotorCmd::LASER_STOP:
		case EtherMotorCmd::MOVE_FORWARD:
		case EtherMotorCmd::MOVE_BACK:
		case EtherMotorCmd::SPIN_LEFT:
		case EtherMotorCmd::SPIN_RIGHT:
		case EtherMotorCmd::MOVE_STOP:
		case EtherMotorCmd::ERR_RESET:
		case EtherMotorCmd::ASK_STATE:
		case EtherMotorCmd::ASK_MACRO_RES:
		case EtherMotorCmd::FORCE_STOP:
		case EtherMotorCmd::FORCE_STOP_CLEAR:
		case EtherMotorCmd::OP_END:
			SetPara(ID, Type, 1);
			break;
		case EtherMotorCmd::SPD_SET:
			SetPara(ID, Type, Value);
			break;
		case EtherMotorCmd::MACRO_PGM:
			SetPara(ID, Type, Value);
			SetPara_Stamp(ID, SET_STAMP_MACRO);
			break;
		}
		ether_send();
	}
	void UpdateMotorDate(ETHER_MOTOR_STATE Input) {
		int dir = Input.ID;
		MotorDate[dir] = Input;
	}
	void handle_udp_msg(int fd)
	{
		socklen_t len;
		int count;
		int coooo = 0;

		printf("handle_udp_msg Connect start! \n");
		memset(buf, 0, ETH_REMOTE_BUFF_LEN);
		while (Ether_Work) {
			int GetSendCount = sent_function(buf);
			int GetRecvCount = recv_function(buf_Replay);
			printf("[%4d] (S = %d [%x])_ (G = %d [%x]) \n", coooo, GetSendCount, buf[0], GetRecvCount, buf_Replay[0]);
			Sleep(1000);

		}
	}
	int sent_function(char* mat) {
		int send_count = send(sockfd, mat, sizeof(buf), 0);
		buf_Write_dir_Now = 0;
		return send_count;
	}
	int recv_function(char* mat) {
		memset(mat, 0, ETH_REMOTE_BUFF_LEN);
		int recv_count = recv(sockfd, mat, sizeof(buf_Replay), 0);
		if (recv_count <= 0) {
			printf("recieve data fail! count = %d\n", recv_count);
			recv_function_Fail = 1;
			return -1;
		}
		else {
			//printf("recieve recvfrom! count = %d \n", recv_count);
			EthGetData = true;
			AutoKillCount = 0;
			recv_function_Fail = 0;
			int tmpID = mat[0] & 0x0ff;

			if ((tmpID > 0 ) && (tmpID < MOTOR_MAX_NUM)) {
				ETHER_MOTOR_STATE tmpS;
				int tmpV = 0;
				tmpS.ID = mat[0] & 0x0ff;
				tmpS.READY = mat[1] & 0x0ff;
				tmpS.MOVING = mat[2] & 0x0ff;
				tmpS.ERR = mat[3] & 0x0ff;

				tmpV = Tool.CombineCharToInt(mat + 4, 4);
				tmpS.POS = tmpV;
				tmpV = Tool.CombineCharToInt(mat + 8, 4);
				tmpS.SPD_READ = tmpV;
				tmpV = Tool.CombineCharToInt(mat + 12, 4);
				tmpS.STAMP = tmpV;

				UpdateMotorDate(tmpS);
			}
			else if ((tmpID >= MOTOR_MAX_NUM) && (tmpID < MOTOR_MAX_NUM_2)) {
				int tmpid = tmpID - MOTOR_MAX_NUM;
				MacroDate[tmpid].ID = tmpid;
				MacroDate[tmpid].STAMP = Tool.CombineCharToInt(mat + 1, 3);
				MacroDate[tmpid].DATE[0] = Tool.CombineCharToInt(mat + 4, 4);
				MacroDate[tmpid].DATE[1] = Tool.CombineCharToInt(mat + 8, 4);
				MacroDate[tmpid].DATE[2] = Tool.CombineCharToInt(mat + 12, 4);
				MacroDate[tmpid].DATE[3] = Tool.CombineCharToInt(mat + 16, 4);
				MacroDate[tmpid].DATE[4] = Tool.CombineCharToInt(mat + 20, 4);
			}
			else if (tmpID ==  MOTOR_MAX_NUM_2) {
				//int tmpid = (mat[0] & 0x0ff) - MOTOR_MAX_NUM;
				SensorDate.ID = 0;
				SensorDate.STAMP = Tool.CombineCharToInt(mat + 1, 3);
				SensorDate.DATE[0] = Tool.CombineCharToInt(mat + 4, 4);
				SensorDate.DATE[1] = Tool.CombineCharToInt(mat + 8, 4);
				SensorDate.DATE[2] = Tool.CombineCharToInt(mat + 12, 4);
				SensorDate.DATE[3] = Tool.CombineCharToInt(mat + 16, 4);
				SensorDate.DATE[4] = Tool.CombineCharToInt(mat + 20, 4);
			}
		}
		return recv_count;
	}
};

#endif