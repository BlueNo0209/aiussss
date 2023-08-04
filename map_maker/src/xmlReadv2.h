#pragma once
#include <tinyxml.h>
//#include "tinystr.h"
#include <string>

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "def.hpp"
#include "Hiroshima_tool.hpp"
//using namespace std;

class SOWAN_PARAMETERv2 {
public:
	struct Set_Point
	{
		int x;
		int y;
	};

	class Set_Custom_Partol
	{
	public:
		bool pass = false;
		bool SW = true;
		int Room = -1;
		int EndPartolSize = -1;
		int PartolEntry = -1;
		int x = 0;
		int y = 0;
		Set_Custom_Partol() {

		}
		~Set_Custom_Partol() {

		}
		void DataClear() {
			x = 0;
			y = 0;
			Room = -1;
			PartolEntry = -1;
			EndPartolSize = -1;
		}

	};


public:
	std::string UserName;
	std::string HomePath;
	std::string SowanPath;

	std::vector<Set_Point> Point_Route;
	std::vector<Set_Point> Point_Charge;
	std::vector<Set_Point> Point_Room;

	std::vector<Set_Custom_Partol> Custom_Partol;

	std::vector<int> RoomAddress;
	std::vector<int> RoomSequence;
	std::vector<int> RoomReturnPatrol;
	std::vector<std::string> RoomNumber;

	std::string BlockImgPath;
	std::string RS485Port = "/dev/ch341";

	std::vector<VoicePlayInfo> RoomVoice;

	VoicePlayInfo PatrolVoice;
	VoicePlayInfo VehicleStopVoice_Normal;
	VoicePlayInfo VehicleStopVoice_Low;
	VoicePlayInfo DoorNotOpenVoice;
	VoicePlayInfo RoomPatrolVoice;
	VoicePlayInfo GoChargeVoice;
	VoicePlayInfo ReturnOriginVoice_Normal;
	VoicePlayInfo ReturnOriginVoice_Charge;
	VoicePlayInfo ReturnOriginVoice_RoomCall;
	VoicePlayInfo ReturnOriginVoice_RoomPatrol;
	VoicePlayInfo ReturnOriginVoice_Patrol;
	VoicePlayInfo RoomCallEndReturnPatrol;


	double FrontSafeDistance = 0.4;
	double BackSafeDistance = 0.4;
	double FrontSafeDistance_CamTrac = 0.4;
	double BackSafeDistance_CamTrac = 0.4;
	double SideSafeDistance = 0.25;

	double ArrivalRange = 0.1;
	double MaxCircleSpd = 0.33;
	double MaxLineSpd = 0.2;
	double MaxTurnSpd = 0.2;
	double MinCircleNeedDeg = 5;

	double MinTurnNeedDeg = 30;
	double normalMovDegOffset = 3;
	double PatrolOverDeg = 120;
	double PatrolOverDeg_FinDeg = 45;
	double BackMovEscapeDistance = 0.1;

	double DoorOpenTime_high_T1 = 7.5;
	double DoorOpenTime_low_T1 = 1.0;
	double DoorOpenTime_high_T2 = 0.075;
	double DoorOpenTime_low_T2 = 0.075;
	double DoorOpenTime_Wait = 1.0;

	double RoomPatrol_RecWaitTime = 1;
	double Motor_Current_Limit = 10.0;
	double ManualCouveRadius = 1.0;
	double auto_acc_RPM = 1500;
	double obj_spd_RPM = 300;

	double obj_acc_RPM = 800;
	double DoorOpenDistance = 1.5;
	double DoorOpenWidth = 0.4;
	double acc = 1500;
	double spd = 400;

	double turnRate = 0.2;

	int PatrolCycleCount = 1;
	int PatrolCycleWaitTime = 0;
	int WorkLoopTime = -1;
	int WorkLoopSleepTime = 0;
	int ChargeWaitTime = 0;
	int HitObjectRunawayType = 0;
	int normalMoveType = 0;
	int DoorOpenType = 1;
	int DefaultMonitorType = 0;
	int Motor_ENAFC = 0;
	int Motor_CEMS = 3000;
	int Motor_CEVAL = 3000;
	int Motor_PEMS = 100;
	int Motor_PEVAL = 1000;
	int DEG_OFFSET_BEFORE = 0;
	int DEBUG_SelPatrolType = 0;

	bool HitObjectStop = true;
	bool Debug_SW = false;
	bool PatrolVoiceSwitch = false;
	bool ChargePlatform = false;
	bool AutoMotorOn = false;

	bool LowSensorInstall = false;
	bool BatterySensorInstall = false;
	bool PressSWInstall = false;
	bool HitSensorInstall = false;
	bool ChargePlatformSensorInstall = false;

	bool DoorClose_RoomPatrol = true;
	bool Debug_GetGobalPathTest = false;
	bool Debug_ShowReadError = false;
	bool DEBUG_HitObjectStop = true;
	bool DEBUG_LowSensorByIO = true;

private:
#define HOME_ROOT "/home/pi/catkin_ws/src/Hiroshima_sys_test"
	std::string Home_Root_String = HOME_ROOT;
	std::string defaultVoicePath = Home_Root_String + "/Voice/DefaultVoice.wav";
	std::string PathCatkin_Sowan = "/catkin_ws/src/Hiroshima_sys_test";
	VoicePlayInfo VoiceDefault;
	//TiXmlElement* pRootElement = NULL;
	TiXmlElement* pSOWAN = NULL;
	TiXmlElement* pNode = NULL;

public:
	SOWAN_PARAMETERv2(std::string file) {
		getHomePath();
		defaultVoicePath = SowanPath + "/Voice/DefaultVoice.wav";
		VoiceDefault.Path = defaultVoicePath;
		VoiceDefault.Time = 5;

		PatrolVoice = VoiceDefault;
		VehicleStopVoice_Normal = VoiceDefault;
		VehicleStopVoice_Low = VoiceDefault;
		DoorNotOpenVoice = VoiceDefault;
		RoomPatrolVoice = VoiceDefault;
		GoChargeVoice = VoiceDefault;
		ReturnOriginVoice_Normal = VoiceDefault;
		ReturnOriginVoice_Charge = VoiceDefault;
		ReturnOriginVoice_RoomCall = VoiceDefault;
		ReturnOriginVoice_RoomPatrol = VoiceDefault;
		ReturnOriginVoice_Patrol = VoiceDefault;
		RoomCallEndReturnPatrol = VoiceDefault;

		LoadXml(SowanPath + file);
		LoadXml_SysConfig(SowanPath + "/map/SysConfig.xml");
	}
	~SOWAN_PARAMETERv2() {

	}

	void getHomePath()
	{
		uid_t userid;
		struct passwd* pwd;
		userid = getuid();
		pwd = getpwuid(userid);

		UserName = (std::string)pwd->pw_name;
		HomePath = (std::string)pwd->pw_dir;
		SowanPath = HomePath + PathCatkin_Sowan;

		std::cout << "UserName :" << UserName << std::endl;
		std::cout << "HomePath :" << HomePath << std::endl;
		std::cout << "SowanPath :" << SowanPath << std::endl;

	}
	bool TextExist(TiXmlElement* node_tmp) {
		
		if (node_tmp) {
			const char *text = node_tmp->GetText();
			if (text) {
				std::cout << "[Read]:";
				return true;
			}
			else {
				std::cout << "[Def ]:";
				return false;
			}
		}
		else {
			std::cout << "[Def ]:";
			return false;
		}
	}
	std::string GetSowanPath(std::string Input) {
		std::string AfterCut;
		int res = (int)Input.find(PathCatkin_Sowan, 0);
		if (res != -1) {
			int fetpathptr, fetpathptr_2;
			fetpathptr = res + PathCatkin_Sowan.size();
			fetpathptr_2 = Input.size() - fetpathptr;
			AfterCut = AfterCut.append(Input, fetpathptr, fetpathptr_2);
		}
		else {
			AfterCut = Input;
		}
		return SowanPath + AfterCut;
	}

	void GetBracketsPos(int *data, std::string brackets) {
		data[0] = brackets.find('(');
		data[1] = brackets.find(',');
		data[2] = brackets.find(')');
	}
	std::string GetBracketsValue(int *data, std::string brackets, int dir) {
		std::string tmp;
		tmp = tmp.assign(brackets, data[dir] + 1, data[dir + 1] - data[dir] - 1);
		return tmp;
	}
	bool GetIntPara(int *data, TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (TextExist(node_tmp) ) {
			*data = atoi(node_tmp->GetText());
			res = true;
		}
		std::cout << tag << ":(" << *data << ")" << std::endl;
		return res;
	}
	bool GetBoolPara(bool *data, TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (TextExist(node_tmp)) {
			int tmpDebug_SW = atoi(node_tmp->GetText());
			if (tmpDebug_SW == 0) {
				*data = false;
			}
			else {
				*data = true;
			}
			res = true;
		}
		std::cout << tag << ":(" << *data << ")" << std::endl;
		return res;
	}
	bool GetDoublePara(double *data, TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (TextExist(node_tmp)) {
			*data = strtod(node_tmp->GetText(), NULL);
			res = true;
		}
		std::cout << tag << ":(" << *data << ")" << std::endl;
		return res;
	}
	bool GetVoicePara(VoicePlayInfo *data, TiXmlElement* node, std::string tag_time, std::string tag_path) {
		int resget = 0;
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag_path.c_str());
		*data = VoiceDefault;
		std::string str_tmp;

		if (TextExist(node_tmp)) {
			str_tmp = node_tmp->GetText();
			str_tmp = GetSowanPath(str_tmp);
			data->Path = str_tmp;			
			//res = true;
			resget++;
			std::cout << tag_path << ":(" << data->Path << ")" << std::endl;
		}
		else {
			data->Path = VoiceDefault.Path;			
			std::cout << tag_path << ":(" << "NonExist" << ")" << std::endl;
		}

		node_tmp = node->FirstChildElement(tag_time.c_str());
		if (TextExist(node_tmp)) {
			data->Time = atoi(node_tmp->GetText());
			//res = true;
			resget++;
			std::cout << tag_time << ":(" << data->Time << ")" << std::endl;
		}
		else {
			data->Path = VoiceDefault.Time;
			std::cout << tag_path << ":(" << "NonExist" << ")" << std::endl;
		}

		if (resget == 2) {
			res = true;
		}
		else {
			std::cout << tag_time << ":(" << data->Time << ")" << std::endl;
		}
		return res;
	}
	bool GetStringPara(std::string *data, TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		std::string str_tmp;
		if (TextExist(node_tmp)) {
			str_tmp = node_tmp->GetText();
			*data = GetSowanPath(str_tmp);
			res = true;
		}
		std::cout << tag << ":(" << *data << ")" << std::endl;
		return res;
	}
	bool GetString485Para(std::string *data, TiXmlElement* node, std::string tag) {
		return GetTxtPara(data, node, tag);
	}
	bool GetTxtPara(std::string *data, TiXmlElement* node, std::string tag) {
		bool res;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		std::string str_tmp;
		if (TextExist(node_tmp)) {
			str_tmp = node_tmp->GetText();
			*data = str_tmp;
			res = true;
		}
		else {
			res = false;
		}
		std::cout << tag << ":(" << *data << ")" << std::endl;
		return res;
	}
	bool GetPointPara(Set_Point *data, TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (TextExist(node_tmp)) {
			int str_dir[3];
			std::string tmp;
			std::string nodeString = node_tmp->GetText();

			GetBracketsPos(str_dir, nodeString);

			tmp = GetBracketsValue(str_dir, nodeString, 0);
			data->x = atoi(tmp.c_str());
			tmp = GetBracketsValue(str_dir, nodeString, 1);
			data->y = atoi(tmp.c_str());
			res = true;
			std::cout << tag << ":(" << data->x << "," << data->y << ")" << std::endl;
		}
		return res;
	}
	bool GetPointPara(Set_Point *data, TiXmlElement* node, std::string tag,int default_X, int default_Y) {
		bool res;
		if ( GetPointPara(data, node, tag) ) {
			res = true;
		}
		else {
			data->x = default_X;
			data->y = default_Y;
			res = false;
		}
		return res;
	}
	bool Get_Custom_Partol(TiXmlElement* node, std::string tag) {
		bool res = false;
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (node_tmp ) {
			std::string SET_XX, nodeString, nodeString_Cut;
			Set_Custom_Partol CustomTmp;

			int str_dir[3];
			TiXmlElement* pRoute = node_tmp->FirstChildElement("Route");
			if (pRoute) {
				for (int i = 1; i < 1001; i++) {
					SET_XX = "SET_" + std::to_string(i);
					pNode = pRoute->FirstChildElement(SET_XX.c_str());
					if (TextExist(pNode)) {

						nodeString = pNode->GetText();
						GetBracketsPos(str_dir, nodeString);
						CustomTmp.DataClear();

						std::cout << tag << "[" << i << "]:";

						nodeString_Cut = GetBracketsValue(str_dir, nodeString, 0);
						if (nodeString_Cut.find('R') != -1) {
							nodeString_Cut = GetBracketsValue(str_dir, nodeString, 1);
							CustomTmp.Room = atoi(nodeString_Cut.c_str()) - 1;
							Custom_Partol.push_back(CustomTmp);

							std::cout << "(R," << CustomTmp.Room << ")" << std::endl;
						}
						else if (nodeString_Cut.find('E') != -1) {
							nodeString_Cut = GetBracketsValue(str_dir, nodeString, 1);
							CustomTmp.Room = -2;
							CustomTmp.PartolEntry = atoi(nodeString_Cut.c_str()) - 1;
							Custom_Partol.push_back(CustomTmp);
							CustomTmp.EndPartolSize = i - 1;
							std::cout << "(E," << CustomTmp.Room << ")" << std::endl;
							//break;
						}
						else {
							CustomTmp.x = atoi(nodeString_Cut.c_str());
							nodeString_Cut = GetBracketsValue(str_dir, nodeString, 1);
							CustomTmp.y = atoi(nodeString_Cut.c_str());
							Custom_Partol.push_back(CustomTmp);

							std::cout << "(" << CustomTmp.x << "," << CustomTmp.y << ")" << std::endl;
						}
					}
					else {
						break;
					}
				}
			}
			

			GetDoublePara(&RoomPatrol_RecWaitTime, node_tmp, "RoomPatrol_RecWaitTime");
			GetVoicePara(&RoomPatrolVoice, node_tmp, "RoomPatrolVoiceTime", "RoomPatrolVoicePath");

			res = true;
		}
		return res;
	}

	void Get_RoomData(TiXmlElement* node, std::string tag) {
		TiXmlElement* node_tmp = node->FirstChildElement(tag.c_str());
		if (node_tmp) {
			TiXmlElement* pRoom = NULL;
			TiXmlElement* pPosType = NULL;
			std::string ROOM_SET, ROOM_SET_P;
			VoicePlayInfo RoomVoiceTemp;
			Set_Point PointTmp;
			int RoomAddress_tmp;
			int RoomReturnPatrol_tmp;
			std::string RoomNumTmp;

			for (int i = 1; i < 101; i++) {
				ROOM_SET = "ROOM_SET_" + std::to_string(i);
				pRoom = node_tmp->FirstChildElement(ROOM_SET.c_str());
				if (pRoom) {
					ROOM_SET_P = "SET_1";

					pPosType = pRoom->FirstChildElement(ROOM_SET_P.c_str());
					if (pPosType){
						for (int j = 0; j < 4; j++) {
							ROOM_SET_P = "SET_" + std::to_string(1 + j);

							GetPointPara(&PointTmp, pRoom, ROOM_SET_P,0,0);
							Point_Room.push_back(PointTmp);
						}					
					}
					else {
						ROOM_SET_P = "DoorFrontPos";
						GetPointPara(&PointTmp, pRoom, ROOM_SET_P, 0, 0);
						Point_Room.push_back(PointTmp);
						ROOM_SET_P = "DoorInsidePos";
						GetPointPara(&PointTmp, pRoom, ROOM_SET_P, 0, 0);
						Point_Room.push_back(PointTmp);
						ROOM_SET_P = "StandPos";
						GetPointPara(&PointTmp, pRoom, ROOM_SET_P, 0, 0);
						Point_Room.push_back(PointTmp);
						ROOM_SET_P = "BedPos";
						GetPointPara(&PointTmp, pRoom, ROOM_SET_P, 0, 0);
						Point_Room.push_back(PointTmp);
					}
					
					GetVoicePara(&RoomVoiceTemp, pRoom, "RoomVoiceTime", "RoomVoice");
					RoomVoice.push_back(RoomVoiceTemp);

					if (GetIntPara(&RoomAddress_tmp, pRoom, "RoomAddress")) {
						RoomAddress.push_back(RoomAddress_tmp);
					}
					else {
						RoomAddress.push_back(0x80);
					}
					if (GetTxtPara(&RoomNumTmp, pRoom, "RoomNumber")) {
						RoomNumber.push_back(RoomNumTmp);
					}
					else {
						RoomNumTmp = "R_" +std::to_string(i);
						RoomNumber.push_back(RoomNumTmp);
					}

					if (GetIntPara(&RoomReturnPatrol_tmp, pRoom, "RoomReturnPatrol")) {
						RoomReturnPatrol.push_back(RoomReturnPatrol_tmp - 1);
					}
					else {
						RoomReturnPatrol.push_back(0);
					}

				}
				else {
					break;
				}
			}
		}
	}


	/////////
	void LoadXml_SysConfig(std::string file) {
		std::cout << "LoadXml_SysConfig" << file<< std::endl;
		TiXmlDocument xmlDoc(file.c_str());
		xmlDoc.LoadFile();
		if (xmlDoc.ErrorId() > 0) {
			printf("xmlDoc.LoadFile Fail \n");
			return;
		}
		TiXmlElement* pRootElement = xmlDoc.RootElement();
		if (!pRootElement) {
			printf("pRootElement Load Fail \n");
			return;
		}
		////////////////
				
		std::cout << "GetStringPara" << std::endl;
		GetStringPara(&BlockImgPath, pRootElement, "BlockImgPath");
		GetString485Para(&RS485Port, pRootElement, "RS485Port");
		////////////////
		GetIntPara(&WorkLoopTime, pRootElement, "WorkLoopTime");
		GetIntPara(&WorkLoopSleepTime, pRootElement, "WorkLoopSleepTime");
		GetIntPara(&HitObjectRunawayType, pRootElement, "HitObjectRunawayType");
		GetIntPara(&normalMoveType, pRootElement, "normalMoveType");
		GetIntPara(&DoorOpenType, pRootElement, "DoorOpenType");
		GetIntPara(&DEG_OFFSET_BEFORE, pRootElement, "DEG_OFFSET_BEFORE");
		GetIntPara(&DEBUG_SelPatrolType, pRootElement, "DEBUG_SelPatrolType");
		GetIntPara(&DefaultMonitorType, pRootElement, "DefaultMonitorType");
		GetIntPara(&Motor_ENAFC, pRootElement, "Motor_ENAFC");
		GetIntPara(&Motor_CEMS, pRootElement, "Motor_CEMS");
		GetIntPara(&Motor_CEVAL, pRootElement, "Motor_CEVAL");
		GetIntPara(&Motor_PEMS, pRootElement, "Motor_PEMS");
		GetIntPara(&Motor_PEVAL, pRootElement, "Motor_PEVAL");

		/////////////
		GetBoolPara(&HitObjectStop, pRootElement, "HitObjectStop");
		GetBoolPara(&Debug_SW, pRootElement, "Debug_SW");
		GetBoolPara(&Debug_GetGobalPathTest, pRootElement, "Debug_GetGobalPathTest");
		GetBoolPara(&PatrolVoiceSwitch, pRootElement, "PatrolVoiceSwitch");
		GetBoolPara(&Debug_ShowReadError, pRootElement, "Debug_ShowReadError");
		GetBoolPara(&BatterySensorInstall, pRootElement, "BatterySensorInstall");
		GetBoolPara(&PressSWInstall, pRootElement, "PressSWInstall");
		GetBoolPara(&HitSensorInstall, pRootElement, "HitSensorInstall");
		GetBoolPara(&ChargePlatformSensorInstall, pRootElement, "ChargePlatformSensorInstall");
		GetBoolPara(&LowSensorInstall, pRootElement, "LowSensorInstall");
		GetBoolPara(&DoorClose_RoomPatrol, pRootElement, "DoorClose_RoomPatrol");
		GetBoolPara(&DEBUG_HitObjectStop, pRootElement, "DEBUG_HitObjectStop");
		GetBoolPara(&DEBUG_LowSensorByIO, pRootElement, "DEBUG_LowSensorByIO");

		///////
		GetDoublePara(&ManualCouveRadius, pRootElement, "ManualCouveRadius");
		GetDoublePara(&FrontSafeDistance, pRootElement, "FrontSafeDistance");
		GetDoublePara(&BackSafeDistance, pRootElement, "BackSafeDistance");
		GetDoublePara(&FrontSafeDistance_CamTrac, pRootElement, "FrontSafeDistance_CamTrac");
		GetDoublePara(&BackSafeDistance_CamTrac, pRootElement, "BackSafeDistance_CamTrac");
		GetDoublePara(&SideSafeDistance, pRootElement, "SideSafeDistance");
		GetDoublePara(&ArrivalRange, pRootElement, "ArrivalRange");
		GetDoublePara(&MaxCircleSpd, pRootElement, "MaxCircleSpd");
		GetDoublePara(&MaxLineSpd, pRootElement, "MaxLineSpd");
		GetDoublePara(&MaxTurnSpd, pRootElement, "MaxTurnSpd");
		GetDoublePara(&MinCircleNeedDeg, pRootElement, "MinCircleNeedDeg");
		GetDoublePara(&MinTurnNeedDeg, pRootElement, "MinTurnNeedDeg");
		GetDoublePara(&PatrolOverDeg, pRootElement, "PatrolOverDeg");
		GetDoublePara(&PatrolOverDeg_FinDeg, pRootElement, "PatrolOverDeg_FinDeg");
		GetDoublePara(&normalMovDegOffset, pRootElement, "normalMovDegOffset");
		GetDoublePara(&acc, pRootElement, "acc");
		GetDoublePara(&spd, pRootElement, "spd");
		GetDoublePara(&turnRate, pRootElement, "turnRate");
		GetDoublePara(&DoorOpenDistance, pRootElement, "DoorOpenDistance");
		GetDoublePara(&DoorOpenWidth, pRootElement, "DoorOpenWidth");
		GetDoublePara(&DoorOpenTime_high_T1, pRootElement, "DoorOpenTime_high_T1");
		GetDoublePara(&DoorOpenTime_low_T1, pRootElement, "DoorOpenTime_low_T1");
		GetDoublePara(&DoorOpenTime_high_T2, pRootElement, "DoorOpenTime_high_T2");
		GetDoublePara(&DoorOpenTime_low_T2, pRootElement, "DoorOpenTime_low_T2");
		GetDoublePara(&DoorOpenTime_Wait, pRootElement, "DoorOpenTime_Wait");
		GetDoublePara(&Motor_Current_Limit, pRootElement, "Motor_Current_Limit");
		GetDoublePara(&BackMovEscapeDistance, pRootElement, "BackMovEscapeDistance");
		GetDoublePara(&auto_acc_RPM, pRootElement, "auto_acc_RPM");
		GetDoublePara(&obj_spd_RPM, pRootElement, "obj_spd_RPM");
		GetDoublePara(&obj_acc_RPM, pRootElement, "obj_acc_RPM");

		xmlDoc.Clear();
		printf("Load File Finish\n");
	}

	void LoadXml(std::string file){
		std::cout << "LoadXml" << file<< std::endl;
		TiXmlDocument xmlDoc(file.c_str());
		xmlDoc.LoadFile();
		if (xmlDoc.ErrorId() > 0) {
			printf("xmlDoc.LoadFile Fail \n");
			return;
		}
		TiXmlElement* pRootElement = xmlDoc.RootElement();
		if (!pRootElement) {
			printf("pRootElement Load Fail \n");
			return;
		}
		////////////////
		std::cout << "Patrol_Point" << std::endl;
		pSOWAN = pRootElement->FirstChildElement("Patrol_Point");
		if (pSOWAN)
		{
			TiXmlElement* pRoute = pSOWAN->FirstChildElement("Route");
			if (pRoute) {
				std::string SET_XX;
				Set_Point PointTmp;
				for (int i = 1; i < 1001; i++) {
					SET_XX = "SET_" + std::to_string(i);
					if (GetPointPara(&PointTmp, pRoute, SET_XX)) {
						Point_Route.push_back(PointTmp);
					}
					else {
						if (i == 1) {
							PointTmp.x = 1024;
							PointTmp.y = 1024;
							Point_Route.push_back(PointTmp);
						}
						break;
					}
				}
			}

			GetIntPara(&PatrolCycleWaitTime, pSOWAN, "PatrolCycleWaitTime");
			GetIntPara(&PatrolCycleCount, pSOWAN, "PatrolCycleCount");
			GetVoicePara(&PatrolVoice, pSOWAN, "PatrolVoiceTime", "PatrolVoicePath");
		}

		////////////////
		std::cout << "Room_Patrol_Sequence" << std::endl;
		pSOWAN = pRootElement->FirstChildElement("Room_Patrol_Sequence");
		if (pSOWAN) {
			std::string ROOM_SET_XX;
			int RoomSequence_tmp;
			for (int i = 1; i < 101; i++) {
				ROOM_SET_XX = "ROOM_SET_" + std::to_string(i);
				if (GetIntPara(&RoomSequence_tmp, pSOWAN, ROOM_SET_XX)) {
					RoomSequence.push_back(RoomSequence_tmp - 1);
				}
				else {
					break;
				}
			}
		}
		////////////////
		std::cout << "Charge_Point" << std::endl;
		pSOWAN = pRootElement->FirstChildElement("Charge_Point");
		if (pSOWAN)
		{
			std::string SET_XX;
			Set_Point PointTmp;
			SET_XX = "SET_1" ;
			TiXmlElement* pPos = pSOWAN->FirstChildElement("SET_1");
			if (pPos) {
				for (int i = 1; i < 3; i++) {
					SET_XX = "SET_" + std::to_string(i);
					if (GetPointPara(&PointTmp, pSOWAN, SET_XX)) {
						Point_Charge.push_back(PointTmp);
					}
					else {
						break;
					}
				}
			}
			else {
				SET_XX = "StandPos";
				GetPointPara(&PointTmp, pSOWAN, SET_XX, 0, 0);
				Point_Charge.push_back(PointTmp);
				SET_XX = "PlatformPos";
				GetPointPara(&PointTmp, pSOWAN, SET_XX, 0, 0);
				Point_Charge.push_back(PointTmp);
			}


			GetIntPara(&ChargeWaitTime, pSOWAN, "ChargeWaitTime");
			GetVoicePara(&GoChargeVoice, pSOWAN, "GoChargeVoiceTime", "GoChargeVoicePath");
			GetBoolPara(&ChargePlatform, pSOWAN, "ChargePlatform");
		}
		////////////////
		std::cout << "Get_Custom_Partol" << std::endl;
		Get_Custom_Partol(pRootElement, "Room_Patrol");// Custom_Partol

		////////////////
		std::cout << "Get_RoomData" << std::endl;
		Get_RoomData(pRootElement, "Room_Set");


		////////////////
		pSOWAN = pRootElement->FirstChildElement("VehicleStop");
		if (pSOWAN)
		{
			GetVoicePara(&VehicleStopVoice_Normal, pSOWAN, "VehicleStopVoiceTime_Normal", "VehicleStopVoicePath_Normal");
			GetVoicePara(&VehicleStopVoice_Low, pSOWAN, "VehicleStopVoiceTime_Low", "VehicleStopVoicePath_Low");
		}

		////////////////

		GetBoolPara(&AutoMotorOn, pRootElement, "AutoMotorOn");


		////////////////
		pSOWAN = pRootElement->FirstChildElement("ReturnOriginVoice");
		if (pSOWAN)
		{
			bool res;
			res = GetVoicePara(&ReturnOriginVoice_Normal, pSOWAN, "ReturnOriginVoice_Normal_Time", "ReturnOriginVoice_Normal_Path");
			if (!res) {
				ReturnOriginVoice_Normal = PatrolVoice;
				std::cout << "ReturnOriginVoice_Normal_Time" << std::endl;
				std::cout << "Path" << ":(" << ReturnOriginVoice_Normal.Path << ")" << std::endl;
				std::cout << "time" << ":(" << ReturnOriginVoice_Normal.Time << ")" << std::endl;
			}
			res = GetVoicePara(&ReturnOriginVoice_Charge, pSOWAN, "ReturnOriginVoice_Charge_Time", "ReturnOriginVoice_Charge_Path");
			if (!res) {
				ReturnOriginVoice_Charge = GoChargeVoice;
				std::cout << "ReturnOriginVoice_Normal_Time" << std::endl;
				std::cout << "Path" << ":(" << ReturnOriginVoice_Charge.Path << ")" << std::endl;
				std::cout << "time" << ":(" << ReturnOriginVoice_Charge.Time << ")" << std::endl;
			}
			res = GetVoicePara(&ReturnOriginVoice_RoomCall, pSOWAN, "ReturnOriginVoice_RoomCall_Time", "ReturnOriginVoice_RoomCall_Path");
			if (!res) {
				ReturnOriginVoice_RoomCall = PatrolVoice;
				std::cout << "ReturnOriginVoice_RoomCall" << std::endl;
				std::cout << "Path" << ":(" << ReturnOriginVoice_RoomCall.Path << ")" << std::endl;
				std::cout << "time" << ":(" << ReturnOriginVoice_RoomCall.Time << ")" << std::endl;
			}
			res = GetVoicePara(&ReturnOriginVoice_RoomPatrol, pSOWAN, "ReturnOriginVoice_RoomPatrol_Time", "ReturnOriginVoice_RoomPatrol_Path");
			if (!res) {
				ReturnOriginVoice_RoomPatrol = RoomPatrolVoice;
				std::cout << "ReturnOriginVoice_RoomPatrol" << std::endl;
				std::cout << "Path" << ":(" << ReturnOriginVoice_RoomPatrol.Path << ")" << std::endl;
				std::cout << "time" << ":(" << ReturnOriginVoice_RoomPatrol.Time << ")" << std::endl;
			}
			res = GetVoicePara(&ReturnOriginVoice_Patrol, pSOWAN, "ReturnOriginVoice_Patrol_Time", "ReturnOriginVoice_Patrol_Path");
			if (!res) {
				ReturnOriginVoice_Patrol = PatrolVoice;
				std::cout << "ReturnOriginVoice_Patrol" << std::endl;
				std::cout << "Path" << ":(" << ReturnOriginVoice_Patrol.Path << ")" << std::endl;
				std::cout << "time" << ":(" << ReturnOriginVoice_Patrol.Time << ")" << std::endl;
			}
			res = GetVoicePara(&RoomCallEndReturnPatrol, pSOWAN, "RoomCallEndReturnPatrol_Time", "RoomCallEndReturnPatrol_Path");
			if (!res) {
				RoomCallEndReturnPatrol = PatrolVoice;
				std::cout << "RoomCallEndReturnPatrol" << std::endl;
				std::cout << "Path" << ":(" << RoomCallEndReturnPatrol.Path << ")" << std::endl;
				std::cout << "time" << ":(" << RoomCallEndReturnPatrol.Time << ")" << std::endl;
			}
		}
		else {
			std::cout << "[Def ]: ReturnOriginVoice_XXXX" << std::endl;
			std::cout << "[Def ]: RoomCallEndReturnPatrol" << std::endl;
			ReturnOriginVoice_Normal = PatrolVoice;
			ReturnOriginVoice_Charge = GoChargeVoice;
			ReturnOriginVoice_RoomCall = PatrolVoice;
			ReturnOriginVoice_RoomPatrol = RoomPatrolVoice;
			ReturnOriginVoice_Patrol = PatrolVoice;
			RoomCallEndReturnPatrol = PatrolVoice;
		}

		
		GetVoicePara(&DoorNotOpenVoice, pRootElement, "DoorNotOpenVoiceTime", "DoorNotOpenVoicePath");

		/////////

		xmlDoc.Clear();
		printf("Load File Finish\n");
	}


};
