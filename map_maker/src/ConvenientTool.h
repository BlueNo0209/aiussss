#pragma once
#ifndef __CONVENTIENT_TOOL__
#define __CONVENTIENT_TOOL__

#include <iostream> 
#include <string> 
#include <vector>
#include <fstream>
#include <fstream>
#include <math.h>
#define TOOL_PI 3.14159265358979323846   // pi

//using namespace System;
//using namespace System::ComponentModel;
//using namespace System::Collections;
//using namespace System::Windows::Forms;
//using namespace System::Data;
//using namespace System::Drawing;

enum KAJIMA_CAR_MOV {
	SERV_ON,
	SERV_OFF,
	STOP,
	FWD,
	BACK,
	LEFT,
	RIGHT,
	TURN_LEFT,
	TURN_RIGHT,
	FWD_LEFT,
	FWD_RIGHT,
	BACK_LEFT,
	BACK_RIGHT,

};

class ConvenientTool
{
public:
	const char IntToChar[17] = "0123456789ABCDEF";
public:
	ConvenientTool() {
	};
	~ConvenientTool() {
	};
#ifdef _WIN32
	void MarshalString(String^ s, std::string& os) {
		using namespace Runtime::InteropServices;
		const char* chars =
			(const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
		os = chars;
		Marshal::FreeHGlobal(IntPtr((void*)chars));
	}

	int TextToInt(System::String^ Value) {
		int tmp;
		System::String^ string;
		try {
			tmp = System::Convert::ToInt32(Value);
		}
		catch (System::FormatException^ e) {
			tmp = 0;
		}
		catch (System::OverflowException^ e) {
			tmp = 0;
		}
		finally {
		}
		return tmp;
	}
#endif
	int CombineCharToInt(char *tmp ,int count) {
		int tmpV = 0;
		for (int i = 0 ; i < count; i++) {
			tmpV = tmpV | ((tmp[i] & 0x0ff) << ((count - i - 1) * 8));
		}
		//tmpV = (tmp[0] & 0x0ff);
		//tmpV = (tmpV << 8) | (tmp[1] & 0x0ff);
		//tmpV = (tmpV << 8) | (tmp[2] & 0x0ff);
		//tmpV = (tmpV << 8) | (tmp[3] & 0x0ff);	
		return tmpV;
	}
	void cutIntToChar(char* tmpc, int input,int count) {

		for (int i = count - 1; i >= 0; i--) {
			tmpc[i] = input & 0x0ff;
			input = input >> 8;
		}
		//buf[tmp + 5] = Value & 0x0ff;
		//Value = Value >> 8;
		//buf[tmp + 4] = (Value) & 0x0ff;
		//Value = Value >> 8;
		//buf[tmp + 3] = (Value) & 0x0ff;
		//Value = Value >> 8;
		//buf[tmp + 2] = (Value) & 0x0ff;
	}
	void ShowMat(char *mat,int size) {
		std::cout << "[";
		for (int i = 0; i < size; i += 1) {
			std::cout << (int)(mat[i] & 0x0ff) << "_";
		}
		std::cout <<"]"<< std::endl;
	}
	void Get_CarMov_SpdPara(char Direction, double Spd, double* WorkSpd) {
		switch (Direction) {
		case KAJIMA_CAR_MOV::FWD:
			WorkSpd[0] = 2.366219 * Spd;
			WorkSpd[1] = -2.366219 * Spd;
			WorkSpd[2] = 0;
			break;
		case KAJIMA_CAR_MOV::BACK:
			WorkSpd[0] = -2.366219 * Spd;
			WorkSpd[1] = 2.366219 * Spd;
			WorkSpd[2] = 0;
			break;
		case KAJIMA_CAR_MOV::LEFT:
			WorkSpd[0] = -1.35 * Spd;
			WorkSpd[1] = -1.35 * Spd;
			WorkSpd[2] = 2.7 * Spd;
			break;
		case KAJIMA_CAR_MOV::RIGHT:
			WorkSpd[0] = 1.35 * Spd;
			WorkSpd[1] = 1.35 * Spd;
			WorkSpd[2] = -2.7 * Spd;
			break;
		case KAJIMA_CAR_MOV::TURN_LEFT:
			WorkSpd[0] = 0.75 * Spd;
			WorkSpd[1] = 0.75 * Spd;
			WorkSpd[2] = 0.75 * Spd;
			break;
		case KAJIMA_CAR_MOV::TURN_RIGHT:
			WorkSpd[0] = -0.75 * Spd;
			WorkSpd[1] = -0.75 * Spd;
			WorkSpd[2] = -0.75 * Spd;
			break;
		case KAJIMA_CAR_MOV::FWD_LEFT:
			WorkSpd[0] = 0;
			WorkSpd[1] = -2.366219 * Spd;
			WorkSpd[2] = 2.366219 * Spd;
			break;
		case KAJIMA_CAR_MOV::FWD_RIGHT:
			WorkSpd[0] = 2.366219 * Spd;
			WorkSpd[1] = 0;
			WorkSpd[2] = -2.366219 * Spd;
			break;
		case KAJIMA_CAR_MOV::BACK_LEFT:
			WorkSpd[0] = -2.366219 * Spd;
			WorkSpd[1] = 0;
			WorkSpd[2] = 2.366219 * Spd;
			break;
		case KAJIMA_CAR_MOV::BACK_RIGHT:
			WorkSpd[0] = 0;
			WorkSpd[1] = 2.366219 * Spd;
			WorkSpd[2] = -2.366219 * Spd;
			break;
		}
	}
	void Get_CarMov_StepPara(char Direction, double Value, double* WorkStep) {
		///degree (deg) = (Value / 360.0) * 387363;
		/// distance (mm) = Value * 400
		int resDisValue = 400 * Value;
		int resDegValue = 1076.0083 * Value;//(Value / 360.0) * 387363;
		int resShiftValue = 225.4221 * Value;
		switch (Direction) {
		case KAJIMA_CAR_MOV::FWD:
			WorkStep[0] = resDisValue;
			WorkStep[1] = -resDisValue;
			WorkStep[2] = 0;
			break;
		case KAJIMA_CAR_MOV::BACK:
			WorkStep[0] = -resDisValue;
			WorkStep[1] = resDisValue;
			WorkStep[2] = 0;
			break;
		case KAJIMA_CAR_MOV::LEFT:
			resDisValue = int(resDisValue * 0.5597);
			WorkStep[0] = -resDisValue;
			WorkStep[1] = -resDisValue;
			WorkStep[2] = 2 * resDisValue;
			break;
		case KAJIMA_CAR_MOV::RIGHT:
			resDisValue = int(resDisValue * 0.5597);
			WorkStep[0] = resDisValue;
			WorkStep[1] = resDisValue;
			WorkStep[2] = -2 * resDisValue;
			break;
		case KAJIMA_CAR_MOV::TURN_LEFT:
			WorkStep[0] = resDegValue;
			WorkStep[1] = resDegValue;
			WorkStep[2] = resDegValue;
			break;
		case KAJIMA_CAR_MOV::TURN_RIGHT:
			WorkStep[0] = -resDegValue;
			WorkStep[1] = -resDegValue;
			WorkStep[2] = -resDegValue;
			break;
		case KAJIMA_CAR_MOV::FWD_LEFT:
			WorkStep[0] = 0;
			WorkStep[1] = -resDisValue;
			WorkStep[2] = resDisValue;
			break;
		case KAJIMA_CAR_MOV::FWD_RIGHT:
			WorkStep[0] = resDisValue;
			WorkStep[1] = 0;
			WorkStep[2] = -resDisValue;
			break;
		case KAJIMA_CAR_MOV::BACK_LEFT:
			WorkStep[0] = -resDisValue;
			WorkStep[1] = 0;
			WorkStep[2] = resDisValue;
			break;
		case KAJIMA_CAR_MOV::BACK_RIGHT:
			WorkStep[0] = 0;
			WorkStep[1] = resDisValue;
			WorkStep[2] = -resDisValue;
			break;
		}
	}
	double targetRadReturn(double self_x, double self_y, double tag_x, double tag_y) {

		double deltaX = tag_x - self_x;
		double deltaY = tag_y - self_y;
		//double deltaX = tag_x;
		//double deltaY = tag_y;

		double MoveDir = atan(deltaY / deltaX);

		if (deltaX < 0)
			MoveDir += TOOL_PI;
		while (MoveDir < 0)
			MoveDir += (2 * TOOL_PI);
		while (MoveDir > (2 * TOOL_PI))
			MoveDir -= (2 * TOOL_PI);

		return MoveDir;
	}

	double targetDegReturn(double self_x, double self_y, double tag_x, double tag_y){
		return (targetRadReturn(self_x, self_y, tag_x, tag_y) * 180 / TOOL_PI);
	}
	double GetTagDegDiff(double SelfDir, double TagDir) {
		double MoveDir = TagDir;
		double nowD = SelfDir;
		double savetmd2_1, savetmd2_2, savetmd2_work;

		savetmd2_1 = MoveDir - nowD;
		if (savetmd2_1 > 0)
			savetmd2_2 = savetmd2_1 - 360;
		else if (savetmd2_1 < 0)
			savetmd2_2 = 360 + savetmd2_1;
		else
			savetmd2_2 = 0;
		if (fabs(savetmd2_1) > fabs(savetmd2_2)) {
			savetmd2_work = savetmd2_2;
			savetmd2_2 = savetmd2_1;
			savetmd2_1 = savetmd2_work;
		}
		else
			savetmd2_work = savetmd2_1;

		return savetmd2_work;
	}
	double targetDistanceReturn(double self_x, double self_y, double tag_x, double tag_y) {
		double deltaX = tag_x - self_x;
		double deltaY = tag_y - self_y;
		//double deltaX = tag_x;
		//double deltaY = tag_y;
		double distance = pow(pow(deltaX, 2) + pow(deltaY, 2), 0.5);
		return distance;
	}


private:

};



#endif // !1