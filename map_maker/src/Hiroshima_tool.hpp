#ifndef __SOWAN_TOOL_H__
#define __SOWAN_TOOL_H__

#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <iostream>


#define VOICEINFO VoicePlayInfo

struct VoicePlayInfo
{
	std::string Path;
	int Time;
};

class CustomPartolData {
private:
	struct MyStruct
	{
		char Type;
		cv::Point Pos;
		int Room;
	};
	MyStruct Tmp;
	void SetData() {
		Data.push_back(Tmp);
		WorkState.push_back(0);
	}

public:
	int WorkProcess = -1;
	std::vector<MyStruct> Data;
	std::vector<int> WorkState;
	int PartolEntry = -1;
	int EndPartolSize = -1;

	void SetDataRoom(int room) {
		Tmp.Room = room;
		Tmp.Type = 'R';
		Tmp.Pos = cv::Point(0, 0);
		SetData();
	}
	void SetDataPos(int x, int y) {
		Tmp.Room = -1;
		Tmp.Type = 'P';
		Tmp.Pos = cv::Point(x, y);
		SetData();
	}
	void DisableRoom(int InputRoom) {
		for (int i = 0; i < WorkState.size(); i++) {
			if (Data.at(i).Type == 'R' && Data.at(i).Room == InputRoom) {
				Data.at(i).Type = 'r';
			}
		}
	}
	void EnableRoom(int InputRoom) {
		for (int i = 0; i < WorkState.size(); i++) {
			if (Data.at(i).Type == 'r' && Data.at(i).Room == InputRoom) {
				Data.at(i).Type = 'R';
			}
		}
	}
	void EnableAllRoom() {
		for (int i = 0; i < WorkState.size(); i++) {
			if (Data.at(i).Type == 'r' ) {
				Data.at(i).Type = 'R';
			}
		}
	}



	void  WorkStateInit() {
		int size = Data.size();
		WorkState.clear();
		WorkState.assign(size, 0);
	}
	int GetProcessNow() {
		WorkProcess = -1;
		for (int i = 0; i < WorkState.size();i++) {		
			///
			char Ttype = Data.at(i).Type;
			int WorkStateT = WorkState.at(i);
			///
			if ((WorkStateT == 0 ) && (Ttype != 'r')) {
				WorkProcess = i;
				if ((EndPartolSize >= 0) && (WorkProcess >= EndPartolSize)) {
					WorkProcess = EndPartolSize;
				}
				return WorkProcess;
				break;
			}
		}
		if ( (WorkProcess == -1 ) && (WorkState.at(WorkState.size()-1) == 1)) {
			WorkProcess = WorkState.size();
		}
		return WorkProcess;
	}
	void FinishWork() {
	
	}
};


#endif