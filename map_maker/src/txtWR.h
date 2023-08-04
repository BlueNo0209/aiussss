#pragma once
#ifndef __TXT_OUTPUT__
#define __TXT_OUTPUT__


#define _TXT_OUTPUT_SETTING_ 1
#include <iostream> 
#include <string> 
#include <vector>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include <pwd.h>
#include <ctime>

#define FILE_WRITE_SW 1

#define SHOWFNC_N(...) printf(__VA_ARGS__);printf("\n")
#define SHOWFNC(...) printf(__VA_ARGS__);
#define STRING_OUTPUT(STRING,format,...) sprintf(STRING,format,##__VA_ARGS__);printf("%s",STRING)
#define STRING_OUTPUT_N(STRING,format,...) sprintf(STRING,format,##__VA_ARGS__);printf("%s\n",STRING)
#define STRING_OUTPUT_R(STRING,format,...) sprintf(STRING,format,##__VA_ARGS__);printf("%s\r",STRING)
#define TXT_FUNC(STRING,format,...) STRING_OUTPUT(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr(STRING)
#define TXT_FUNC_N(STRING,format,...) STRING_OUTPUT_N(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr_N(STRING)
#define TXT_FUNC_R(STRING,format,...) STRING_OUTPUT(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr(STRING)

#define TXT_FUNC_TAG(TAG,STRING,format,...) STRING_OUTPUT(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr(TAG,STRING)
#define TXT_FUNC_TAG_N(TAG,STRING,format,...) STRING_OUTPUT_N(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr_N(TAG,STRING)

#ifdef FILE_WRITE_SW
	#define TXT_FUNC_TAG(TAG,STRING,format,...) STRING_OUTPUT(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr(TAG,STRING)
	#define TXT_FUNC_TAG_N(TAG,STRING,format,...) STRING_OUTPUT_N(STRING,format,##__VA_ARGS__);OutputText->Write_CharPtr_N(TAG,STRING)
#else
	#define TXT_FUNC_TAG(TAG,STRING,format,...) STRING_OUTPUT(STRING,format,##__VA_ARGS__);
	#define TXT_FUNC_TAG_N(TAG,STRING,format,...) STRING_OUTPUT_N(STRING,format,##__VA_ARGS__);
#endif



#define Target_Path "/Desktop/Log/"

class Save_ReadWrite {
public:
	struct recovery_Data
	{
		int x = 0;
		int y = 0;
		int deg = 0;
		int Mode = 0;
		int Step = 0;
	};
public:
	recovery_Data recoveryData;

	bool GetData = false;

	Save_ReadWrite() {
		getHomePath();
		BackupPreFile();
	}
	~Save_ReadWrite() {
	}
private:
	std::string UserName;
	std::string HomePath;
	std::string SowanPath;
	std::string PathCatkin_Sowan = "/catkin_ws/src/sowan";
	std::string RecoveryFilePath;
	std::string SaveFilePath;
	std::string SavePerFilePath;

	std::string strArr;
	std::string strLine;
	
	void getHomePath()
	{
		uid_t userid;
		struct passwd* pwd;
		userid = getuid();
		pwd = getpwuid(userid);

		UserName = (std::string)pwd->pw_name;
		HomePath = (std::string)pwd->pw_dir;
		SowanPath = HomePath + PathCatkin_Sowan;
		RecoveryFilePath = SowanPath + "/recovery.dat";
		SaveFilePath = SowanPath + "/SaveFile.dat";
		SavePerFilePath = SowanPath + "/SaveFilePre.dat";

		std::cout << "RecoveryFilePath :" << RecoveryFilePath << std::endl;
	}

	bool ReadData(std::string Path) {
		std::fstream TXT_Ptr_read;
		TXT_Ptr_read.open(Path.c_str());
		if (!TXT_Ptr_read.is_open())
		{
			return false;
		}
		getline(TXT_Ptr_read, strArr);
		recoveryData.Mode = GetIntPara(strArr);
		getline(TXT_Ptr_read, strArr);
		recoveryData.x = GetIntPara(strArr);
		getline(TXT_Ptr_read, strArr);
		recoveryData.y = GetIntPara(strArr);
		getline(TXT_Ptr_read, strArr);
		recoveryData.deg = GetIntPara(strArr);
		getline(TXT_Ptr_read, strArr);
		recoveryData.Step = GetIntPara(strArr);

		TXT_Ptr_read.close();

		return true;
	}
public:
	bool ReadData_Recovery() {
		return ReadData(RecoveryFilePath);
	}
	bool ReadData_SaveFile() {
		return ReadData(SavePerFilePath);
	}

	void WriteDate(int mode, int x,int y,int deg,int step) {
		//std::ofstream TXT_tmp;
		std::ofstream TXT_tmp;
		TXT_tmp.open(SaveFilePath.c_str());
		TXT_tmp << mode << std::endl
		 << x << std::endl
		 << y << std::endl
		 << deg << std::endl
		 << step << std::endl;
		TXT_tmp.close();
	}

	void BackupPreFile() {
		std::fstream TXT_Ptr_read;
		std::ofstream TXT_write;
		TXT_Ptr_read.open(SaveFilePath.c_str());
		if (!TXT_Ptr_read.is_open())
		{
			return;
		}
		TXT_write.open(SavePerFilePath.c_str());

		for (int i = 0; i < 5; i++) {
			getline(TXT_Ptr_read, strArr);
			TXT_write << strArr << std::endl;
		}
		TXT_write.close();
		TXT_Ptr_read.close();

	}
private:
	double GetDoublePara(std::string tag) {		
		double data = strtod(tag.c_str(), NULL);
		std::cout << "recoveryData :" << data << std::endl;
		return data;
	}
	int GetIntPara(std::string tag) {
		double data = atoi(tag.c_str());
		std::cout << "recoveryData :" << data << std::endl;
		return data;
	}

};

//using namespace std;
class TXT_ReadWrite {
public:
	bool Write_Work = true;
	bool TimeStampOn = false;
	int Rec_Stamp = 0;
	

private:
	std::fstream TXT_Ptr_read;
	std::ofstream TXT_Ptr_write;
	std::ofstream Log_Ptr;
	std::string Home_Path;

	std::string strArr;
	std::string strLine;
	int Limit_Dir = 100;
	double Time_Stamp = 0;
	bool frist_Time_Stamp = true;
	std::string ouputPath;

public:
	TXT_ReadWrite(std::string HomePath,std::string ImgFileName, std::string SAVE_FileName, std::string Type,int LimitDir = 100) {
		if (LimitDir > 999) {
			Limit_Dir = 999;
		}
		else
			Limit_Dir = LimitDir;

		std::cout << "HomePath :" << HomePath << std::endl;
		std::cout << "ImgFileName :" << ImgFileName << std::endl;
		std::cout << "SAVE_FileName :" << SAVE_FileName << std::endl;
		std::cout << "Type :" << Type << std::endl;
		std::cout << "Home_Path :" << Home_Path << std::endl;
		Home_Path = HomePath;
		
		TXT_LoadInit(HomePath + ImgFileName, SAVE_FileName, Type);

	}
	~TXT_ReadWrite() {

	}
	void TxtClose() {
		std::cout << "SAVE_FileName :" << ouputPath << std::endl;
		//Log_Ptr << "TXT WRITE PROGRAM END" << std::endl;
		//std::cout << "TXT WRITE PROGRAM END" << std::endl;
		//Log_Ptr << "PROGRAM CLOSE" << std::endl;
		Log_Ptr.close();
	}
	void SetTimeStamp(double TimeStamp) {
		Time_Stamp = TimeStamp;
	}

	void TXT_LoadInit(std::string ImgFileName, std::string SAVE_FileName, std::string Type) {

		//printf("TXT_LoadInit \n%s\n%s ", ImgFileName, SAVE_FileName);
		std::cout << "TXT_LoadInit :" << std::endl;
		std::cout << "ImgFileName :" << ImgFileName << std::endl;
		std::cout << "SAVE_FileName :" << SAVE_FileName << std::endl;
		
		TXT_Ptr_read.open(ImgFileName.c_str());
		if (!TXT_Ptr_read.is_open())
		{
			std::ofstream TXT_tmp;
			TXT_tmp.open(ImgFileName.c_str());
			TXT_tmp << 0;
			TXT_tmp.close();
		}

		getline(TXT_Ptr_read, strArr);
		TXT_Ptr_read.close();

		int count = 0;
		count = atoi(strArr.c_str());
		printf("count = %d to ", count);
		if (count >= Limit_Dir)
			count = 0;
		else
			count++;
		printf("count = %d \n", count);

		TXT_Ptr_write.open(ImgFileName.c_str());
		TXT_Ptr_write <<count;
		TXT_Ptr_write.close();

		std::string tmps = std::to_string(count);
		while (tmps.size() < 3) {
			tmps = "0" + tmps;
		}
		ouputPath = Home_Path + Target_Path + SAVE_FileName + tmps + Type;
		std::cout<< "Final ouput_FileName :"<< ouputPath <<std::endl;

		Log_Ptr.open(ouputPath.c_str());

	}
	void TXT_Write(std::string InputInfo) {
		if (Write_Work)
			Log_Ptr << InputInfo;
	}
	void Write_CharPtr(char *Wdata) {
		if (Write_Work)
			Log_Ptr << Wdata;
	}


	void TXT_Write_N(std::string InputInfo) {
		TXT_Write(InputInfo + "\n");
	}

	void TimeStamp() {
		std::time_t t = std::time(0);   // get time now
		std::tm* now = std::localtime(&t);
		Log_Ptr <<"[" <<(now->tm_year + 1900) << '-'
			<< (now->tm_mon + 1) << '-'
			<< now->tm_mday << '_'
			<< now->tm_hour << ':'
			<< now->tm_min << ':'
			<< now->tm_sec
			<< "]";
		Log_Ptr << "[" << std::to_string(Time_Stamp) << "]";
	
	}
	void Write_CharPtr(char *Tag,char *Wdata) {
		if (TimeStampOn) {
			TimeStamp();
		}
		if (Write_Work) {
			Log_Ptr <<"["<< Tag <<"]" << Wdata;
		}
	}
	void Write_CharPtr_N(char *Tag, char *Wdata) {
		if (TimeStampOn) {
			TimeStamp();
		}
		if (Write_Work) {
			Log_Ptr << "[" << Tag << "]" << Wdata<<std::endl;
		}
	}

	void Write_CharPtr_N(char *Wdata) {
		if (Write_Work) {
			Log_Ptr <<  Wdata << std::endl;
		}
	}
	void TXT_Write_int(int InputInfo) {
		TXT_Write(std::to_string(InputInfo));
	}

};


#endif