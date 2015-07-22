
/*
 * Copyright 2015 Intelligent Vehicle Lab of CQUPT.
 * Author :Tian Zhen   
 * Date:2015.5.4 
 *	
 * 这是一个Ibeo LUX系列四线激光雷达的数据采集文件
 *    
 *
 */

#pragma once
#define M_PI 3.14159265358979323846

#include "database.h"

#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>
#include <vector>

//======================================================================


using namespace std;
using namespace ibeo;

class IbeoLidar 
{
public:
	void init(int);
	bool isLidarWork(void);
	void beginScan(int);
	void PreProLidar(void);

private:
	ibeo::ObjectListLux pObj;
	ibeo::ScanLux scan;
	ibeo::LogFileManager logFileManager;
	ibeo::LogFile logfile;
};


class IbeoData
{
public:

	vector<LidarData> m_LidarScanFRight;
	vector<LidarData> m_LidarScanFLeft;
	vector<LidarTargetData> m_LidarObjectFLeft;
	vector<LidarTargetData> m_LidarObjectFRight;

private:

};


extern IbeoData TempIbeoData;
extern IbeoLidar Fleft;
extern IbeoLidar FRight;

