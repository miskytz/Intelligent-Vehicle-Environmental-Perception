
/*
 * Copyright 2015 Intelligent Vehicle Lab of CQUPT.
 * Author :Tian Zhen   
 * Date:2015.5.4 
 *	
 * 这是一个lidar单线激光雷达数据采集的头文件，定义了一个lidar类
 *    IbeoLidar类是雷达采集类，init对IBEO四线雷达进行初始化
 *
 */
#pragma once
#include "database.h"
#include "Urg_driver.h"
#include <vector>


using namespace qrk;
using namespace std;


class SingleLineLidar
{
public:
	bool open(int);
	void beginScan(int);
	void preProData(void);

	Urg_driver urg;
	vector<LidarData> m_LidarScanData;
	vector<LidarTargetData> m_LidarTarget;

protected:
private:
		
};

extern SingleLineLidar lidar_BR;extern SingleLineLidar lidar_BL;extern SingleLineLidar lidar_ML;
extern SingleLineLidar lidar_MR;





