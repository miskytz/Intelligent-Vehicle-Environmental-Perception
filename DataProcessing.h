
/************************************************************************/
	/* 根据系统设计的要求， 辆前方长 80m，宽 20m;
	    车辆左侧、右侧、后方 10m 建立网格地图;
		清除范围外的点;
	*/
/************************************************************************/


#pragma once

#include "database.h"
#include <list>
#include <vector>
using namespace std;

const int s_nGridFrontLenth=8000; //网格前方长度单位8000cm;
const int s_nGridBackLenth=1000;
const int s_nGridWidth=1000;
const int s_nCarLenth=1000;
const int s_nCarWidth=1000;



// 申明函数;
int DataClear(vector<LidarData> &data);
float TurnToRange(vector<LidarData> &data,int n);
int ClusterLidar(vector<LidarData> &Vectordata);
void IepfAlgorithm(vector<LidarData> &data);
void LeastSquareMethod(vector<BreakPointData> &breakdata,vector<LidarData> &data);
void FeatureExtraction(vector<BreakPointData> &breakdata,vector<LidarData> &data,vector<LidarTargetData> &Target);