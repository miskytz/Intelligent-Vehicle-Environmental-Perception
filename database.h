/************************************************************************/
/*  Copyright 2015 Intelligent Vehicle Lab of CQUPT.
/* Author :Tian Zhen   
/* 2015.5.4 
/************************************************************************/
#include <string>
#include <iostream>
#pragma once

using namespace std;

//全局变量;

//声明全局变量;
extern int g_nLidarFlag;
extern int g_nMapUpdateTime;


//雷达命名宏定义;
enum LidarName{  FrontLeft=0,FrontRight=1, MidLeft=2, MidRight=3,BackLeft=4,BackRight=5};

//雷达IP的字符串，其IP值与名字宏定义相对于比如 ip[FrontLeft]代表前左方的雷达;
const string ip[6]={"192.168.1.30","192.168.1.31","192.168.1.20","192.168.1.21","192.168.1.22","192.168.1.23"};


//雷达是否打开标志量,0为关闭，1位打开;
// FrontLeft=0,FrontRight=1, MidLeft=2, MidRight=3,BackLeft=4,BackRight
const int g_nOpenflag[6]={0, 0, 0, 0, 1, 0 }; 


// 雷达偏移信息，用于雷达坐标转换，单位cm;                          
//front left lidar,ibeo; 
const float LIDAR_FRONT_LEFT_POSITION_X=-60.5;   //X轴偏移量;
const float LIDAR_FRONT_LEFT_POSITION_Y=-27.8;		//Y轴偏移量;
const float LIDAR_FRONT_LEFT_ANGLE=0.56;	//旋转角度;

//front right lidar,ibeo; 
const float LIDAR_FRONT_RIGHT_POSITION_X=66.3;
const float LIDAR_FRONT_RIGHT_POSITION_Y=-34.2;
const float LIDAR_FRONT_RIGHT_ANGLE=1;

//Middle left lidar; 
const float LIDAR_MIDDLE_LEFT_POSITION_X=-75.1;
const float LIDAR_MIDDLE_LEFT_POSITION_Y=-160.0;
const float LIDAR_MIDDLE_LEFT_ANGLE=92.22;

//Middle right lidar;  
const float LIDAR_MIDDLE_RIGHT_POSITION_X=88.5;
const float LIDAR_MIDDLE_RIGHT_POSITION_Y=-143.4;
const float LIDAR_MIDDLE_RIGHT_ANGLE=-90.75;

//back left lidar;  
const float LIDAR_BACK_LEFT_POSITION_X=-50.7;
const float LIDAR_BACK_LEFT_POSITION_Y=-530.7;
const float LIDAR_BACK_LEFT_ANGLE=136.2;

//back right lidar;  
const float LIDAR_BACK_RIGHT_POSITION_X=51.1;
const float LIDAR_BACK_RIGHT_POSITION_Y=-549.0;
const float LIDAR_BACK_RIGHT_ANGLE=-136.9;


//	雷达扫描在显示中的范围变量，由于图像显示是在-1到1之间;
//	所以需要进行一定的转换，显示范围为-5000cm到5000cm，需要除以比例变量;
const float LIDAR_DISPLAY_RANGE=2000;


//	雷达扫描点数据类型，包含扫描得到的X,Y坐标;
//	使用get，set方法对成员变量进行赋值或取值;
class LidarData
{
public:
	void SetScanX(int x){this->LidarScanX=x;}
	void SetScanY(int y){this->LidarScanY=y;}
	float GetScanX(){return LidarScanX;}
	float GetScanY(){return LidarScanY;}


private:
	float LidarScanX;
	float LidarScanY;

};

//	雷达目标数据类型，包含目标，长length,宽width,坐标x,y，速度v, 目标类别，包含线段数;
//	使用get，set方法对成员变量进行赋值或取值;
class LidarTargetData
{
public:
	void SetTargetX(int x){this->TargetPostionX=x;}
	void SetTargetY(int y){this->TargetPostionY=y;}
	void SetTargetLength(int Length){this->TargetLength=Length;}
	void SetTargetWidth(int Width){this->TargetWidth=Width;}
	void SetTargetVelocity(int Velocity){this->TargetVelocity=Velocity;}
	void SetTargetClass(int name){this->TargetClass=name;}
	void SetTargetLineNum(int LineNum){this->TargetLineNum=LineNum;}

	int GetTargetX(){return TargetPostionX;}
	int GetTargetY(){return TargetPostionY;}
	int GetTargetLength(){return TargetLength;}
	int GetTargetWidth(){return TargetWidth;}
	int GetTargetVelocity(){return TargetVelocity;}
	int GetTargetClass(){return TargetClass;}
	int GetTargetLineNum(){return TargetLineNum;}

private:
	int TargetPostionX;
	int TargetPostionY;
	int TargetLength;
	int TargetWidth;
	int TargetVelocity;
	int TargetClass;
	int TargetLineNum;

};


//聚类点数据类型;
class BreakPointData
{
public:
	void SetObjectId(int x){this->ObjectId=x;}
	void SetStartPosition(int x){this->StartPosition=x;}
	void SetEndPosition(int x){this->EndPosition=x;}
	void SetLineNum(int x){this->LineNum=x;}

	int GetObjectId(){return ObjectId;}
	int GetStartPosition(){return StartPosition;}
	int GetEndPosition(){return EndPosition;}
	int GetPointNum(){return EndPosition-StartPosition+1;}
	int GetLineNum(){return LineNum;}

private:
	int ObjectId;		//	breakpoint ID，
	int StartPosition;	//	breakpoint起点位置，指向容器相应数据
	int EndPosition;	//	breakpoint起点坐标
	int PointNum;    //breakpoint包含的点数
	int LineNum;    //
};
