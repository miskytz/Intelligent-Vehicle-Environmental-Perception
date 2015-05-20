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
const int g_nOpenflag[6]={1, 0, 0, 0, 0, 0 }; 


// 雷达偏移信息，用于雷达坐标转换，单位cm;                          
//front left lidar,ibeo;
const float LIDAR_FRONT_LEFT_POSITION_X=-68.2;   //X轴偏移量;
const float LIDAR_FRONT_LEFT_POSITION_Y=-1.5;		//Y轴偏移量;
const float LIDAR_FRONT_LEFT_POSITION_RATE=1;	//比例，旋转角度的乘以的系数;

//front right lidar,ibeo; 
const float LIDAR_FRONT_RIGHT_POSITION_X=68.2;
const float LIDAR_FRONT_RIGHT_POSITION_Y=-1.5;
const float LIDAR_FRONT_RIGHT_POSITION_RATE=1;

//Middle left lidar; 
const float LIDAR_MIDDLE_LEFT_POSITION_X=-99.1;
const float LIDAR_MIDDLE_LEFT_POSITION_Y=-134.5;
const float LIDAR_MIDDLE_LEFT_POSITION_RATE=1;

//Middle right lidar;  
const float LIDAR_MIDDLE_RIGHT_POSITION_X=99.1;
const float LIDAR_MIDDLE_RIGHT_POSITION_Y=-134.5;
const float LIDAR_MIDDLE_RIGHT_POSITION_RATE=1;

//back left lidar;  
const float LIDAR_BACK_LEFT_POSITION_X=-67.2;
const float LIDAR_BACK_LEFT_POSITION_Y=-486.6;
const float LIDAR_BACK_LEFT_POSITION_COS=0.742;
const float LIDAR_BACK_LEFT_POSITION_Sin=0.670;


//back right lidar;  
const float LIDAR_BACK_RIGHT_POSITION_X=67.2;
const float LIDAR_BACK_RIGHT_POSITION_Y=-486.6;
const float LIDAR_BACK_RIGHT_POSITION_COS=0.613;
const float LIDAR_BACK_RIGHT_POSITION_Sin=0.793;

//	雷达扫描在显示中的范围变量，由于图像显示是在-1到1之间;
//	所以需要进行一定的转换，显示范围为-5000cm到5000cm，需要除以比例变量;
const float LIDAR_DISPLAY_RANGE=1000;


//	雷达扫描点数据类型，包含扫描得到的X,Y坐标;
//	使用get，set方法对成员变量进行赋值或取值;
class LidarData
{
public:
	void SetScanX(int x){this->LidarScanX=x;}
	void SetScanY(int y){this->LidarScanY=y;}
	float GetScanX(){return LidarScanX;}
	float GetScanY(){return LidarScanY;}
protected:

private:
	float LidarScanX;
	float LidarScanY;

};

//	雷达目标数据类型，包含目标，长length,宽width,坐标x,y，速度v;
//	使用get，set方法对成员变量进行赋值或取值;
class LidarTargetData
{
public:
	void SetTargetX(int x){this->TargetPostionX=x;}
	void SetTargetY(int y){this->TargetPostionY=y;}
	void SetTargetLength(int Length){this->TargetLength=Length;}
	void SetTargetWidth(int Width){this->TargetWidth=Width;}
	void SetTargetVelocity(int Velocity){this->TargetVelocity=Velocity;}


	int GetTargetX(){return TargetPostionX;}
	int GetTargetY(){return TargetPostionY;}
	int GetTargetLength(){return TargetLength;}
	int GetTargetWidth(){return TargetWidth;}
	int GetTargetVelocity(){return TargetVelocity;}

protected:

private:
	int TargetPostionX;
	int TargetPostionY;
	int TargetLength;
	int TargetWidth;
	int TargetVelocity;

};

//聚类点数据类型;
class BreakPointData
{
public:
	void SetObjectId(int x){this->ObjectId=x;}
	void SetStartPosition(int x){this->StartPosition=x;}
	void SetEndPosition(int x){this->EndPosition=x;}

	int GetObjectId(){return ObjectId;}
	int GetStartPosition(){return StartPosition;}
	int GetEndPosition(){return EndPosition;}
	int GetPointNum(){return EndPosition-StartPosition+1;}

private:
	int ObjectId;		//	breakpoint ID，
	int StartPosition;	//	breakpoint起点位置，指向容器相应数据
	int EndPosition;	//	breakpoint起点坐标
	int PointNum;    //breakpoint包含的点数
};
