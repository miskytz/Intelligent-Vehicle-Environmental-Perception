

#include <string>
#include <iostream>
#pragma once

using namespace std;
/*  基本数据类型 */

//全局变量;

//定义，比如 const int a = 10;


extern string ip[6]={"192.168.1.30","192.168.1.31","192.168.1.20","192.168.1.21","192.168.1.22","192.168.1.23"};
extern int g_nLidarFlag=0;
extern int g_nMapUpdateTime=0;
enum LidarName{  FrontLeft=0,FrontRight=1, MidLeft=2, MidRight=3,BackLeft=4,BackRight=5};
extern int g_nOpenflag[6]={1,1,1,1,1,1};    //雷达是否打开标志，1为打开，0位关闭，可以从初始文件中读取；



/*宏定义*/ 
/*front left lidar,ibeo*/ 
const int LIDAR_FRONT_LEFT_POSITION_X=-68.2;
const int LIDAR_FRONT_LEFT_POSITION_Y=-1.5;
const float LIDAR_FRONT_LEFT_POSITION_RATE=1;

/*front right lidar,ibeo*/ 
const int LIDAR_FRONT_RIGHT_POSITION_X=68.2;
const int LIDAR_FRONT_RIGHT_POSITION_Y=-1.5;
const float LIDAR_FRONT_RIGHT_POSITION_RATE=1;

/*Middle left lidar*/ 
const int LIDAR_MIDDLE_LEFT_POSITION_X=-99.1;
const int LIDAR_MIDDLE_LEFT_POSITION_Y=-134.5;
const float LIDAR_MIDDLE_LEFT_POSITION_RATE=1;

/*Middle right lidar*/ 
const int LIDAR_MIDDLE_RIGHT_POSITION_X=99.1;
const int LIDAR_MIDDLE_RIGHT_POSITION_Y=-134.5;
const float LIDAR_MIDDLE_RIGHT_POSITION_RATE=1;

/*back left lidar*/ 
const int LIDAR_BACK_LEFT_POSITION_X=-67.2;
const int LIDAR_BACK_LEFT_POSITION_Y=-486.6;
const float LIDAR_BACK_LEFT_POSITION_RATE=0.707;

/*back right lidar*/ 
const int LIDAR_BACK_RIGHT_POSITION_X=67.2;
const int LIDAR_BACK_RIGHT_POSITION_Y=-486.6;
const float LIDAR_BACK_RIGHT_POSITION_RATE=0.707;



class LidarData
{
public:
	void SetScanX(int x){this->LidarScanX=x;}
	void SetScanY(int y){this->LidarScanY=y;}
	int GetScanX(){return LidarScanX;}
	int GetScanY(){return LidarScanY;}
protected:

private:
	int LidarScanX;
	int LidarScanY;

};

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

