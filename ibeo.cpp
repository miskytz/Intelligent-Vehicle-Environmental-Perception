
/*
 * Copyright 2015 Intelligent Vehicle Lab of CQUPT.
 * Author :Tian Zhen   
 * Date:2015.5.4 
 *	
 * 这是一个Ibeo LUX系列四线激光雷达的数据采集源文件
 *    
 *
 */

#include "ibeo.h"
#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>
#include <windows.h>
#include <process.h>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <fstream>

using namespace ibeo;
using namespace std;

//======================================================================
class AllLuxListener : public ibeo::DataListener<ScanLux>,
                       public ibeo::DataListener<ObjectListLux>{
public:
	
	AllLuxListener(int f){flag = f;};

	//========================================
	void onData(const ScanLux* const scan)
	{
		const std::vector<ScanPointLux>& points = scan->getScanPoints();
		LidarData TempLidarData;
		float Transform_X=0;float Transform_Y=0;
		switch(flag)
		{
			case FrontLeft:TempIbeoData.m_LidarScanFLeft.clear(); break;
			case FrontRight:TempIbeoData.m_LidarScanFRight.clear(); break;
		}

		for (unsigned int i = 0; i < points.size(); i++)  
		{
			if (points.at(i).getLayer()==1 )
			{   
				ibeo::UINT16  Distance=points.at(i).getDistance();   //单位cm'
		
				float Angle=points.at(i).getHorizontalAngle();    //角度为与y轴夹角，从+50到-50；
					Angle=Angle/32;
				//C++中cos,sin,asin,acos这些三角函数操作的是弧度,而非角度;弧度=角度*Pi/180;
					//ibeo的坐标与直角坐标不同，需要转换  X=-gety=-dsinr;y=getx=dcosr;
				float x = static_cast<long>(Distance * -sin(Angle*M_PI/180));//
				float y = static_cast<long>(Distance * cos(Angle*M_PI/180));
			
				//cout<<"Distance="<<Distance<<"   "<<" Angle="<< Angle<<endl;
				//32是角度系数，参见ibeo说明文档，central range 0.125'
				//*medium range 0.25',lateral range 0.5'*	
				
				//*****存储数据;*****/
				if(flag==FrontLeft)//****Fron left lidar*****//
				{
					Transform_X=cos(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*x-sin(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*y+LIDAR_FRONT_LEFT_POSITION_X;
					Transform_Y=sin(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*x+cos(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*y+LIDAR_FRONT_LEFT_POSITION_Y;
				}

				else if (flag==FrontRight)//****Fron right lidar*****//
				{
					Transform_X=cos(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*x-sin(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*y+LIDAR_FRONT_RIGHT_POSITION_X;
					Transform_Y=sin(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*x+cos(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*y+LIDAR_FRONT_RIGHT_POSITION_Y;
				}
			
			
				//cerr<<"SetScanX="<<x<<" "<<"SetScanY="<<y<<endl;
				TempLidarData.SetScanX(Transform_X);
				TempLidarData.SetScanY(Transform_Y);

			
				switch(flag)
				{
					case FrontLeft:TempIbeoData.m_LidarScanFLeft.push_back(TempLidarData); break;
					case FrontRight:TempIbeoData.m_LidarScanFRight.push_back(TempLidarData); break;
				}

			}		
			
		}
	}			
		


	//========================================
	void onData(const ObjectListLux* const pObj)
	{
	
		const std::vector<ObjectLux>& objects= pObj->getObjects();
		LidarTargetData TempTargerData;
		switch(flag)
		{
		case FrontLeft:TempIbeoData.m_LidarObjectFLeft.clear(); break;
		case FrontRight:TempIbeoData.m_LidarObjectFRight.clear(); break;
		}

		float Transform_Ob_X=0;float Transform_Ob_Y=0;float ObjectX,ObjectY;
		for (unsigned int i = 0; i < objects.size(); ++i) {
			
			//*****存储数据;*****/
			
			//坐标信息
			ObjectX=-objects.at(i).getBoundingBoxCenter().getY();
			ObjectY=objects.at(i).getBoundingBoxCenter().getX();

							
			if(flag==FrontLeft)//****Fron left lidar*****//
			{
				Transform_Ob_X=cos(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*ObjectX-sin(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*ObjectY+LIDAR_FRONT_LEFT_POSITION_X;
				Transform_Ob_Y=sin(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*ObjectX+cos(LIDAR_FRONT_LEFT_ANGLE*M_PI/180)*ObjectY+LIDAR_FRONT_LEFT_POSITION_Y;
			}

			else if (flag==FrontRight)//****Fron right lidar*****//
			{
				Transform_Ob_X=cos(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*ObjectX-sin(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*ObjectY+LIDAR_FRONT_RIGHT_POSITION_X;
				Transform_Ob_Y=sin(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*ObjectX+cos(LIDAR_FRONT_RIGHT_ANGLE*M_PI/180)*ObjectY+LIDAR_FRONT_RIGHT_POSITION_Y;
			}
			
			int VelocitySigmaX=objects.at(i).getAbsoluteVelocitySigmaX();
			int VelocitySigmaY=objects.at(i).getAbsoluteVelocitySigmaY();

			TempTargerData.SetTargetX(Transform_Ob_X);
			TempTargerData.SetTargetY(Transform_Ob_Y);
			TempTargerData.SetTargetLength(objects.at(i).getBoundingBoxLength());
			TempTargerData.SetTargetWidth(objects.at(i).getBoundingBoxWidth());
			TempTargerData.SetTargetVelocity(0.707*VelocitySigmaX+0.707*VelocitySigmaY);
			TempTargerData.SetTargetClass(objects.at(i).getClassification());

			
			
			switch(flag)
			{
				case FrontLeft:TempIbeoData.m_LidarObjectFLeft.push_back(TempTargerData);break;
				case FrontRight:TempIbeoData.m_LidarObjectFRight.push_back(TempTargerData);break;
			}
		}

	}

private:
	int flag;  //雷达编号标志;
	//========================================;
};



void IbeoLidar::init(int flag)
{
	const char *argv[]={"D:\\"};//输出地址
	const off_t maxLogFileSize = 1000000;
	logfile.setTargetFileSize(maxLogFileSize);
	logfile.setLogFileBaseName(argv[0]);
	logFileManager.start();
	logInfo << argv[0] << "/n" 
		<< " IP：" << ip[flag] << std::endl;
	
}

bool IbeoLidar::isLidarWork()
{
	return 0;

}

void IbeoLidar::beginScan(int flag)
{
	AllLuxListener allLuxListener(flag);
	IbeoLux lux(ip[flag],12002);
	//注意，开始工工作时必须要传入LogFileManager类对象，否则会报错;
	lux.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allLuxListener));
	lux.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allLuxListener));
	lux.getConnected();	
	_sleep(300);
	
}