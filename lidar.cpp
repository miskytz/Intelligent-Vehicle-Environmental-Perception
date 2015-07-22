

#include"lidar.h"
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"setupapi")
#include "math_utilities.h"
#include <iostream>
#include <string>
#include <list>
#include "math_utilities.h"
#include <iostream>
#include <string>

using namespace qrk;
using namespace std;



bool SingleLineLidar::open(int flag)
{
	string lidar_ip=ip[flag];
	if (!SingleLineLidar::urg.open(lidar_ip.c_str(),10940, Urg_driver::Ethernet)) 
	{
		std::cout << "Urg_driver::open(): "
			<< lidar_ip << ": " << urg.what() << endl;
		return 1;
	}
	SingleLineLidar::urg.set_scanning_parameter(urg.deg2step(-90), urg.deg2step(+90), 0);
}

void SingleLineLidar::beginScan(int flag)
{
	SingleLineLidar::urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
	vector<long> data;
	long time_stamp = 0;
	SingleLineLidar::urg.get_distance(data,&time_stamp);
	long min_distance = urg.min_distance();	
	long max_distance = urg.max_distance();
	float ScanX=0;float ScanY=0;

	size_t data_n = data.size();
	//清空上一次扫描数据list.;
	m_LidarScanData.clear();

	for (size_t i = 0; i < data_n; ++i) {
		long l = data[i];
		if ((l <= min_distance) || (l >= max_distance)) {
			continue;
		}

		double radian = i*0.25*M_PI/180;//弧度=角度*Pi/180;单线雷达分辨率0.25,角度=i*0.25;
		LidarData TempLidarData;
		//	从极坐标转为笛卡尔坐标,x,y单位为mm;
		long x = static_cast<long>(l * cos(radian));
		long y = static_cast<long>(l * sin(radian));
		//	将x,y单位从mm转为cm; 
		//std::cout<<"radian="<<radian<<" "<<"i="<<i<<endl;
		x=x/10;y=y/10;

		if(flag==MidLeft)  //****中左边的雷达坐标转换 *****//
		{
			ScanX=cos(LIDAR_MIDDLE_LEFT_ANGLE*M_PI/180)*x-sin(LIDAR_MIDDLE_LEFT_ANGLE*M_PI/180)*y+LIDAR_MIDDLE_LEFT_POSITION_X;
			ScanY=sin(LIDAR_MIDDLE_LEFT_ANGLE*M_PI/180)*x+cos(LIDAR_MIDDLE_LEFT_ANGLE*M_PI/180)*y+LIDAR_MIDDLE_LEFT_POSITION_Y;
		}
		else if(flag==MidRight)//****中右边的雷达坐标转换 *****//
		{
			ScanX=cos(LIDAR_MIDDLE_RIGHT_ANGLE*M_PI/180)*x-sin(LIDAR_MIDDLE_RIGHT_ANGLE*M_PI/180)*y+LIDAR_MIDDLE_RIGHT_POSITION_X;
			ScanY=sin(LIDAR_MIDDLE_RIGHT_ANGLE*M_PI/180)*x+cos(LIDAR_MIDDLE_RIGHT_ANGLE*M_PI/180)*y+LIDAR_MIDDLE_RIGHT_POSITION_Y;
		}

		else if(flag==BackLeft)//****Back left lidar*****//
		{
			ScanX=cos(LIDAR_BACK_LEFT_ANGLE*M_PI/180)*x-sin(LIDAR_BACK_LEFT_ANGLE*M_PI/180)*y+LIDAR_BACK_LEFT_POSITION_X;
			ScanY=sin(LIDAR_BACK_LEFT_ANGLE*M_PI/180)*x+cos(LIDAR_BACK_LEFT_ANGLE*M_PI/180)*y+LIDAR_BACK_LEFT_POSITION_Y;
		}

		else if (flag==BackRight)//****Back right lidar*****//
		{
			ScanX=cos(LIDAR_BACK_RIGHT_ANGLE*M_PI/180)*x-sin(LIDAR_BACK_RIGHT_ANGLE*M_PI/180)*y+LIDAR_BACK_RIGHT_POSITION_X;
			ScanY=sin(LIDAR_BACK_RIGHT_ANGLE*M_PI/180)*x+cos(LIDAR_BACK_RIGHT_ANGLE*M_PI/180)*y+LIDAR_BACK_RIGHT_POSITION_Y;
		}
	
		//	将采集数据用set方法存入templidardata中;
		TempLidarData.SetScanX(ScanX);
		TempLidarData.SetScanY(ScanY);
		//std::cout<<"SetScanX="<<ScanX<<" "<<"SetScanY="<<ScanY<<endl;
		
		m_LidarScanData.push_back(TempLidarData);
	}
	urg.stop_measurement();	
}

void SingleLineLidar::preProData( void )
{

}
