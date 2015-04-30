


#include "init.h"
#include "Urg_driver.h"
#include "math_utilities.h"
#include <iostream>
#include <string>
#include <list>
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"setupapi")

using namespace qrk;
using namespace std;


class SingleLineLidar
{
public:
	bool open(int flag);
	void beginScan(int flag);
	void preProData(void);

	Urg_driver urg;
	list<LidarData> m_LidarScanMLeft;
	list<LidarData> m_LidarScanMRight;
	list<LidarData> m_LidarScanBLeft;
	list<LidarData> m_LidarScanBRight;

protected:
private:
		
};


//;

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

	size_t data_n = data.size();
	cout<<"data.size=="<<g_nLidarFlag<<"   "<<data.size()<<endl;
	//清空上一次扫描数据list.;
	switch(flag)
	{
	case MidLeft:m_LidarScanMLeft.clear();break;
	case MidRight:m_LidarScanMRight.clear();break;
	case BackLeft:m_LidarScanBLeft.clear();break;
	case BackRight:m_LidarScanBRight.clear();break;
	}
		
	for (size_t i = 0; i < data_n; ++i) {
		long l = data[i];
		if ((l <= min_distance) || (l >= max_distance)) {
			continue;
		}

		double radian = urg.index2rad(i);
		LidarData TempLidarData;

		long x = static_cast<long>(l * cos(radian));
		long y = static_cast<long>(l * sin(radian));

		if(flag==MidLeft)  //****Middle left lidar *****//
		{
			x=-y+LIDAR_MIDDLE_LEFT_POSITION_X;
			y=x+LIDAR_MIDDLE_LEFT_POSITION_Y;
		}
		else if(flag==MidRight)//****Middle left lidar *****//
		{
			x=y+LIDAR_MIDDLE_RIGHT_POSITION_X;
			y=-x+LIDAR_MIDDLE_RIGHT_POSITION_Y;
		}

		else if(flag==BackLeft)//****Back left lidar*****//
		{
			x=LIDAR_BACK_LEFT_POSITION_RATE*(-x+y)+LIDAR_BACK_LEFT_POSITION_X;
			y=LIDAR_BACK_LEFT_POSITION_RATE*-(x+y)+LIDAR_BACK_LEFT_POSITION_Y;
		}

		else if (flag==BackRight)//****Back right lidar*****//
		{
			x=LIDAR_BACK_RIGHT_POSITION_RATE*(x-y)+LIDAR_BACK_RIGHT_POSITION_X;
			y=LIDAR_BACK_RIGHT_POSITION_RATE*-(x+y)+LIDAR_BACK_RIGHT_POSITION_Y;
		}
		//std::cerr<<"SetScanX="<<x<<" "<<"SetScanY="<<y<<endl;
		TempLidarData.SetScanX(x);
		TempLidarData.SetScanY(y);

		
		switch(flag)
		{
			case MidLeft:m_LidarScanMLeft.push_back(TempLidarData);break;
			case MidRight:m_LidarScanMRight.push_back(TempLidarData);break;
			case BackLeft:m_LidarScanBLeft.push_back(TempLidarData);break;
			case BackRight:m_LidarScanBRight.push_back(TempLidarData);break;
		}
	}
	urg.stop_measurement();	
}

void SingleLineLidar::preProData()
{
	//add the program;过滤异常点，去除范围外的点；
}





