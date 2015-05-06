

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
	long ScanX;long ScanY;

	size_t data_n = data.size();
	//�����һ��ɨ������list.;
	/*switch(flag)
	{
	case MidLeft:m_LidarScanMLeft.clear();break;
	case MidRight:m_LidarScanMRight.clear();break;
	case BackLeft:m_LidarScanBLeft.clear();break;
	case BackRight:m_LidarScanBRight.clear();break;
	}*/
		m_LidarScanData.clear();

	for (size_t i = 0; i < data_n; ++i) {
		long l = data[i];
		if ((l <= min_distance) || (l >= max_distance)) {
			continue;
		}

		double radian = i*0.25*M_PI/180;//����=�Ƕ�*Pi/180;�����״�ֱ���0.25,�Ƕ�=i*0.25;
		LidarData TempLidarData;
		//	�Ӽ�����תΪ�ѿ�������,x,y��λΪmm;
		long x = static_cast<long>(l * cos(radian));
		long y = static_cast<long>(l * sin(radian));
		//	��x,y��λ��mmתΪcm; 
		//std::cout<<"radian="<<radian<<" "<<"i="<<i<<endl;
		x=x/10;y=y/10;

		if(flag==MidLeft)  //****����ߵ��״�����ת�� *****//
		{
			ScanX=-y+LIDAR_MIDDLE_LEFT_POSITION_X;
			ScanY=x+LIDAR_MIDDLE_LEFT_POSITION_Y;
		}
		else if(flag==MidRight)//****���ұߵ��״�����ת�� *****//
		{
			ScanX=y+LIDAR_MIDDLE_RIGHT_POSITION_X;
			ScanY=-x+LIDAR_MIDDLE_RIGHT_POSITION_Y;
		}

		else if(flag==BackLeft)//****Back left lidar*****//
		{
			ScanX=LIDAR_BACK_LEFT_POSITION_RATE*-(x+y)+LIDAR_BACK_LEFT_POSITION_X;
			ScanY=LIDAR_BACK_LEFT_POSITION_RATE*(x-y)+LIDAR_BACK_LEFT_POSITION_Y;
		}

		else if (flag==BackRight)//****Back right lidar*****//
		{
			ScanX=LIDAR_BACK_RIGHT_POSITION_RATE*(-x+y)+LIDAR_BACK_RIGHT_POSITION_X;
			ScanY=LIDAR_BACK_RIGHT_POSITION_RATE*-(x+y)+LIDAR_BACK_RIGHT_POSITION_Y;
		}
	
		//	���ɼ�������set��������templidardata��;
		TempLidarData.SetScanX(ScanX);
		TempLidarData.SetScanY(ScanY);
		//std::cout<<"SetScanX="<<ScanX<<" "<<"SetScanY="<<ScanY<<endl;
		
		//����
		/*switch(flag)
		{
			case MidLeft:m_LidarScanMLeft.push_back(TempLidarData);break;
			case MidRight:m_LidarScanMRight.push_back(TempLidarData);break;
			case BackLeft:m_LidarScanBLeft.push_back(TempLidarData);break;
			case BackRight:m_LidarScanBRight.push_back(TempLidarData);break;
		}*/
		m_LidarScanData.push_back(TempLidarData);
	}
	cout<<"data.size=="<<data.size()<<endl;
	urg.stop_measurement();	
}

void SingleLineLidar::preProData( void )
{

}