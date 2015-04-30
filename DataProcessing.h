


#include "init.h"
#include <iostream>


using namespace std;




int DataClear(list<LidarData> &data)
{
	/************************************************************************/
	/* 根据系统设计的要求， 辆前方长 80m，宽 20m;
	    车辆左侧、右侧、后方 10m 建立网格地图;
		清除范围外的点;
	*/
	/************************************************************************/
	const int s_nGridFrontLenth=8000; //网格前方长度单位8000cm;
	const int s_nGridBackLenth=1000;
	const int s_nGridWidth=1000;
	const int s_nCarLenth=1000;
	const int s_nCarWidth=1000;

	list<LidarData>::iterator i;
	for (i=data.begin();i!=data.end();++i)
	{
		if ((abs(i->GetScanX())) >(s_nGridWidth+s_nCarWidth ))
		{
			data.erase(i);
		}
		
		if (i->GetScanY()>s_nGridFrontLenth || i->GetScanY()<-(s_nCarLenth+s_nGridBackLenth))
		{
			data.erase(i);
		}
	}
	return 0;
}