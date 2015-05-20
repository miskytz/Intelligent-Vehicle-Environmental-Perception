



#include "AllLidarRun.h";


//定义每个雷达类的实例
SingleLineLidar lidar_BR;
SingleLineLidar lidar_BL;
SingleLineLidar lidar_ML;
SingleLineLidar lidar_MR;
IbeoData TempIbeoData;
IbeoLidar Fleft;
IbeoLidar FRight;

//****************************************//
//***所有单线激光雷达初始化程序**************//
void AllLidarInit()
{
	if (g_nOpenflag[FrontLeft]==1 )
	{
		Fleft.init(FrontLeft);
	}

	if (g_nOpenflag[FrontLeft]!=1 && g_nOpenflag[FrontRight]==1) //四线激光雷达智能打开一次;
	{
		FRight.init(FrontRight);
	}
	//****中左雷达***********//
	if (g_nOpenflag[MidLeft]==1)
	{
		lidar_ML.open(MidLeft);
	}
	//****中右边雷达***********//
	if (g_nOpenflag[MidRight]==1)
	{
		lidar_MR.open(MidRight);
	}

	if (g_nOpenflag[BackLeft]==1)
	{
		lidar_BL.open(BackLeft);
	}

	if (g_nOpenflag[BackRight]==1)
	{
		lidar_BR.open(BackRight);
	}

}


//****************************************//
//************四线激光雷达运行程序**********//


void IbeoThreadFLeft()
{
	//**前面左边多线激光雷达**//
	Fleft.beginScan(FrontLeft);
}

void IbeoThreadFRight()
{
	//**前面右边多线激光雷达**//
	FRight.beginScan(FrontRight);
	//cout<<"i am working"<<endl;
}


//****************************************//
//************单线激光雷达运行程序**********//
void runLidarML()
{
	lidar_ML.beginScan(MidLeft);
	//cerr<<"MidLeft.size= " << lidar_ML.m_LidarScanData.size()<< std::endl;
}

void runLidarMR()
{
	lidar_MR.beginScan(MidRight);
	//	cerr<<"MidRight.size= " << lidar_MR.m_LidarScanData.size()<< std::endl;
}

void runLidarBL()
{
	lidar_BL.beginScan(BackLeft);
	//cerr<<"BackLeft.size= " << lidar_BL.m_LidarScanData.size()<< std::endl;
}

void runLidarBR()
{
	lidar_BR.beginScan(BackRight);
	//cerr<<"BackRight.size= " << lidar_BR.m_LidarScanData.size()<< std::endl;
}
