

/************************************************************************/
	/* 雷达运行头文件;
	   
		
	*/
	/************************************************************************/



#include "database.h"
#include "ibeo.h"
#include "lidar.h"

//声明实例
extern SingleLineLidar lidar_BR;
extern SingleLineLidar lidar_BL;
extern SingleLineLidar lidar_ML;
extern SingleLineLidar lidar_MR;
extern IbeoData TempIbeoData;
extern IbeoLidar Fleft;
extern IbeoLidar FRight;


//声明实例
void AllLidarInit();	//	所有雷达初始化程序;
void IbeoThreadFLeft();		//	前左Ibeo雷达;
void IbeoThreadFRight();	//	前右Ibeo雷达;
void runLidarML();
void runLidarMR();
void runLidarBL();
void runLidarBR();
