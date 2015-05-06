


#include "database.h"
#include "DataProcessing.h"
#include <list>
#include <iostream>



//	根据栅格地图范围，去掉范围外的点;
int DataClear(list<LidarData> &data)
{
	

	list<LidarData>::iterator listpointer;
	for (listpointer=data.begin();listpointer!=data.end();++listpointer)
	{
		if ((abs(listpointer->GetScanX())) >(s_nGridWidth+s_nCarWidth ))
		{
			data.erase(listpointer);
		}
		
		if (listpointer->GetScanY()>s_nGridFrontLenth || listpointer->GetScanY()<-(s_nCarLenth+s_nGridBackLenth))
		{
			data.erase(listpointer);
		}
	}
	return 0;
}


////	中值滤波;
//int GetMedianNum(list<LidarData> &data)  
//{  
//	list<LidarData>::iterator listpointer;
//	listpointer
//	
//	int i,j;// 循环变量  
//
//	unsigned char bTemp;  
//
//	// 用冒泡法对数组进行排序  
//	for (j = 0; j < iFilterLen - 1; j ++)  
//	{  
//		for (i = 0; i < iFilterLen - j - 1; i ++)  
//		{  
//			if (bArray[i] > bArray[i + 1])  
//			{  
//				// 互换  
//				bTemp = bArray[i];  
//				bArray[i] = bArray[i + 1];  
//				bArray[i + 1] = bTemp;  
//			}  
//		}  
//	}  
//
//	// 计算中值  
//	if ((iFilterLen & 1) > 0)  
//	{  
//		// 数组有奇数个元素，返回中间一个元素  
//		bTemp = bArray[(iFilterLen + 1) / 2];  
//	}  
//	else  
//	{  
//		// 数组有偶数个元素，返回中间两个元素平均值  
//		bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
//	}  
//
//	return bTemp;  
//}  
