


#include "database.h"
#include "DataProcessing.h"
#include <list>
#include <iostream>
#include<math.h>
#include <vector>
#include <algorithm> 
//	根据栅格地图范围，去掉范围外的点;
int DataClear(vector<LidarData> &data)
{
	

	vector<LidarData>::iterator listpointer;
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


float TurnToRange(vector<LidarData> &data,int n)
{
	float ScanX=data.at(n).GetScanX();
	float ScanY=data.at(n).GetScanY();
	float iScanRange=sqrt(ScanX*ScanX+ScanY*ScanY);
	return(iScanRange);
}



////	中值滤波;
//中值滤波有问题，Y轴竖直方向的数据没见了；
//int GetMedianNum(list<LidarData> &data)  
//{  
//	vector<LidarData> Vectordata;
//	list<LidarData>::iterator ListPointer;
//	ListPointer=data.begin();
//
//	Vectordata.assign(data.begin(),data.end());//将数据转化为vector类型；
//	int iVectorN=0;  //用于取出容器值；
//
//	const int iFilterLen=10;   //中指滤波的阈值,取奇数；
//	
//
//	while (iVectorN<=Vectordata.size()-iFilterLen)//减去iFilterLen是因为，如果最后一堆数没有iFilterLen个，就不排序
//	{
//		//冒泡排序
//		for (int i=iVectorN;i<iVectorN+iFilterLen-1;++i)
//		{
//			for (int j=iVectorN;j<iVectorN+iFilterLen-i-1;++j)
//			{
//				//if (TurnToRange(Vectordata,j)>TurnToRange(Vectordata,j+1))
//				if (Vectordata.at(j).GetScanY()>Vectordata.at(j+1).GetScanY())
//	
//				{
//					std::iter_swap (&Vectordata[j],&Vectordata[j+1]);
//				}
//			}
//		}
//		//取中指；
//		for (int i=0;i<iFilterLen;++i)
//		{
//			
//			ListPointer->SetScanY(Vectordata.at((iVectorN+iFilterLen)/2).GetScanY());
//			++ListPointer;
//			++iVectorN;
//		}
//
//		
//	}
//
//	return 0;
//}
	
vector<BreakPointData> BreakPoints;


//基于距离的聚类算法
int ClusterLidar(vector<LidarData> &Vectordata)
{
	float Resolution=0.25;   //转化为幅度M_PI/180
	float RLimit=2;
	float NoiseR=10; //噪声设定，根据IBEO说明文档
	BreakPointData TempPoints;

	BreakPoints.clear();
	//聚类算法，出自Line Extraction in 2D Range Images for Mobile Robotics;
	for (int i=1;i<Vectordata.size();++i)
	{
		float Rn_1=TurnToRange(Vectordata,i-1);
		
		float Dmax=Rn_1*(sin(Resolution*3.14/180)/sin((RLimit-Resolution)*3.14/180))+3*NoiseR;
		
		float DinX=abs(Vectordata.at(i).GetScanX()-Vectordata.at(i-1).GetScanX());
		float DinY=abs(Vectordata.at(i).GetScanY()-Vectordata.at(i-1).GetScanY());
		float Din=sqrt(DinX*DinX+DinY*DinY);
		
	
		if (i==1) //才开始，将x0,y0作为起始坐标点
		{
						
			TempPoints.SetStartPosition(0);	
		}
		
		if (Din>Dmax)
		{
			TempPoints.SetEndPosition(i-1);

			if(TempPoints.GetPointNum()>4)   //端点的条件是必须要包含4个点；
			{
				BreakPoints.push_back(TempPoints);//存储一个聚类起点终点;
			}

			
			//下一个X,Y作为起始点;
			TempPoints.SetStartPosition(i);
		}
		
		if (i==(Vectordata.size()-1)) //结束
		{
			TempPoints.SetEndPosition(Vectordata.size()-1);
			BreakPoints.push_back(TempPoints);//存储一个聚类起点终点;
		}
	}
	
	cout<<"breakpoint"<<BreakPoints.size()<<endl;

	return 1;
}

