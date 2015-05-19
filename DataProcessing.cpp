


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
	for (int i=1;i<Vectordata.size();i++)
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

			if(TempPoints.GetPointNum()>5)   //断点的条件是必须要包含5个点；
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
	
	//聚类目标编号
	for (int i=0;i<BreakPoints.size();i++)
	{
		BreakPoints.at(i).SetObjectId(i);
	}

	return 1;
}


//求两个点之间的距离;
float GetPointDistance(LidarData p1, LidarData p2)   
{  
	float PointDistance=(p1.GetScanX()-p2.GetScanX())*(p1.GetScanX()-p2.GetScanX())+
		(p1.GetScanY()-p2.GetScanY())*(p1.GetScanY()-p2.GetScanY());
	PointDistance=sqrt((float)PointDistance);

	return PointDistance;  
} 

//海伦公式求取点到直线的距离;
float GetNearestDistance(LidarData PA, LidarData PB, LidarData PX)  
{  
	//PA为起点，PB为终点;PX为选择的点
	//----------图2--------------------  
	float a,b,c;  
	a=GetPointDistance(PB,PX);  
	if(a<=0.00001)  
		return 0.0f;  
	b=GetPointDistance(PA,PX);  
	if(b<=0.00001)  
		return 0.0f;  
	c=GetPointDistance(PA,PB);  
	if(c<=0.00001)  
		return a;//如果PA和PB坐标相同，则退出函数，并返回距离  
	//------------------------------  

	if(a*a>=b*b+c*c)//--------图3--------  
		return b;      //如果是钝角返回b  
	if(b*b>=a*a+c*c)//--------图4-------  
		return a;      //如果是钝角返回a  

	//图1  
	float l=(a+b+c)/2;     //周长的一半  
	float s=sqrt(l*(l-a)*(l-b)*(l-c));  //海伦公式求面积，也可以用矢量求  
	return 2*s/c;  
}  






void IepfAlgorithm(vector<LidarData> &data)
{
	BreakPointData TempPoints;

	for (int i=0;i<BreakPoints.size();i++)
	{
		const int IEPFThreshold=20;//iepf算法边界；
		int StartPostion=BreakPoints.at(i).GetStartPosition();
		int EndPostion=BreakPoints.at(i).GetEndPosition();

		for (int j=StartPostion+1;j<EndPostion;j++)
		{
			int NearestDistance=GetNearestDistance(data.at(StartPostion),data.at(EndPostion),data.at(j));
			if (NearestDistance>=IEPFThreshold)
			{
				BreakPoints.at(i).SetEndPosition(j-1);//将当点作为结束;
				TempPoints.SetStartPosition(j-1);
				TempPoints.SetEndPosition(EndPostion);
				TempPoints.SetObjectId(i);
				BreakPoints.insert(BreakPoints.begin()+i+1,TempPoints);//插入一个点;
				break;
			}

		}
	}
	cout<<"breakpoint"<<BreakPoints.size()<<endl;
};

//最小二乘法拟合直线;
void LeastSquareMethod(vector<BreakPointData> &breakdata,vector<LidarData> &data)
{
	float a,b;	//定义最小而二乘法两个参数;
	

	for (int i=0;i<breakdata.size();i++)
	{
		int SizeN=breakdata.at(i).GetPointNum();
		int StartPostion=BreakPoints.at(i).GetStartPosition();
		float SumXY=0,SumX=0,SumY=0,SumXX=0;
		//SumXY 为plus（XiYi），SumX 为 plus（Xi），SumY为puls(Yi);
 		for (int j=0;j<SizeN;j++)
		{

			float Xj=data.at(j+StartPostion).GetScanX();
			float Yj=data.at(j+StartPostion).GetScanY();
			
			SumXY +=Xj*Yj;
			SumX +=Xj;
			SumY +=Yj;
			SumXX +=Xj*Xj;
		}
		a=(SumXX*SumY-SumX*SumXY)/(SizeN*SumXX-SumX*SumX);
		b=(SizeN*SumXY-SumX*SumY)/(SizeN*SumXX-SumX*SumX);
		

		//将断点起点Y转为最小二乘法的Y；
		float ScanX=data.at(breakdata.at(i).GetStartPosition()).GetScanX();//求得断点起始的X坐标
		float LeastSquareY=a+b*ScanX;   //求得其对应最小二乘法的Y
		data.at(breakdata.at(i).GetStartPosition()).SetScanY(LeastSquareY);	//存与Y

		//将断点终点Y转为最小二乘法的Y；
		ScanX=data.at(breakdata.at(i).GetEndPosition()).GetScanX();//求得断点起始的X坐标
		LeastSquareY=a+b*ScanX;   //求得其对应最小二乘法的Y
		data.at(breakdata.at(i).GetEndPosition()).SetScanY(LeastSquareY);	//存与Y
	}
	
}