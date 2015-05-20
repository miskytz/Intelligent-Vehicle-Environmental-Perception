/************************************************************************/
/*  Copyright 2015 Intelligent Vehicle Lab of CQUPT.
/* Author :Tian Zhen   
/* 2015.5.19 
/*主程序
/************************************************************************/


#include "database.h"
#include "DataProcessing.h"
#include "AllLidarRun.h"
#include <boost/thread/thread.hpp>  
#include <boost/thread/mutex.hpp>  
#include <boost/bind.hpp>  
#include <tchar.h>
#include <gl/freeglut.h> //使用freeglut;（可以不用一直等待消息循环）
//#include <gl/glut.h> //使用glut;


//	声明或定义全局变量;

extern int g_nLidarFlag=0;
extern vector<BreakPointData> BreakPoints; //聚类目标断点的目标;
extern int g_nMapUpdateTime=1;
// 重调画图的程序，进行画图，以及显示;
void myDisplay(void)  
{   
	glClear(GL_COLOR_BUFFER_BIT);  //	清除缓冲区，GL_COLOR_BUFFER_BIT:当前可写的颜色缓冲;
	glPointSize(1.5f);		//	设置画图像素大小;
	
	//	绘制车辆位置;
	glColor3f(0.0f,0.0f, 1.0f);	//设置为蓝色;
	glBegin(GL_LINE_LOOP);
	glVertex2f(0.019,-0.003);
	glVertex2f(0.019,-0.097);
	glVertex2f(-0.019,-0.097);
	glVertex2f(-0.019,-0.003);
	glEnd();

	//将扫描点图绘制出来;
	glColor3f(1.0f, 1.0f, 1.0f);	//设置为灰色;
	vector<LidarData>::iterator ListPointer;	//	定义一个迭代器i，用于访问vector;
	glBegin( GL_POINTS);
	

	/* 在这里使用 glVertex*系列函数 ;*/
	if (g_nOpenflag[FrontLeft]==1)  //	如果左前雷达打开，开始绘图;
	{
		for (ListPointer=TempIbeoData.m_LidarScanFLeft.begin();ListPointer!=TempIbeoData.m_LidarScanFLeft.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE, drawY/LIDAR_DISPLAY_RANGE);
		}
	}
	
	if (g_nOpenflag[FrontRight]==1)	//	如果右前雷达打开，开始绘图;
	{
		for (ListPointer=TempIbeoData.m_LidarScanFRight.begin();ListPointer!=TempIbeoData.m_LidarScanFRight.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}
	
	if (g_nOpenflag[MidLeft]==1)	//	如果中左雷达打开，开始绘图;
	{
		for (ListPointer=lidar_ML.m_LidarScanData.begin();ListPointer!=lidar_ML.m_LidarScanData.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}

	if (g_nOpenflag[MidRight]==1)	//	如果中右雷达打开，开始绘图;
	{
		for (ListPointer=lidar_MR.m_LidarScanData.begin();ListPointer!=lidar_MR.m_LidarScanData.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}

	if (g_nOpenflag[BackLeft]==1)	//	如果后左雷达打开，开始绘图;
	{
		for (ListPointer=lidar_BL.m_LidarScanData.begin();ListPointer!=lidar_BL.m_LidarScanData.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}

	if (g_nOpenflag[BackRight]==1)	//	如果后右雷达打开，开始绘图;
	{
		for (ListPointer=lidar_BR.m_LidarScanData.begin();ListPointer!=lidar_BR.m_LidarScanData.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}
	glEnd();

	//绘制目标轮廓;
	glColor3f(1.0f, 0.0f, 0.0f);
	for (int i=0;i<BreakPoints.size();++i)
	{
		glBegin(GL_LINES);
		
		int StartPostion=BreakPoints.at(i).GetStartPosition();
		int EndPostion=BreakPoints.at(i).GetEndPosition();
		float StartX=TempIbeoData.m_LidarScanFLeft.at(StartPostion).GetScanX()/LIDAR_DISPLAY_RANGE;	
		float StartY=TempIbeoData.m_LidarScanFLeft.at(StartPostion).GetScanY()/LIDAR_DISPLAY_RANGE;	
		float EndX=TempIbeoData.m_LidarScanFLeft.at(EndPostion).GetScanX()/LIDAR_DISPLAY_RANGE;	
		float EndY=TempIbeoData.m_LidarScanFLeft.at(EndPostion).GetScanY()/LIDAR_DISPLAY_RANGE;	
		
		glVertex2f(StartX,StartY);
		glVertex2f(EndX,EndY);
		glEnd();
	}


	glFlush();
	glutSwapBuffers();
}  


int draw_main(int argc, char *argv[])
{
	glutInit(&argc, argv);  
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE );  // 修改了参数为 GLUT_DOUBLE;
	glutInitWindowPosition(100, 100);  
	glutInitWindowSize(500, 500);  
	glutCreateWindow("第一个OpenGL程序");  
	glutDisplayFunc(&myDisplay); 

	clock_t t_start; /* start time when test starts */ 
	clock_t t_end; /* end time when test ends */ 
	while(true)
	{
		t_start = clock(); /* get start time */ 
		glutMainLoopEvent();

		//	使用线程组，管理线程,采集雷达数据,并行采集;
		//	IBEO需要0.34s,激光雷达0.15s,完成采集0.4s;
		boost::thread_group grp;
		if (g_nOpenflag[FrontLeft]==1) {grp.create_thread(&IbeoThreadFLeft);}
		if (g_nOpenflag[FrontRight]==1) {grp.create_thread(&IbeoThreadFRight);}
		if (g_nOpenflag[MidLeft]==1)   {grp.create_thread(&runLidarML);}
		if (g_nOpenflag[MidRight]==1) {grp.create_thread(&runLidarMR);	}
		if (g_nOpenflag[BackLeft]==1) {grp.create_thread(&runLidarBL);	}
		if (g_nOpenflag[BackRight]==1) {grp.create_thread(&runLidarBR);}
		grp.join_all();

		//*********在此添加数据处理程序*********//
		//此消息循环会一直循环执行，直到程序结束//
		ClusterLidar(TempIbeoData.m_LidarScanFLeft);
		IepfAlgorithm(TempIbeoData.m_LidarScanFLeft);
		//LeastSquareMethod(BreakPoints,TempIbeoData.m_LidarScanFLeft);

		glutPostRedisplay();
		t_end = clock();/* get end time */ 
		printf("time: %.3f s\n", (double)(t_end-t_start)/CLOCKS_PER_SEC); 	/*printf test time */ 
	}
	return 0;
}
	





int main(int argc, char *argv[])
{
	
	AllLidarInit();	//	雷达chu1=-=
	

	////****设置全局采集次数****/
	while(g_nMapUpdateTime--)
	{
		//TODO::在采集器增加循环功能;
	}


	boost::thread_group grp;
	grp.create_thread(boost::bind(draw_main,argc,argv));
	grp.join_all();

	return 0;
}
