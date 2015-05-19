//======================================================================
/*! \雷达融合程序
 *
 * 
 * \author Tz 
 * \date 2015.3.3
 *
 *一个融合全部雷达的程序
 * 
 *///-----------
#include "database.h"
#include "ibeo.h"
#include "lidar.h"
#include "DataProcessing.h"
#include <boost/thread/thread.hpp>  
#include <boost/thread/mutex.hpp>  
#include <boost/bind.hpp>  
#include <tchar.h>
#include <gl/freeglut.h> //使用freeglut;
//#include <gl/glut.h> //使用glut;
//****************************************//

SingleLineLidar lidar_BR;
SingleLineLidar lidar_BL;
SingleLineLidar lidar_ML;
SingleLineLidar lidar_MR;
IbeoData TempIbeoData;
IbeoLidar Fleft;
IbeoLidar FRight;
extern vector<BreakPointData> BreakPoints;
//****************************************//
//***所有单线激光雷达初始化程序**************//
void AllLidarInit()
{
	if (g_nOpenflag[FrontLeft]==1 )
	{
		Fleft.init(FrontLeft);
	}
	
	if (g_nOpenflag[FrontLeft]!=1 && g_nOpenflag[FrontRight]==1)
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

/************************************************************************/
/* 画图的程序                                                          */
/************************************************************************/
void myDisplay(void)  
{   
	glClear(GL_COLOR_BUFFER_BIT);  //	清除缓冲区，GL_COLOR_BUFFER_BIT:当前可写的颜色缓冲;
	glPointSize(1.0f);		//	设置画图像素大小;
	//绘制车辆位置
	glColor3f(1.0f, 0.0f, 0.0f);//设置为红色；
	glBegin(GL_LINE_LOOP);
	glVertex2f(0.019,-0.003);
	glVertex2f(0.019,-0.097);
	glVertex2f(-0.019,-0.097);
	glVertex2f(-0.019,-0.003);
	glEnd();

	glColor3f(0.5f, 0.5f, 0.5f);//设置为灰色；
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

	if (g_nOpenflag[BackRight]==1)	//	如果后左雷达打开，开始绘图;
	{
		for (ListPointer=lidar_BR.m_LidarScanData.begin();ListPointer!=lidar_BR.m_LidarScanData.end();++ListPointer)
		{
			float drawX=ListPointer->GetScanX();
			float drawY=ListPointer->GetScanY();
			glVertex2f(drawX/LIDAR_DISPLAY_RANGE,drawY/LIDAR_DISPLAY_RANGE);
		}
	}
	glEnd();

	glColor3f(1.0f, 0.0f, 0.0f);//将聚类首位连接起来；
	for (int i=0;i<BreakPoints.size();++i)
	{
		glBegin(GL_LINES);
		
		int StartPostion=BreakPoints.at(i).GetStartPosition();
		int EndPostion=BreakPoints.at(i).GetEndPosition();
		float StartX=TempIbeoData.m_LidarScanFRight.at(StartPostion).GetScanX()/LIDAR_DISPLAY_RANGE;	
		float StartY=TempIbeoData.m_LidarScanFRight.at(StartPostion).GetScanY()/LIDAR_DISPLAY_RANGE;	
		float EndX=TempIbeoData.m_LidarScanFRight.at(EndPostion).GetScanX()/LIDAR_DISPLAY_RANGE;	
		float EndY=TempIbeoData.m_LidarScanFRight.at(EndPostion).GetScanY()/LIDAR_DISPLAY_RANGE;	
		
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
		ClusterLidar(TempIbeoData.m_LidarScanFRight);
		IepfAlgorithm(TempIbeoData.m_LidarScanFRight);
		LeastSquareMethod(BreakPoints,TempIbeoData.m_LidarScanFRight);


		//***************************************//

		glutPostRedisplay();
		t_end = clock();/* get end time */ 
		printf("time: %.3f s\n", (double)(t_end-t_start)/CLOCKS_PER_SEC); 	/*printf test time */ 
	}
	return 0;
}
	

extern int g_nLidarFlag=0;
extern int g_nMapUpdateTime=0;


int main(int argc, char *argv[])
{

	///*    雷初始化   */
	AllLidarInit();
	

	////****设置全局采集次数****/
	g_nMapUpdateTime=1;
	int time=0;

	
	while(g_nMapUpdateTime--)
	{
		//TODO::增加循环功能;
	}

	boost::thread_group grp;
	grp.create_thread(boost::bind(draw_main,argc,argv));
	
	
	/*while (	1)
	{
		cout<<"this is main program"<<time<<endl;
	}*/
	grp.join_all();

	cout<<"time=over"<<time<<endl;
	//t_end = clock();/* get end time */ 
	//printf("time: %.3f s\n", (double)(t_end-t_start)/CLOCKS_PER_SEC); 	/*printf test time */ 

	return 0;
}
