#include "stdafx.h"
#include "FirstPlan.h"
#include "Node.h"
extern CvPoint StartPoint;
extern IplImage * MoPlanMap;
extern IplImage * CostMap;
extern CvPoint FTargetPoint;
extern bool FirstPlanSuccess;

CFirstPlan::CFirstPlan(void)
{
	OpenTable = new CNode::FirstPlanNode[2000000];
	CloseTable = new CNode::FirstPlanNode[100000];
	FPathpoints = new CvPoint[1000000];
	white = CV_RGB(255,255,255);
	black = CV_RGB(0,0,0);
	blue = CV_RGB(0,0,255);
    	green = CV_RGB(0,255,0);
	red = CV_RGB(255,0,0);
	orange = CV_RGB(255,255,110);
}
IplImage* CFirstPlan::FirstPlan(IplImage* Image)
{
	OpenTableNum = 0;
	CloseTableNum = 0;
	FPathpointsNum = 0; 
	FNodeIDcounter = 0;

	FFatherNode.FChildNum = 0;
	FFatherNode.FFatherNodeID = 0;
	FFatherNode.FNodeID = 0;
	FFatherNode.FNodeCoord = StartPoint;
	FFatherNode.FFcostval = 0.0;
	FFatherNode.FGcostval = 0.0;
	FFatherNode.FHcostval = 0.0;
	//建立起点模型
	
	CloseTable[CloseTableNum++] = FFatherNode;
	double distance;
	for(int i=0;i<15000;i++)
	{
		distance = sqrt( (double)((FFatherNode.FNodeCoord.x-FTargetPoint.x)*(FFatherNode.FNodeCoord.x-FTargetPoint.x)+(FFatherNode.FNodeCoord.y-FTargetPoint.y)*(FFatherNode.FNodeCoord.y-FTargetPoint.y)) );
		if(distance<10)
		{
			FGeneratePath(CloseTable,FFatherNode,Image);
			FirstPlanSuccess = true;
			break;
		}
		else
			Gene_CldNode(FFatherNode);

		cvCircle(Image,FFatherNode.FNodeCoord,1,orange,1); 
		/*cvShowImage("PLANNING2",Image);
		cvWaitKey();*/
		if(i==14999)
			FirstPlanSuccess = false;
	}
	return(Image);
}

void CFirstPlan::FGeneratePath(CNode::FirstPlanNode* Nodes,CNode::FirstPlanNode EndNode, IplImage* ImageTem)
{
	FPathpointsNum = 0;
	CNode::FirstPlanNode CurNode;
	CurNode = EndNode;
	while( CurNode.FNodeID != Nodes[0].FNodeID )//没有找到起点，就一直寻找
	{
		FPathpoints[FPathpointsNum++] = CurNode.FNodeCoord;//第一个是终点，依次到起点
		for(int i=0;i<(CloseTableNum-1);i++)
		{
			if(CurNode.FFatherNodeID==Nodes[i].FNodeID)//回溯找父节点
			{
				CurNode = Nodes[i];
				break;
			}
		}
	}
	FPathpoints[FPathpointsNum++] = CurNode.FNodeCoord;
	for(int i=1;i<FPathpointsNum;i++)
	{
		cvLine(ImageTem,FPathpoints[i], FPathpoints[i-1], orange, 1);
		/*printf("路径点： %d  %d\n",FPathpoints[i].x,FPathpoints[i].y);*/	
	}
	FPointNum = FPathpointsNum /30;
}

void CFirstPlan::Gene_CldNode(CNode::FirstPlanNode Node)
{
	bool NewNodeJudge;
	double AngleError;
	CNode::FirstPlanNode NewNode;

	if(FTargetPoint.x>Node.FNodeCoord.x)
	{
		Angle = -180.0*atan((double)(FTargetPoint.y-Node.FNodeCoord.y)/(double)(FTargetPoint.x-Node.FNodeCoord.x))/M_PI;
		if(Angle<0.0)
			Angle = 360.0 + Angle ;
	}
	else if(FTargetPoint.x<Node.FNodeCoord.x)
	{
		Angle = 180.0 - 180.0*atan((double)(FTargetPoint.y-Node.FNodeCoord.y)/(double)(FTargetPoint.x-Node.FNodeCoord.x))/M_PI;
	}
	else if(FTargetPoint.y>Node.FNodeCoord.y)
		Angle = 270.0;
	else 
		Angle = 90.0;

	for(int n=0;n<8;n++)
	{
		NewNode.FChildNum = n+1;
		switch(n)
		{
		case 0:
			NewNode.FNodeCoord.x = Node.FNodeCoord.x+3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				AngleError = abs(Angle);
				if(AngleError>180.0)
					AngleError=360.0-AngleError;
				NewNode.FGcostval = 3.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x) )+ AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 1:
			AngleError = abs(Angle-45.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x+3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y-3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 4.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x) )+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 2:
			AngleError = abs(Angle-90.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y-3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 3.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x) )+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 3:
			AngleError = abs(Angle-135.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x-3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y-3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 4.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x) )+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 4:
			AngleError = abs(Angle-180.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x-3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 3.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * (  abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x) )+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 5:
			AngleError = abs(Angle-225.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x-3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y+3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 4.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x))+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 6:
			AngleError = abs(Angle-270.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y+3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 3.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x))+AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		case 7:
			AngleError = abs(Angle-315.0);
			if(AngleError>180.0)
					AngleError=360.0-AngleError;
			NewNode.FNodeCoord.x = Node.FNodeCoord.x+3;
			NewNode.FNodeCoord.y = Node.FNodeCoord.y+3;
			NewNodeJudge = NodeJudge(NewNode.FNodeCoord);
			if(NewNodeJudge)
			{
				NewNode.FGcostval= 4.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 5.0 * ( abs(FTargetPoint.y-NewNode.FNodeCoord.y)+abs(FTargetPoint.x-NewNode.FNodeCoord.x))  +AngleError/3.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			else
			{
				NewNode.FGcostval = 1000.0 + FFatherNode.FGcostval;
				NewNode.FHcostval = 10000.0;
				NewNode.FFcostval = NewNode.FGcostval + NewNode.FHcostval;
			}
			break;
		}
		if(NewNodeJudge)
		{
			FNodeIDcounter++;
			NewNode.FNodeID = FNodeIDcounter;
			NewNode.FFatherNodeID = FFatherNode.FNodeID;
			OpenTable[OpenTableNum++]=NewNode;
			/*printf("Num ; %d, CostG: %f, CostH: %f, CostF: %f\n",OpenTableNum,OpenTable[OpenTableNum-1].FGcostval,OpenTable[OpenTableNum-1].FHcostval,OpenTable[OpenTableNum-1].FFcostval);*/
		}
	}
	if(OpenTableNum!=0)
		{
			Caculate.FFindMinNum(OpenTable,OpenTableNum);
			CloseTable[CloseTableNum++]=OpenTable[OpenTableNum-1];
			FFatherNode = OpenTable[OpenTableNum-1];//更新Father
			OpenTableNum--;
		}
		else
			printf("Can't Plan For Route!\n");
}

bool CFirstPlan::NodeJudge(CvPoint Point)
{
	if((Point.x<5)||(Point.x>2995)||(Point.y<5)||(Point.y>2995))
	{
		return(false);
	}
	else
	{
		for(int i=-3;i<4;i++)
		{
			for(int j=-3;j<4;j++)
			{
				if(
					cvGet2D(CostMap,Point.y+j,Point.x+i).val[0]>150
				  )
				{
					return(false);
				}
			}
		}
	}
	for(int m=0;m<CloseTableNum;m++)//如果该点已经访问过，则不继续访问
	{
		if( (Point.x==CloseTable[m].FNodeCoord.x)&&(Point.y==CloseTable[m].FNodeCoord.y) )
		{
			return(false);
		}
	}
	return(true);
}

CFirstPlan::~CFirstPlan(void)
{
}
