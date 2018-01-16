#include "stdafx.h"
#include "Caculate.h"
extern IplImage * CostMap;
extern double angleTarget;


CCaculate::CCaculate(void)
{
}
double CCaculate::CaculateGcostVal(CvPoint* nodes)
{
	double GcostValTemp=0.0;
	for(int i=0;i<20;i++)
	{
		 CvScalar PixelVal;
		 PixelVal = cvGet2D(CostMap,nodes[i].y,nodes[i].x);
		 GcostValTemp += PixelVal.val[0] + 3.0;
	}
	return(GcostValTemp); 
}
double CCaculate::CaculateHcostVal(CNode::MainNode CurNode,CvPoint TargetNode)
{
	double HcostValTemp=0.0;
	double angle=0.0,angleError=0.0;
	//if(TargetNode.x>CurNode.NodeCoord.x)
	//{
	//	angle = -180.0*atan((double)(TargetNode.y-CurNode.NodeCoord.y)/(double)(TargetNode.x-CurNode.NodeCoord.x))/M_PI;
	//	if(angle<0.0)
	//		angle = 360.0 + angle ;
	//}
	//else if(TargetNode.x<CurNode.NodeCoord.x)
	//{
	//	angle = 180.0 - 180.0*atan((double)(TargetNode.y-CurNode.NodeCoord.y)/(double)(TargetNode.x-CurNode.NodeCoord.x))/M_PI;
	//}
	//else if(TargetNode.y>CurNode.NodeCoord.y)
	//	angle = 270.0;
	//else 
	//	angle = 90.0;
	
	angle = angleTarget;

	switch(CurNode.AngleNum)
	{
		case 0:
			angleError = abs(90.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 1:
			angleError = abs(105.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 2:
			angleError = abs(120.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 3:
			angleError = abs(135.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 4:
			angleError = abs(150.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 5:
			angleError = abs(165.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 6:
			angleError = abs(180.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 7:
			angleError = abs(195.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 8:
			angleError = abs(210.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 9:
			angleError = abs(225.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 10:
			angleError = abs(240.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 11:
			angleError = abs(255.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 12:
			angleError = abs(270.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 13:
			angleError = abs(285.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 14:
			angleError = abs(300.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 15:
			angleError = abs(315.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 16:
			angleError = abs(330.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 17:
			angleError = abs(345.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 18:
			angleError = abs(0.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 19:
			angleError = abs(15.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 20:
			angleError = abs(30.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 21:
			angleError = abs(45.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 22:
			angleError = abs(60.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
		case 23:
			angleError = abs(75.0-angle);
			if(angleError>180.0)
				angleError = 360.0 - angleError;
			break;
	}
	//	printf("********************%d\n",CurNode.AngleNum);
	//	printf("********************%f\n",angle);
	//printf("********************%f\n",angleError);
	HcostValTemp =3.0*(abs(TargetNode.y-CurNode.NodeCoord.y)+abs(TargetNode.x-CurNode.NodeCoord.x))+ 5.0*angleError;
	//HcostValTemp = sqrt( (double)((CurNode.x-TargetNode.x)*(CurNode.x-TargetNode.x)+(CurNode.y-TargetNode.y)*(CurNode.y-TargetNode.y)) );
	return (HcostValTemp);
}
//void CCaculate::FindMinNum(extern CNode::MainNode* NodeGroup, extern int Num)
void CCaculate::FindMinNum(CNode::MainNode* NodeGroup, int Num)
{
	CNode::MainNode NodeTemp;
	int MinNodeNum=0;
	for(int i=Num-2;i>=0;i--)
	{
		if(NodeGroup[i].Fcostval<NodeGroup[Num-1].Fcostval)
		{
			NodeTemp = NodeGroup[Num-1];
			NodeGroup[Num-1] = NodeGroup[i];
			NodeGroup[i] = NodeTemp;
		}//选择出一个代价最小的节点，放到数组末尾
	}
}
//void CCaculate::FFindMinNum(extern CNode::FirstPlanNode* FNodeGroup, extern int FNum)
void CCaculate::FFindMinNum(CNode::FirstPlanNode* FNodeGroup, int FNum)
{
	CNode::FirstPlanNode NodeTemp;
	int MinNodeNum=0;
	for(int i=FNum-2;i>=0;i--)
	{
		if(FNodeGroup[i].FFcostval<FNodeGroup[FNum-1].FFcostval)
		{
			NodeTemp = FNodeGroup[FNum-1];
			FNodeGroup[FNum-1] = FNodeGroup[i];
			FNodeGroup[i] = NodeTemp;
		}//选择出一个代价最小的节点，放到数组末尾
	}
}
CCaculate::~CCaculate(void)
{
}
