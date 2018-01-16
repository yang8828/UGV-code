#include "stdafx.h"
#include "DrawPlanLines.h"
#include "Node.h"
extern CvPoint* TargetPoint;
extern CvPoint StartPoint;
extern IplImage * MoPlanMap;
extern IplImage * CostMap;
extern int AllTargetNum;
bool Error;
extern bool *SecondPlanSuccess;
extern int CarAngleNow;

CDrawPlanLines::CDrawPlanLines(void)
{
	white = CV_RGB(255,255,255);
	black = CV_RGB(0,0,0);
	blue = CV_RGB(0,0,255);
    	green = CV_RGB(0,255,0);
	red = CV_RGB(255,0,0);

	MainNodes =  new CNode::MainNode[2000000];//路径主节点,open表
	VisitedNodes = new CNode::MainNode[100000];//close表
	MainPathNodes = new CNode::MainNode[10000];

	PathNodes = new CvPoint[20000];//形成的路径，最大保存5000个点，也就是1000米。
	PathPoints = new CvPoint[1000000];
	NewNode = new CNode::MainNode[8];
	nodes1 = new CvPoint[20];
	nodes2 = new CvPoint[20];
	nodes3 = new CvPoint[20];
	nodes4 = new CvPoint[20];
	nodes5 = new CvPoint[20]; 
	nodes6 = new CvPoint[20];
	nodes7 = new CvPoint[20]; 
	nodes8 = new CvPoint[20];
	Nodes = new CvPoint[20];
	
	DrawTreeJudge = new bool[8];
	
	//路径子节点模型模板
}
void CDrawPlanLines::GeneralTree(CvPoint center_node, int angle)
{
	switch (angle)
	{
	case 0:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem1[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem1[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem1[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem1[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem1[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem1[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem1[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem1[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem1[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem1[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem1[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem1[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem1[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem1[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem1[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem1[i+140].y+center_node.y;
		}
		break;
	case 1:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem2[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem2[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem2[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem2[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem2[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem2[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem2[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem2[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem2[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem2[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem2[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem2[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem2[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem2[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem2[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem2[i+140].y+center_node.y;
		}
		break;
	case 2:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem3[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem3[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem3[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem3[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem3[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem3[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem3[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem3[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem3[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem3[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem3[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem3[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem3[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem3[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem3[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem3[i+140].y+center_node.y;
		}
		break;
	case 3:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem4[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem4[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem4[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem4[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem4[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem4[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem4[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem4[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem4[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem4[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem4[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem4[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem4[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem4[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem4[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem4[i+140].y+center_node.y;
		}
		break;
	case 4:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem5[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem5[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem5[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem5[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem5[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem5[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem5[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem5[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem5[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem5[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem5[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem5[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem5[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem5[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem5[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem5[i+140].y+center_node.y;
		}
		break;
	case 5:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem6[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem6[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem6[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem6[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem6[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem6[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem6[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem6[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem6[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem6[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem6[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem6[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem6[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem6[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem6[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem6[i+140].y+center_node.y;
		}
		break;
	case 6:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem7[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem7[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem7[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem7[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem7[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem7[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem7[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem7[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem7[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem7[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem7[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem7[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem7[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem7[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem7[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem7[i+140].y+center_node.y;
		}
		break;
	case 7:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem8[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem8[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem8[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem8[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem8[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem8[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem8[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem8[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem8[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem8[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem8[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem8[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem8[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem8[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem8[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem8[i+140].y+center_node.y;
		}
		break;
	case 8:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem9[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem9[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem9[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem9[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem9[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem9[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem9[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem9[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem9[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem9[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem9[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem9[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem9[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem9[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem9[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem9[i+140].y+center_node.y;
		}
		break;
	case 9:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem10[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem10[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem10[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem10[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem10[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem10[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem10[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem10[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem10[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem10[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem10[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem10[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem10[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem10[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem10[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem10[i+140].y+center_node.y;
		}
		break;
	case 10:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem11[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem11[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem11[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem11[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem11[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem11[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem11[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem11[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem11[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem11[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem11[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem11[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem11[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem11[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem11[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem11[i+140].y+center_node.y;
		}
		break;
	case 11:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem12[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem12[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem12[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem12[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem12[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem12[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem12[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem12[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem12[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem12[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem12[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem12[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem12[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem12[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem12[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem12[i+140].y+center_node.y;
		}
		break;
	case 12:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem13[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem13[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem13[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem13[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem13[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem13[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem13[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem13[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem13[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem13[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem13[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem13[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem13[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem13[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem13[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem13[i+140].y+center_node.y;
		}
		break;
	case 13:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem14[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem14[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem14[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem14[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem14[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem14[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem14[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem14[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem14[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem14[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem14[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem14[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem14[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem14[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem14[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem14[i+140].y+center_node.y;
		}
		break;
	case 14:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem15[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem15[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem15[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem15[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem15[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem15[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem15[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem15[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem15[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem15[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem15[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem15[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem15[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem15[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem15[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem15[i+140].y+center_node.y;
		}
		break;
	case 15:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem16[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem16[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem16[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem16[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem16[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem16[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem16[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem16[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem16[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem16[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem16[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem16[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem16[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem16[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem16[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem16[i+140].y+center_node.y;
		}
		break;
	case 16:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem17[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem17[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem17[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem17[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem17[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem17[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem17[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem17[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem17[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem17[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem17[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem17[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem17[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem17[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem17[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem17[i+140].y+center_node.y;
		}
		break;
	case 17:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem18[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem18[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem18[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem18[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem18[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem18[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem18[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem18[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem18[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem18[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem18[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem18[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem18[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem18[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem18[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem18[i+140].y+center_node.y;
		}
		break;
	case 18:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem19[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem19[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem19[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem19[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem19[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem19[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem19[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem19[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem19[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem19[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem19[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem19[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem19[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem19[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem19[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem19[i+140].y+center_node.y;
		}
		break;
	case 19:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem20[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem20[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem20[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem20[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem20[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem20[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem20[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem20[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem20[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem20[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem20[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem20[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem20[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem20[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem20[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem20[i+140].y+center_node.y;
		}
		break;
	case 20:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem21[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem21[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem21[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem21[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem21[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem21[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem21[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem21[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem21[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem21[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem21[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem21[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem21[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem21[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem21[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem21[i+140].y+center_node.y;
		}
		break;
	case 21:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem22[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem22[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem22[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem22[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem22[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem22[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem22[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem22[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem22[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem22[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem22[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem22[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem22[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem22[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem22[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem22[i+140].y+center_node.y;
		}
		break;
	case 22:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem23[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem23[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem23[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem23[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem23[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem23[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem23[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem23[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem23[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem23[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem23[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem23[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem23[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem23[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem23[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem23[i+140].y+center_node.y;
		}
		break;
	case 23:
		for(int i=0;i<20;i++)
		{
			nodes1[i].x=Node.nodesTem24[i].x+center_node.x;
			nodes1[i].y=Node.nodesTem24[i].y+center_node.y;
			nodes2[i].x=Node.nodesTem24[i+20].x+center_node.x;
			nodes2[i].y=Node.nodesTem24[i+20].y+center_node.y;
			nodes3[i].x=Node.nodesTem24[i+40].x+center_node.x;
			nodes3[i].y=Node.nodesTem24[i+40].y+center_node.y;
			nodes4[i].x=Node.nodesTem24[i+60].x+center_node.x;
			nodes4[i].y=Node.nodesTem24[i+60].y+center_node.y;
			nodes5[i].x=Node.nodesTem24[i+80].x+center_node.x;
			nodes5[i].y=Node.nodesTem24[i+80].y+center_node.y;
			nodes6[i].x=Node.nodesTem24[i+100].x+center_node.x;
			nodes6[i].y=Node.nodesTem24[i+100].y+center_node.y;
			nodes7[i].x=Node.nodesTem24[i+120].x+center_node.x;
			nodes7[i].y=Node.nodesTem24[i+120].y+center_node.y;
			nodes8[i].x=Node.nodesTem24[i+140].x+center_node.x;
			nodes8[i].y=Node.nodesTem24[i+140].y+center_node.y;
		}
		break;
	default:
		break;
	}
	bool NewNodeJudge = true;
	//保存新生成的主节点
	for(int k=0;k<8;k++)//生成新的8个子节点
	{
		NewNode[k].ChildNum = k+1;
		switch(k)
		{
		case 0:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes1[i].x<5) || (nodes1[i].x>2995) || (nodes1[i].y<5) || (nodes1[i].y>2995) )
				{					
					NewNodeJudge = false;
					DrawTreeJudge[0] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes1[i].y,nodes1[i].x).val[0]>150 )	
					{
						NewNodeJudge = false;
						DrawTreeJudge[0] =false;
						break;
					}
				}
			}

			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes1[19].x;
				NewNode[k].NodeCoord.y=nodes1[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+22)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes1)+FatherNodeGiven.Gcostval+10.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;//转大弯的代价较大
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes1[19].x;
				NewNode[k].NodeCoord.y=nodes1[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+22)%24;
				NewNode[k].Gcostval = 1000000.0;
				NewNode[k].Hcostval = 1000000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;//转大弯的代价较大
			}
			break;
		case 1:			
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes2[i].x<5) || (nodes2[i].x>2995) || (nodes2[i].y<5) || (nodes2[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[1] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes2[i].y,nodes2[i].x).val[0]>150 )	
					{
						NewNodeJudge = false;
						DrawTreeJudge[1] =false;
						break;
					}
				}
			}

			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes2[19].x;
				NewNode[k].NodeCoord.y=nodes2[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+23)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes2)+FatherNodeGiven.Gcostval+5.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			else
			{			
				NewNode[k].NodeCoord.x=nodes2[19].x;
				NewNode[k].NodeCoord.y=nodes2[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+23)%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			break;
		case 2:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes3[i].x<5) || (nodes3[i].x>2995) || (nodes3[i].y<5) || (nodes3[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[2] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes3[i].y,nodes3[i].x).val[0]>150 )	
					{
						NewNodeJudge = false;
						DrawTreeJudge[2] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes3[19].x;
				NewNode[k].NodeCoord.y=nodes3[19].y;
				NewNode[k].AngleNum = FatherNodeGiven.AngleNum%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes3)+FatherNodeGiven.Gcostval;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;//直线的代价权重较小
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes3[19].x;
				NewNode[k].NodeCoord.y=nodes3[19].y;
				NewNode[k].AngleNum = FatherNodeGiven.AngleNum%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;//直线的代价权重较小
			}
			break;
		case 3:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes4[i].x<5) || (nodes4[i].x>2995) || (nodes4[i].y<5) || (nodes4[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[3] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes4[i].y,nodes4[i].x).val[0]>150)	
					{
						NewNodeJudge = false;
						DrawTreeJudge[3] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{

				NewNode[k].NodeCoord.x=nodes4[19].x;
				NewNode[k].NodeCoord.y=nodes4[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+1)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes4)+FatherNodeGiven.Gcostval+5.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes4[19].x;
				NewNode[k].NodeCoord.y=nodes4[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+1)%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			break;
		case 4:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes5[i].x<5) || (nodes5[i].x>2995) || (nodes5[i].y<5) || (nodes5[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[4] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes5[i].y,nodes5[i].x).val[0]>150 )	
					{
						NewNodeJudge = false;
						DrawTreeJudge[4] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes5[19].x;
				NewNode[k].NodeCoord.y=nodes5[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+2)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes5)+FatherNodeGiven.Gcostval+10.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes5[19].x;
				NewNode[k].NodeCoord.y=nodes5[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+2)%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			break;
		case 5:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes6[i].x<5) || (nodes6[i].x>2995) || (nodes6[i].y<5) || (nodes6[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[5] =false;
					break;
				}
				else
				{	
					if( cvGet2D(CostMap,nodes6[i].y,nodes6[i].x).val[0]>150 )	
					{
						NewNodeJudge = false;
						DrawTreeJudge[5] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes6[19].x; 
				NewNode[k].NodeCoord.y=nodes6[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+23)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes6)+FatherNodeGiven.Gcostval+150.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes6[19].x; 
				NewNode[k].NodeCoord.y=nodes6[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+23)%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}		
			break;
		case 6:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes7[i].x<5) || (nodes7[i].x>2995) || (nodes7[i].y<5) || (nodes7[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[6] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes7[i].y,nodes7[i].x).val[0]>150)	
					{
						NewNodeJudge = false;
						DrawTreeJudge[6] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes7[19].x;
				NewNode[k].NodeCoord.y=nodes7[19].y;
				NewNode[k].AngleNum = FatherNodeGiven.AngleNum%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes7)+FatherNodeGiven.Gcostval+100.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			else
			{			
				NewNode[k].NodeCoord.x=nodes7[19].x;
				NewNode[k].NodeCoord.y=nodes7[19].y;
				NewNode[k].AngleNum = FatherNodeGiven.AngleNum%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			break;
		case 7:
			NewNodeJudge = true;
			for(int i=0;i<20;i++)
			{
				if( (nodes8[i].x<5) || (nodes8[i].x>2995) || (nodes8[i].y<5) || (nodes8[i].y>2995) )
				{
					NewNodeJudge = false;
					DrawTreeJudge[7] =false;
					break;
				}
				else
				{
					if( cvGet2D(CostMap,nodes8[i].y,nodes8[i].x).val[0]>150)	
					{
						NewNodeJudge = false;
						DrawTreeJudge[7] =false;
						break;
					}
				}
			}
			if(NewNodeJudge)
			{
				NewNode[k].NodeCoord.x=nodes8[19].x;
				NewNode[k].NodeCoord.y=nodes8[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+1)%24;
				NewNode[k].Gcostval = Caculate.CaculateGcostVal(nodes8)+FatherNodeGiven.Gcostval+150.0;
				NewNode[k].Hcostval = Caculate.CaculateHcostVal(NewNode[k],TargetPoint[TargetNum]);
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;//倒车的代价较大
			}
			else
			{
				NewNode[k].NodeCoord.x=nodes8[19].x;
				NewNode[k].NodeCoord.y=nodes8[19].y;
				NewNode[k].AngleNum = (FatherNodeGiven.AngleNum+1)%24;
				NewNode[k].Gcostval = 100000.0;
				NewNode[k].Hcostval = 100000.0;
				NewNode[k].Fcostval = NewNode[k].Gcostval+NewNode[k].Hcostval;
			}
			break;
		}
		if(NewNodeJudge)
		{
			NodeIDcounter++;
			NewNode[k].NodeID = NodeIDcounter;
			NewNode[k].FatherNodeID = FatherNodeGiven.NodeID;
			MainNodes[GroupElemNum++]=NewNode[k];//把新生成的子节点也压入Opne表
			/*printf("NewNode_NodeCoord : %d  %d\n",NewNode[k].NodeCoord.x,NewNode[k].NodeCoord.y);
			printf("NewNode_AngleNum : %d  \n",NewNode[k].AngleNum);
			printf("NewNode_Cost : %f %f %f\n",NewNode[k].Gcostval,NewNode[k].Hcostval,NewNode[k].Fcostval);
			printf("OPENNum : %d  \n",GroupElemNum);*/
		}
	}

SecondTime:	
	if(GroupElemNum!=0)
	{
		Caculate.FindMinNum(MainNodes,GroupElemNum);//选择一个最小代价的路线
		for(int i=0;i<CloseNodesNum;i++)//遍历已访问节点，如果完全状态一致，放弃这条路线，以免陷入无限循环
		{
			if((MainNodes[GroupElemNum-1].AngleNum==VisitedNodes[i].AngleNum)&&(MainNodes[GroupElemNum-1].NodeCoord.x==VisitedNodes[i].NodeCoord.x)
				&&(MainNodes[GroupElemNum-1].NodeCoord.y==VisitedNodes[i].NodeCoord.y))
				{
					GroupElemNum--;
					goto SecondTime;
				}
		}

		VisitedNodes[CloseNodesNum++]=MainNodes[GroupElemNum-1];//Close表压入节点
		FatherNodeGiven = MainNodes[GroupElemNum-1];
		GroupElemNum--;//把已访问节点pop出来
		/*printf("*************** SelectVal: %f %f %f\n",VisitedNodes[CloseNodesNum-1].Gcostval,VisitedNodes[CloseNodesNum-1].Hcostval,VisitedNodes[CloseNodesNum-1].Fcostval);
		printf("*************** CLOSENum: %d  \n",CloseNodesNum);*/
	}
	else
	{
		if(Error)
		{
			Error = false;
			printf("Can't Plan For Sencond Route!\n");
		}
	}
}
void CDrawPlanLines::DrawNodeTree(CvPoint FatherNode , int NodeAngle, IplImage* TempImg)
{

	GeneralTree(FatherNode,NodeAngle);

	for(int i=0; i<20; i++)
	{
		if(i!=0)
		{
			if(DrawTreeJudge[0])
				cvLine(TempImg,nodes1[i-1], nodes1[i], red);
			if(DrawTreeJudge[1])
				cvLine(TempImg,nodes2[i-1], nodes2[i], red);
			if(DrawTreeJudge[2])
				cvLine(TempImg,nodes3[i-1], nodes3[i], red);
			if(DrawTreeJudge[3])
				cvLine(TempImg,nodes4[i-1], nodes4[i], red);
			if(DrawTreeJudge[4])
				cvLine(TempImg,nodes5[i-1], nodes5[i], red);
			if(DrawTreeJudge[5])
				cvLine(TempImg,nodes6[i-1], nodes6[i], blue);
			if(DrawTreeJudge[6])
				cvLine(TempImg,nodes7[i-1], nodes7[i], blue);
			if(DrawTreeJudge[7])
				cvLine(TempImg,nodes8[i-1], nodes8[i], blue);
		}
	}

	DrawTreeJudge[0] = true;
	DrawTreeJudge[1] = true;
	DrawTreeJudge[2] = true;
	DrawTreeJudge[3] = true;
	DrawTreeJudge[4] = true;
	DrawTreeJudge[5] = true;
	DrawTreeJudge[6] = true;
	DrawTreeJudge[7] = true;
}
IplImage* CDrawPlanLines::DrawPlanLines(IplImage* ObsImg)
{
	GroupElemNum = 0;
	CloseNodesNum = 0;
	MainPathNodesNum = 0;
	PathNodesNum = 0;
	TargetNum = 0;
	PathPointsNum = 0;
	NodeIDcounter = 0;

	Error = true;
	FatherNodeGiven.AngleNum = CarAngleNow;
	FatherNodeGiven.NodeCoord = StartPoint;
	FatherNodeGiven.ChildNum = 0;
	FatherNodeGiven.Gcostval = 0.0;
	FatherNodeGiven.Hcostval = 0.0;
	FatherNodeGiven.NodeID = 0;
	CloseNodesNum=0;
	VisitedNodes[CloseNodesNum++] = FatherNodeGiven;

	DrawTreeJudge[0] = true;
	DrawTreeJudge[1] = true;
	DrawTreeJudge[2] = true;
	DrawTreeJudge[3] = true;
	DrawTreeJudge[4] = true;
	DrawTreeJudge[5] = true;
	DrawTreeJudge[6] = true;
	DrawTreeJudge[7] = true;
	TargetNum=0;
	double *distance = new double[AllTargetNum];
    
	//printf("%d ",AllTargetNum);
	for(int j=0;j<3000;j++)
	{
		distance[TargetNum] = sqrt( (double)((FatherNodeGiven.NodeCoord.x-TargetPoint[TargetNum].x)*(FatherNodeGiven.NodeCoord.x-TargetPoint[TargetNum].x)+(FatherNodeGiven.NodeCoord.y-TargetPoint[TargetNum].y)*(FatherNodeGiven.NodeCoord.y-TargetPoint[TargetNum].y)) );
		if(distance[TargetNum] < 25.0)//距离目标0.5米时，停止规划
		{
			SecondPlanSuccess[TargetNum] = true;
		        GenerateMainPath(VisitedNodes,FatherNodeGiven,ObsImg);//画出主路径
			StartPoint = cvPoint(FatherNodeGiven.NodeCoord.x,FatherNodeGiven.NodeCoord.y);//更新起点
			CloseNodesNum = 0;
			GroupElemNum= 0;
			TargetNum++;
			if(TargetNum >= AllTargetNum)
				break;
			else
				VisitedNodes[CloseNodesNum++] = FatherNodeGiven;//VisitedNodes中第一个始终是起点，最后一个始终是终点
		}
		else if(AllTargetNum==0)
		{
			break;
		}
		else
			DrawNodeTree(FatherNodeGiven.NodeCoord,FatherNodeGiven.AngleNum,ObsImg);

		//printf("**********************************************\n");
		/*cvWaitKey();
		cvShowImage("PLANNING2",ObsImg);*/
		if(j==2999)
		{
			for(int g = TargetNum; g<AllTargetNum ; g++)
			{
				SecondPlanSuccess[g] = false;
			}
		}
	}
	delete[] distance;
	return(ObsImg);
}
void CDrawPlanLines::GenerateMainPath(CNode::MainNode* MainNodesGroup,CNode::MainNode EndNode,IplImage* TempImg)
{
	CNode::MainNode CurNode;
	CurNode = EndNode;
	MainPathNodesNum = 0;
	while( CurNode.NodeID != MainNodesGroup[0].NodeID )//没有找到起点，就一直寻找
	{
		MainPathNodes[MainPathNodesNum++] = CurNode;//第一个是终点，依次到起点
		for(int i=0;i<(CloseNodesNum-1);i++)
		{
			if(CurNode.FatherNodeID==MainNodesGroup[i].NodeID)//回溯找父节点
			{
				CurNode = MainNodesGroup[i];
				break;
			}
		}
	}
	MainPathNodes[MainPathNodesNum++] = CurNode;//这个MainPathNodes数组里不包括起始点,这是第一层节点
	//for(int i=1;i<MainPathNodesNum;i++)
	//{
	//	cvLine(TempImg,MainPathNodes[i].NodeCoord, MainPathNodes[i-1].NodeCoord, green, 2);
	//	/*printf("路径点： %d  %d\n",MainPathNodes[i].NodeCoord.x,MainPathNodes[i].NodeCoord.y);*/		
	//}
	for(int j=(MainPathNodesNum-1);j>0;j--)//画出详细的路径，从起点开始
	{
		switch(MainPathNodes[j].AngleNum)
		{
			case 0:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem1[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem1[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 1:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem2[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem2[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 2:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem3[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem3[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 3:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem4[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem4[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 4:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem5[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem5[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 5:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem6[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem6[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 6:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem7[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem7[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 7:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem8[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem8[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 8:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem9[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem9[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 9:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem10[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem10[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 10:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem11[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem11[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 11:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem12[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem12[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 12:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem13[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem13[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 13:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem14[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem14[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 14:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem15[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem15[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 15:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem16[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem16[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 16:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem17[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem17[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 17:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem18[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem18[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 18:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem19[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem19[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 19:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem20[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem20[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 20:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem21[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem21[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 21:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem22[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem22[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;	
			case 22:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem23[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem23[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			case 23:
				switch(MainPathNodes[j-1].ChildNum)//判断其子节点是第几个儿子
				{
					case 1:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 2:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+20].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+20].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 3:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+40].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+40].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 4:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+60].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+60].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 5:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+80].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+80].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 6:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+100].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+100].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 7:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+120].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+120].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
					case 8:
						for(int i=0;i<20;i++)
						{
							Nodes[i].x=Node.nodesTem24[i+140].x+MainPathNodes[j].NodeCoord.x;
							Nodes[i].y=Node.nodesTem24[i+140].y+MainPathNodes[j].NodeCoord.y;
						}
						break;
				}
				break;
			}
		
		PathPoints[PathPointsNum++]=Nodes[0];//保存20个点，第一个是从父节点开始的
		for(int i=1;i<20;i++)
		{
			cvLine(TempImg,Nodes[i-1], Nodes[i], green, 1);
			PathPoints[PathPointsNum++]=Nodes[i];
		}//画出来一段小路径，从一个主节点到下一个主节点

	}
	//for(int i=0;i<PathPointsNum;i++)//显示出一个个路径点的坐标
	//{
	//	printf("PointNum: %ld  X: %d  Y: %d \n", i,PathPoints[i].x,PathPoints[i].y);
	//}
}
CDrawPlanLines::~CDrawPlanLines(void)
{
}
