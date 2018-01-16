#include "stdafx.h"
#include "Node.h"

CNode::CNode(void)
{
	nodesTem1 = new CvPoint[160];//0度
	nodesTem2 = new CvPoint[160];//15度
	nodesTem3 = new CvPoint[160];//30度
	nodesTem4 = new CvPoint[160];//45度
	nodesTem5 = new CvPoint[160];//60度
	nodesTem6 = new CvPoint[160];//75度
	nodesTem7 = new CvPoint[160];//90度
	nodesTem8 = new CvPoint[160];//105度
	nodesTem9 = new CvPoint[160];//120度
	nodesTem10 = new CvPoint[160];//135度
	nodesTem11 = new CvPoint[160];//150度
	nodesTem12 = new CvPoint[160];//165度
	nodesTem13 = new CvPoint[160];//180度
	nodesTem14 = new CvPoint[160];//195度
	nodesTem15 = new CvPoint[160];//210度
	nodesTem16 = new CvPoint[160];//225度
	nodesTem17 = new CvPoint[160];//240度
	nodesTem18 = new CvPoint[160];//255度
	nodesTem19 = new CvPoint[160];//270度
	nodesTem20 = new CvPoint[160];//285度
	nodesTem21 = new CvPoint[160];//300度
	nodesTem22 = new CvPoint[160];//315度
	nodesTem23 = new CvPoint[160];//330度
	nodesTem24 = new CvPoint[160];//345度
	Node_Init();
}//当实例化后，构造函数会自动调的

CvPoint CNode::Rot_Deal(double angle, CvPoint node)
{
	CvPoint tempnode;
	tempnode.x = (int)((double)node.x * cos(angle) + (double)node.y * sin(angle));
	tempnode.y = (int)((double)node.y * cos(angle) - (double)node.x * sin(angle));
	return (tempnode);
}

int CNode::Node_Init()
{
	int nodesx[8][20] = {
	{0,0,0,0,0,0,1,1,1,1,2,2,2,3,3,3,4,4,5,5},
	{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,3},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,-2,-2,-2,-2,-3},
	{0,0,0,0,0,0,-1,-1,-1,-1,-2,-2,-2,-3,-3,-3,-4,-4,-5,-5},
	{0,0,0,0,0,0,0,0,-1,-1,-1,-1,-1,-1,-1,-2,-2,-2,-2,-3},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,3}
	};
	int nodesy[8][20] = {
	{-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20},
	{-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20},
	{-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20},
	{-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20},
	{-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20},
	{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20},
	{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20},
	{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}
	};//Node模板
	
	double angle[24]={0.0,15.0,30.0,45.0,60.0,75.0,90.0,
	105.0,120.0,135.0,150.0,165.0,180.0,
	195.0,210.0,225.0,240.0,255.0,270.0,
	285.0,300.0,315.0,330.0,345.0};

	for(int i=0; i<160; i++)
	{
		int j=0;
		double angle1 = angle[j++]*M_PI/180.0;
		nodesTem1[i] = Rot_Deal(angle1,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle2 = angle[j++]*M_PI/180.0;
		nodesTem2[i] = Rot_Deal(angle2,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle3 = angle[j++]*M_PI/180.0;
		nodesTem3[i] = Rot_Deal(angle3,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle4 = angle[j++]*M_PI/180.0;
		nodesTem4[i] = Rot_Deal(angle4,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle5 = angle[j++]*M_PI/180.0;
		nodesTem5[i] = Rot_Deal(angle5,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle6 = angle[j++]*M_PI/180.0;
		nodesTem6[i] = Rot_Deal(angle6,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle7 = angle[j++]*M_PI/180.0;
		nodesTem7[i] = Rot_Deal(angle7,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle8 = angle[j++]*M_PI/180.0;
		nodesTem8[i] = Rot_Deal(angle8,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle9 = angle[j++]*M_PI/180.0;
		nodesTem9[i] = Rot_Deal(angle9,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle10 = angle[j++]*M_PI/180.0;
		nodesTem10[i] = Rot_Deal(angle10,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle11 = angle[j++]*M_PI/180.0;
		nodesTem11[i] = Rot_Deal(angle11,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle12 = angle[j++]*M_PI/180.0;
		nodesTem12[i] = Rot_Deal(angle12,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle13 = angle[j++]*M_PI/180.0;
		nodesTem13[i] = Rot_Deal(angle13,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle14 = angle[j++]*M_PI/180.0;
		nodesTem14[i] = Rot_Deal(angle14,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle15 = angle[j++]*M_PI/180.0;
		nodesTem15[i] = Rot_Deal(angle15,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle16 = angle[j++]*M_PI/180.0;
		nodesTem16[i] = Rot_Deal(angle16,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle17 = angle[j++]*M_PI/180.0;
		nodesTem17[i] = Rot_Deal(angle17,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle18 = angle[j++]*M_PI/180.0;
		nodesTem18[i] = Rot_Deal(angle18,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle19 = angle[j++]*M_PI/180.0;
		nodesTem19[i] = Rot_Deal(angle19,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle20 = angle[j++]*M_PI/180.0;
		nodesTem20[i] = Rot_Deal(angle20,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle21 = angle[j++]*M_PI/180.0;
		nodesTem21[i] = Rot_Deal(angle21,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle22 = angle[j++]*M_PI/180.0;
		nodesTem22[i] = Rot_Deal(angle22,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle23 = angle[j++]*M_PI/180.0;
		nodesTem23[i] = Rot_Deal(angle23,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));
		double angle24 = angle[j++]*M_PI/180.0;
		nodesTem24[i] = Rot_Deal(angle24,cvPoint(nodesx[i/20][i%20],nodesy[i/20][i%20]));

	}//得到旋转24个角度的模板节点
	
	return 1;
}

CNode::~CNode(void)
{
}
