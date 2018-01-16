#pragma once
#include "stdafx.h"
#include "Node.h"
#include "Caculate.h"

class CFirstPlan
{
public:
	CFirstPlan(void);
	CNode::FirstPlanNode FFatherNode;
	CNode::FirstPlanNode* OpenTable;
	CNode::FirstPlanNode* CloseTable;
	CvPoint* FPathpoints;
	CCaculate Caculate;
	CvScalar white;
	CvScalar black;
	CvScalar blue;
	CvScalar green;
	CvScalar red;
	CvScalar orange;

	int CloseTableNum;
	int OpenTableNum;
	int FNodeIDcounter;
	int FPathpointsNum;
	double Angle;
	int FPointNum;

	void FGeneratePath(CNode::FirstPlanNode*,CNode::FirstPlanNode,IplImage*);
	void Gene_CldNode(CNode::FirstPlanNode);
	bool NodeJudge(CvPoint);
	IplImage* FirstPlan(IplImage*);

	~CFirstPlan(void);
};

