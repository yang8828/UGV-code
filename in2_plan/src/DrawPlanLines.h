#pragma once
#include "stdafx.h"
#include "Node.h"
#include "Caculate.h"

class CDrawPlanLines
{
private:
	 CvScalar white;
	 CvScalar black;
	 CvScalar blue;
	 CvScalar green;
	 CvScalar red;
	 void DrawNodeTree(CvPoint, int, IplImage*);
	 void GeneralTree(CvPoint, int);
	 void GenerateMainPath(CNode::MainNode*,CNode::MainNode,IplImage*);

public:
	
	CCaculate Caculate;
	CNode::MainNode FatherNodeGiven;
	CNode::MainNode* NewNode;
	CNode Node;
	CNode::MainNode* MainNodes;
	CNode::MainNode* VisitedNodes;
	CNode::MainNode* MainPathNodes;
	int GroupElemNum;
	int CloseNodesNum;
	int MainPathNodesNum;
	int PathNodesNum;
	int NodeIDcounter;
	int PathPointsNum;
	CvPoint *PathNodes;
	CvPoint* nodes1;
	CvPoint* nodes2;
	CvPoint* nodes3;
	CvPoint* nodes4;
	CvPoint* nodes5;
	CvPoint* nodes6;
	CvPoint* nodes7;
	CvPoint* nodes8;
	CvPoint* Nodes;
	CvPoint* PathPoints;
	int TargetNum;
	bool* DrawTreeJudge;
	CDrawPlanLines(void);
	
	IplImage* DrawPlanLines(IplImage*);
	~CDrawPlanLines(void);
};


