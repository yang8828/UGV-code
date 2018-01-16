#pragma once
class CNode
{
public:
	CNode(void);
	struct FirstPlanNode
	{
		CvPoint FNodeCoord;
		int FChildNum;
		int FFatherNodeID;
		int FNodeID;
		double FGcostval;
		double FHcostval;
		double FFcostval;
	};
	struct MainNode
	{
		CvPoint NodeCoord;
		int ChildNum;
		int AngleNum;
		int FatherNodeID;
		int NodeID;
		double Gcostval;
		double Hcostval;
		double Fcostval;
	};
	CvPoint Rot_Deal(double, CvPoint);
	int Node_Init(void);
	CNode* ChildNode;
	CvPoint* nodesTem1;
	CvPoint* nodesTem2;
	CvPoint* nodesTem3;
	CvPoint* nodesTem4;
	CvPoint* nodesTem5;
	CvPoint* nodesTem6;
	CvPoint* nodesTem7;
	CvPoint* nodesTem8;
	CvPoint* nodesTem9;
	CvPoint* nodesTem10;
	CvPoint* nodesTem11;
	CvPoint* nodesTem12;
	CvPoint* nodesTem13;
	CvPoint* nodesTem14;
	CvPoint* nodesTem15;
	CvPoint* nodesTem16;
	CvPoint* nodesTem17;
	CvPoint* nodesTem18;
	CvPoint* nodesTem19;
	CvPoint* nodesTem20;
	CvPoint* nodesTem21;
	CvPoint* nodesTem22;
	CvPoint* nodesTem23;
	CvPoint* nodesTem24;
	~CNode(void);
};

