#pragma once
#include "Node.h"

class CCaculate
{
public:
	CCaculate(void);
	void FindMinNum(CNode::MainNode*, int);//Ѱ��MainNode���飨һ��intλ���������С���۽ڵ��λ��,����С���ƶ������һλ
	void FFindMinNum(CNode::FirstPlanNode*, int);
	double CaculateGcostVal(CvPoint*);//����ÿ�������ɽڵ��Gcostֵ;
	double CaculateHcostVal(CNode::MainNode,CvPoint);//����ÿ�������ɽڵ��Hcostֵ;
	~CCaculate(void);
};

