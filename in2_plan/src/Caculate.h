#pragma once
#include "Node.h"

class CCaculate
{
public:
	CCaculate(void);
	void FindMinNum(CNode::MainNode*, int);//寻找MainNode数组（一共int位）里具有最小代价节点的位置,把最小的移动至最后一位
	void FFindMinNum(CNode::FirstPlanNode*, int);
	double CaculateGcostVal(CvPoint*);//计算每个新生成节点的Gcost值;
	double CaculateHcostVal(CNode::MainNode,CvPoint);//计算每个新生成节点的Hcost值;
	~CCaculate(void);
};

