#pragma once
#include "stdafx.h"

class CCreatObsMap
{
private:
	IplImage * Gauss_Image;
	IplImage * Temp_Img;
	IplImage * Temp_Img2;
	IplImage * smooth_image;
	IplImage * Dilate_Image;

public:
	CCreatObsMap(void);
	
	IplImage * CreatObsMap(void);

	~CCreatObsMap(void);
};

