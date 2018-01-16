#include "stdafx.h"
#include "CreatObsMap.h"
extern IplImage* MoPlanMap;

CCreatObsMap::CCreatObsMap(void)
{	

}

IplImage* CCreatObsMap::CreatObsMap(void)
{
	Temp_Img = cvCreateImage(cvSize(MoPlanMap->width,MoPlanMap->height),MoPlanMap->depth,1);
	Temp_Img2 = cvCreateImage(cvSize(MoPlanMap->width,MoPlanMap->height),MoPlanMap->depth,1);	
	smooth_image = cvCreateImage(cvSize(MoPlanMap->width,MoPlanMap->height),MoPlanMap->depth,1);
	Dilate_Image = cvCreateImage(cvSize(MoPlanMap->width,MoPlanMap->height),MoPlanMap->depth,1);
	Gauss_Image = cvCreateImage(cvSize(MoPlanMap->width,MoPlanMap->height),MoPlanMap->depth,1);

	cvSplit(MoPlanMap,Temp_Img,0,0,0);
	int value;
	for(int i=0;i<Temp_Img->height;i++)
	    for(int j=0;j<Temp_Img->width;j++)
	{
	    value = (uchar)Temp_Img->imageData[ i * Temp_Img->width + j];
	    if(value > 180)
	    {
		Temp_Img2->imageData[ i * Temp_Img->width + j] = 255;
	    }	
	    else
	   {
		Temp_Img2->imageData[ i * Temp_Img->width + j] = 0;
	   }
	}	
	cvDilate(Temp_Img2,Dilate_Image,0,5);
	cvSmooth(Dilate_Image,Gauss_Image,CV_GAUSSIAN,13,13);
	cvCvtColor(Gauss_Image,MoPlanMap,CV_GRAY2RGB);
	return(Gauss_Image);
}

CCreatObsMap::~CCreatObsMap(void)
{
	cvReleaseImage(&Temp_Img);
	cvReleaseImage(&Temp_Img2);
	cvReleaseImage(&smooth_image);
	cvReleaseImage(&Dilate_Image);
}
