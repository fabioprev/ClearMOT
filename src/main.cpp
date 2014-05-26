/*
Author:         Jianming Zhang
Note:           This application computes CLEAR MOT metric for 2D tracking.
                Objects swapping IDs will be counted as 2 ID switches. If you
                specify a sequence path for 'calMOT()', the matching results will
                be displayed frame by frame. 
                                        
                Green thin and thick boxes: good matches;
                Red thin boxes: missed targets; 
                Red thick boxes: false alarms;
                Yellow thin and thick boxes: ID swithes; 
*/



#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#include "libxml/parser.h"
#include "libxml/tree.h"
#include "libxml/xmlmemory.h"

#include "clearMOT.h"
#include "dataReader.h"

//#define GT_XML_FILE "../groundtruth/Groundtruth-dtv9_clip_3700_7500.xml"
#define GT_XML_FILE "../groundtruth/Groundtruth-Honk-Kong.xml"
//#define GT_XML_FILE "../groundtruth/GroundTruth-Wester_LLTV_VELA.xml"
//#define GT_XML_FILE "../groundtruth/PETS2009-S2L1.xml"
//#define GT_XML_FILE "../groundtruth/TUD-Campus.xml"

//#define HP_XML_FILE "../results/PTracker-dtv9_clip_3700_7500.xml"
#define HP_XML_FILE "../results/PTracker-Honk-Kong.xml"
//#define HP_XML_FILE "../results/PTracker-Wester_LLTV_VELA.xml"
//#define HP_XML_FILE "../results/PTracker-PETS-2009.xml"
//#define HP_XML_FILE "../results/PTracker-TUD-Campus.xml"

//#define SEQUENCE_PATH "../../BoatDetector/Datasets/dtv9_clip_3700_7500/"
#define SEQUENCE_PATH "../../BoatDetector/Datasets/Honk-Kong/"
//#define SEQUENCE_PATH "../../BoatDetector/Datasets/Wester_LLTV_VELA/"
//#define SEQUENCE_PATH "../../DistributedTracker/Datasets/Crowd-PETS09/S2/L1/Time_12-34/View_001/"

class SeqReader
{
	int frameCount;
	string path;

public:
	SeqReader(const string p):frameCount(0),path(p){}
	void readImg(Mat& frame)
	{
		string fileName=path+"frame_";
		char _count[10];
		sprintf(_count,"%d",frameCount);
		string count=_count;
		count.insert(count.begin(),3-count.length(),'0');
		count=count+".png";
		fileName=fileName+count;
		frame=imread(fileName);
		frameCount++;
	}
};

int calMOT(const char* path=NULL)
{
	// dtv9_clip_3700_7500:	0.95, 0.85
	// Honk-Kong:			0.55, 0.90
	// Wester_LLTV_VELA:	0.85, 2.00
	ResultParser gt(GT_XML_FILE,1.0);
	ResultParser hp(HP_XML_FILE,    1.0,                   	0.55,                    0.9);//you may scale the result bounding box: w=w*r*w_r, h=h*r*h_r
	//                              [ratio]					[width_ratio]			[height_ratio]
	C_Mot mot(1.0);//1.0: IOU threshold     

	if (path==NULL)
	{
		while(!gt.isEnd() && !hp.isEnd())
			mot.dealWithDetection(gt.readNextFrame(),hp.readNextFrame());

		mot.getMOT(false);
		return 0;
	}

	SeqReader reader=SeqReader(string(path));
	Mat frame;
	namedWindow("multiTrack",CV_WINDOW_AUTOSIZE);

	reader.readImg(frame);
	if (frame.data==NULL)
	{
		cerr<<"fail to read sequence!"<<endl;
		return 0;
	}
	mot.dealWithDetection(gt.readNextFrame(),hp.readNextFrame());
	mot.paintFrame(frame);
	imshow("multiTrack",frame);
	waitKey(0);
	while(!gt.isEnd() && !hp.isEnd())
	{
		reader.readImg(frame);
		mot.dealWithDetection(gt.readNextFrame(),hp.readNextFrame());
		mot.paintFrame(frame);
		imshow("multiTrack",frame);
		waitKey(0);
	}
	mot.getMOT(true);
	getchar();
	return 0;
}

int main(int, char**)
{
	//calMOT(SEQUENCE_PATH);
	calMOT();
	
	return 0;
}
