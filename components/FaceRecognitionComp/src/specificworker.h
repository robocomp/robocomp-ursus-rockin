/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	vectorClasses getClass();
	void  newFaceAvailable(const DetailedFaceMap& face, long timestamp);


public slots:
 	void compute();

private:
 	DetailedFaceMap faceList;
 	vectorClasses vClasses;
 	FILE *f;
 	Ptr<FaceRecognizer> model;

 	string fname_csv;
 	vector<Mat> images;
	vector<int> labels;
	vector<string> names;
	
	void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';');
	void readClassificationName(const string& filename, vector<string>& names, char separator = '/');
 	void normalizeFace(Mat image, Eye eye_right, Eye eye_left, float offset_x, float offset_y, int w, int h, Mat & normFace);
};

#endif
