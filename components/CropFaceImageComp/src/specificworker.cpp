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
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::compute( )
{
	mutex->lock();

	SensedFaceList sensedFaceList; //Para almacenar la lista ordenada de personas
	SensedFace sensedface;

	map<int,DetailedFace>::iterator iter;

	cout << "Numero de caras: " << faceList.size() << endl;

// 	static int cont=0;
	static int cont1=0;

	for (iter= faceList.begin(); iter!=faceList.end(); iter++)
	{		
		sensedface.leftEye.x=iter->second.leftEye.x-iter->second.left;
		sensedface.leftEye.y=iter->second.leftEye.y-iter->second.top;
		sensedface.rightEye.x=iter->second.rightEye.x-iter->second.left;
		sensedface.rightEye.y=iter->second.rightEye.y-iter->second.top;
		sensedface.faceImage=iter->second.faceImage;
		sensedface.id=iter->second.identifier;
		//miro si la cara está frontal
		if ((iter->second.yaw<=10 && iter->second.yaw>=-10) && (iter->second.pitch<=10&&iter->second.pitch>=-10) && (iter->second.roll<=10&&iter->second.roll>=-10))
		{
			//Calculo la imagen normalizada de la cara
			sensedface.faceImageNorm.height=100;
			sensedface.faceImageNorm.width=100;
			IplImage *faceIm, *temp;
			temp=cvCreateImage(cvSize(sensedface.faceImage.width, sensedface.faceImage.height), IPL_DEPTH_8U,4);
			for(int i=0; i< sensedface.faceImage.width*sensedface.faceImage.height*4;i++)
			{
				temp->imageData[i]=sensedface.faceImage.image[i];
			}

			faceIm=cvCreateImage(cvSize(sensedface.faceImage.width, sensedface.faceImage.height), IPL_DEPTH_8U,3);
			faceIm=temp;
			Mat faceImGray;
			Mat matFace(faceIm);
			faceImGray.create(sensedface.faceImage.width, sensedface.faceImage.height,CV_8UC1);
			Mat rotFace;
			Mat personImage;
			personImage.create(100,100,CV_8UC1);
			rotFace.create(100,100,CV_8UC1);

			cvtColor(matFace, faceImGray, CV_RGB2GRAY);
			
			/*
			char nameImageOriginal[200];
			sprintf(nameImageOriginal,"Original%d.bmp",cont++);
			imwrite(nameImageOriginal,faceImGray);
			*/
			
			normalizeFace(faceImGray, sensedface.rightEye, sensedface.leftEye, 0.2, 0.2, 100,100, rotFace);
			resize(rotFace, personImage, Size(100,100));

			sensedface.faceImageNorm.image.reserve(100*100);
			sensedface.faceImageNorm.image.resize(100*100);

			char nameImage[200];
			sprintf(nameImage,"Normalizada%d.bmp",cont1++);
			imwrite(nameImage,personImage);

		}
	}

	faceList.clear();

	mutex->unlock();

}

void SpecificWorker::normalizeFace(Mat image, Eye eye_right, Eye eye_left, float offset_x, float offset_y, int w, int h, Mat & normFace)
{
	  Mat rotImage(image.size(), CV_8UC1);

	  float offset_h = offset_x*w;
	  float offset_v = offset_y*h;

	  Eye eye_direction;
	  eye_direction.x=abs(eye_right.x - eye_left.x);
	  eye_direction.y=abs(eye_right.y - eye_left.y);

	  float rotation = atan2(eye_direction.y, eye_direction.x)*180./3.1416;

	  float dist = sqrt(pow(eye_direction.x, 2)+pow(eye_direction.y,2));

	  float reference = w - 2.0*offset_h;

	  float scale = dist/reference;

	  Point2f rotCenter;
	  rotCenter.x = eye_left.x;
	  rotCenter.y= eye_left.y;

	  Mat rotM = getRotationMatrix2D(rotCenter, rotation, 1);

	  warpAffine(image, rotImage, rotM, image.size());

	  float cropx = eye_left.x-scale*offset_h;
	  float cropy = eye_left.y-scale*offset_v;

	  float cropw=w*scale;
	  float croph=h*scale;

	  //qDebug()<<cropx<<cropy<<cropw<<croph<<scale<<offset_h<<offset_v;

	  cropx = (cropx<0)?0:cropx;
	  cropy = (cropy<0)?0:cropy;
	  cropw = ( (cropx+cropw)>=rotImage.cols)?rotImage.cols-1:cropw;
	  croph = ( (cropy+croph)>=rotImage.rows)?rotImage.rows-1:croph;

	  rotImage.rowRange(cropy,cropy + croph).colRange(cropx,cropx + cropw).copyTo(normFace);

	  rotImage.release();

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};


void SpecificWorker::newFaceAvailable(const DetailedFaceMap& face, long timestamp)
{
	mutex->lock();
	faceList = face;
	mutex->unlock();
}
