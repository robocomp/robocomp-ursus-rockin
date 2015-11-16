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
	fname_csv = "../files/csv.ext";
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{

	cout << "compute" << endl;

	vector<Mat> images;
	vector<int> labels;
	// Read in the data. This can fail if no valid
	// input filename is given.
	try
	{
		read_csv(fname_csv, images, labels);
	}
	catch (cv::Exception& e)
	{
		cerr << "Error opening file \"" << fname_csv << "\". Reason: " << e.msg << endl;
		// nothing more we can do
		exit(1);
	}

	if(images.size() <= 1)
	{
		string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
		CV_Error(CV_StsError, error_message);
	}

	Ptr<FaceRecognizer> model = createLBPHFaceRecognizer(1,8);

	//Entrena el sistema, creando el modelo y guardandolo

	model->train(images, labels);

	cout<<labels.size()<<endl;

	//Para salvar el modelo en un xml
	model->save("../files/faces.xml");

	//TODO: mirar esto. Como parar el timer
	timer.stop();

}

void SpecificWorker::read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator)
{
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file)
    {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line))
    {
        stringstream liness(line);
        getline(liness, path, separator);

        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty())
        {
            images.push_back(imread(path, CV_LOAD_IMAGE_GRAYSCALE));
            labels.push_back(atoi(classlabel.c_str()));
            cout<<path<<"-----------"<<atoi(classlabel.c_str())<<endl;
        }
    }
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};
