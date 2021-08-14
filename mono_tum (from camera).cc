/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
//to RUN: ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>	
using namespace std;
using namespace cv;


VideoCapture cap(0);
Mat frame;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
                
void SaveMap(ORB_SLAM2::System& SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }

    pointData.close();
}

void _capture(){
	
	
	while(1){
		cap >> frame;
		sleep(0.01);
	}
	

}


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

  
    
    if(!cap.isOpened()){
    	cout << "Error opening video stream or file" << endl;
    	return -1;
  }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);


    // Main loop
    int i=1000;
    thread t1 ( _capture);
    cap >> frame;
    while((i-->0))
    {
        // Read image from camera

        if (frame.empty()) break;
        cap >> frame;




        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, 0.01);

    }

    cap.release();
    destroyAllWindows();

    SaveMap(SLAM);
    // Stop all threads
    SLAM.Shutdown();

   

    return 0;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
