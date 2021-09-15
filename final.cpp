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

#include "/usr/local/include/ctello.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <System.h>
#include <Converter.h>

#include<string.h>
#include<math.h>
#include<fstream>
#include<vector>


using namespace cv;
using ctello::Tello;
cv::Mat angles_mat;


const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

Mat frame;
bool start = false;
bool notStop = true;

Tello tello{};
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

//void find_exit(string pointData.sv)
//{

//}
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

    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()));
    VideoCapture cap{TELLO_STREAM_URL,CAP_FFMPEG};
    tello.SendCommand("takeoff");
    while (!(tello.ReceiveResponse()));

    while(true) {
        cap >> frame;
        usleep(100);
    }

}

void spin()
{
    sleep(1);
    while(!start){
        //help orb slam init a map
        tello.SendCommand("up 20");
        while (!(tello.ReceiveResponse()));
        sleep(0.5);
        tello.SendCommand("down 20");
        while (!(tello.ReceiveResponse()));
        sleep(0.5);

        sleep(1);

    }
    for(int i = 18; i-- >0;){
        {

            tello.SendCommand("up 20");
            while (!(tello.ReceiveResponse()));
            sleep(0.5);
            tello.SendCommand("down 20");
            while (!(tello.ReceiveResponse()));
            sleep(0.5);
            tello.SendCommand("cw 20");
            while (!(tello.ReceiveResponse())) ;


            sleep(2);
        }
    }
    notStop = false;
}

double radius(double x, double y)
{
    return sqrt(x*x+y*y);
}
double dot_product(pair<double,double> p1, pair<double,double> p2){

    return p1.first*p2.first + p1.second*p2.second;
}

double det(pair<double, double> p1, pair<double, double> p2){

    return p1.first*p2.second - p1.second*p2.first;
}

double _angle(pair<double, double> p1, pair<double, double> p2)
{
    double thing = dot_product(p1, p2) / (radius(p1.first, p1.second)*radius(p2.first, p2.second));

    //cout << thing << "\t point: " << p1.first << "," <<p1.second << " and " << p2.first << "," <<p2.second;


    if(thing >= 1) return 0;
    if(thing <= -1) return 180;

    else return acos(thing) * 180 / M_PI;
}

pair<double, double> simple_tokenizer(string s)
{
    vector<double> out;
    size_t start1;
    size_t end = 0;

    while ((start1 = s.find_first_not_of(',', end)) != string::npos)
    {
        end = s.find(',', start1);
        out.push_back(stod(s.substr(start1, end - start1)));
    }
    return make_pair(out[0],out[1]);
}

vector< pair<double, double>> points;

double new_angle(Mat check, pair<double, double> p1 ){//should return angle from every point , p1 need to be the target point from dir func

    Mat r=check(Rect(0,0,3,3)).clone();
    Mat t=check(Rect(3,0,1,3)).clone();
    Mat new_look= r.row(2).t();

    pair<double, double> look(new_look.at<float>(0), new_look.at<float>(2)); //look is the vector we look at

    Mat new_location=(-r.t())*(t);
    pair<double, double> location(new_location.at<float>(0), new_location.at<float>(2));

    pair<double, double> direction(p1.first-location.first,p1.second-location.second);//the vector we want to go
    double new_angle  =_angle(look,direction);
    if(det(look, direction) < 0){
        return -new_angle;
    }
    else{
        return new_angle;
    }
}

pair<double, double> _dir(){

    ifstream fin ("/tmp/pointData.csv");
    string line;
    int slice_size = 8;
    // Open an existing file

    while(getline(fin, line)){
        pair<double, double> ret = simple_tokenizer(line);
        if (radius(ret.first, ret.second) <= 1e-5) continue;

        points.push_back(ret);
    }
    fin.close();


    vector<pair<double, double>> slices[int(360/slice_size) + 1];

    for(pair<double, double>& p : points){

        double angle = _angle(p, points[0]);

        if (det(points[0], p) > 0){ //p is left to first_point
            angle = 360 - angle;
        }

        slices[int(angle/slice_size)].push_back(p);
    }

    double max = -1;
    int max_index = -1;
    int i = 0;

    for(vector<pair<double, double>>& slice : slices){
        int count = slice.size();
        double sum = 0;

        for(pair<double, double> p : slice){
            sum += radius(p.first, p.second);
        }

        double avg = count ? sum / count : -1;
        if(avg > max) {
            max = avg;
            max_index = i;
        }
        ++i;
    }

    int size = slices[max_index].size();
//
//    sort(slices[max_index].begin(), slices[max_index].end(),
//         [](const pair<double, double> & a, const pair<double, double> & b) -> bool
//         {
//             return radius(a.first, a.second) > radius(b.first, b.second);
//         });
//    return slices[max_index][size/2];

    pair<double, double> toGo = *max_element(slices[max_index].begin(), slices[max_index].end(),
                                             [](const pair<double, double> & a, const pair<double, double> & b) -> bool
                                             {
                                                 return radius(a.first, a.second) < radius(b.first, b.second);
                                             });

    return toGo;

}

void MoveToAngle(int right_angle){

    //tello cant rotate angles smaller then 20
    if(right_angle>=0)
    {
        if(right_angle < 20){
            tello.SendCommand("ccw 20");
            while (!(tello.ReceiveResponse())) {
                sleep(0.1);
            }
            right_angle += 20;
        }

        while(right_angle > 40)
        {
            tello.SendCommand("cw 20");
            while (!(tello.ReceiveResponse())) {
                sleep(0.1);
            }
            right_angle = right_angle - 20;
        }

        tello.SendCommand("cw" + to_string(right_angle));
        while (!(tello.ReceiveResponse())) {
            sleep(0.1);
        }

    }
    else
    {
        right_angle *= -1;

        if(right_angle < 20){
            tello.SendCommand("cw 20");
            while (!(tello.ReceiveResponse())) {
                sleep(0.1);
            }
            right_angle += 20;
        }

        while(right_angle > 40)
        {
            tello.SendCommand("ccw 20");
            while (!(tello.ReceiveResponse())) {
                sleep(0.1);
            }
            right_angle = right_angle - 20;
        }

        tello.SendCommand("ccw"+to_string(right_angle));
        while (!(tello.ReceiveResponse())) {
            sleep(0.1);
        }
    }
    sleep(0.5);
}


int main(int argc, char **argv)
{

    Mat check;
    int images = 0;

    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    if (!tello.Bind())
    {
        return 0;
    }


    thread t1 (_capture);
    thread t2 (spin);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);


    // Main loop
    while(notStop)
    {
        // Read image from camera

        if(!frame.empty()){
            // Pass the image to the SLAM system
            check = SLAM.TrackMonocular(frame, images++);
            if(!start && !check.empty()) start = true;
        }


    }

    cout<<"before map\n";
    SaveMap(SLAM);
    cout<<"after map\n";
    sleep(1);

    //new fix angle
    pair<double, double> goal= _dir();

    int right_angle = new_angle(check,goal);


    MoveToAngle(right_angle);


    //new fix
    double distance_left = radius(goal.first, goal.second);

    for(int i = distance_left; i > 0; i-=50) {
        tello.SendCommand("forward 50");
        while (!(tello.ReceiveResponse()));
        if (!frame.empty()) {
            check = SLAM.TrackMonocular(frame, images++);
        }

        right_angle = new_angle(check, goal);

        MoveToAngle(right_angle);
    }



    notStop = false;
    tello.SendCommand("streamoff");
    while (!(tello.ReceiveResponse()));
    tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));
    // Stop all threads

    destroyAllWindows();
    SLAM.Shutdown();

    return 0;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
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
