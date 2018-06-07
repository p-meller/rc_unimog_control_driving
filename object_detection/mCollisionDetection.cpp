//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/rc_unimog_control_group2/object_detection/mCollisionDetection.cpp
 *
 * \author  Lukasz Owsianny
 *
 * \date    2018-01-24
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mCollisionDetection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cfloat>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <chrono>
#include <algorithm>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/coviroa/opencv_utils.h"
//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::signal_filters;

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_group2
{
namespace object_detection
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mCollisionDetection> cCREATE_ACTION_FOR_M_COLLISIONDETECTION("CollisionDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mCollisionDetection constructor
//----------------------------------------------------------------------
mCollisionDetection::mCollisionDetection(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), mirror_flip(false),time_to_wait(500),lowThreshold(170),
  highThreshold(250), pixelsForPylons(10),multiplyForArea(1), stop_cascade_name("sources/cpp/projects/Final/resources/stopsign_classifier.xml"),time_count(0)

{
	if (!stop_cascade.load(stop_cascade_name))
		{
			FINROC_LOG_PRINTF(ERROR, "lut >> could not open '%s'!\n",
					stop_cascade_name.c_str());
		}
	current_time = std::chrono::high_resolution_clock::now();
}

//----------------------------------------------------------------------
// mCollisionDetection destructor
//----------------------------------------------------------------------
mCollisionDetection::~mCollisionDetection()
{}

//----------------------------------------------------------------------
// mCollisionDetection OnStaticParameterChange
//----------------------------------------------------------------------
void mCollisionDetection::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mCollisionDetection OnParameterChange
//----------------------------------------------------------------------
void mCollisionDetection::OnParameterChange()
{
	m = mirror_flip.Get();
	wait =time_to_wait.Get();
	iLowThreshold=lowThreshold.Get();
	iHighThreshold=highThreshold.Get();
	iPixelsToPylons=pixelsForPylons.Get();
	dmultiplyForArea=multiplyForArea.Get();
}

bool mCollisionDetection::PylonsDetection(cv::Mat src,cv::Mat color_image)
{
	static bool ifDetected=false;
	Mat gray;
	data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_contour_ptr = this->out_contour_image.GetUnusedBuffer();
	Mat out_contour_img = rrlib::coviroa::AccessImageAsMat(*out_contour_ptr);
	data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_pyllon_ptr = this->out_pyllon_image.GetUnusedBuffer();
		Mat out_pyllon_img = rrlib::coviroa::AccessImageAsMat(*out_pyllon_ptr);
	std::vector<std::vector<Point>> contours;
	std::vector<Point> approx;
	std::vector<Point> srodek;//,dobreSrodki,najlepszeSrodki;
	std::vector<std::vector<Point>> wektorGlowny;
	std::vector<Point> dobreSrodki;
	dilate(src,src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(src,src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(src,src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//GaussianBlur(gray, gray,{ 31, 31 }, CV_PI / 2, 1);
	int middleX, middleY;
	std::vector<double> rectArea;
	std::vector<double> dobrerectArea;
	std::vector<std::vector<double>> wektorGlownyDobreRectArea;
	findContours(src,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
	for(unsigned int i=0;i<contours.size();i++)
	{

		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
		drawContours(src,contours,i,Scalar(255,255,255),2);
		Rect rect=boundingRect(approx);
		middleX = (rect.x+rect.width/2);
		middleY=(rect.y+rect.width/2);
		srodek.push_back({middleX,middleY});
		rectArea.push_back(rect.width*rect.height);

	}
	/*if(rectArea.size()!=0)
	{
		auto it = rectArea.begin();
		while(it != rectArea.end())
		{
			std::cout<<(*it)<<"  ";
			it++;
		}
		std::cout<<std::endl;
	}*/
	for(unsigned int i=0;i<srodek.size();i++)
	{

		for(unsigned int j=i+1;j<srodek.size();j++)
		{
		if((srodek[i].x-iPixelsToPylons<srodek[j].x)&&(srodek[i].x+iPixelsToPylons>srodek[j].x))
		{
			dobreSrodki.push_back(srodek[i]);
			dobreSrodki.push_back(srodek[j]);
			dobrerectArea.push_back(rectArea[i]);
			dobrerectArea.push_back(rectArea[j]);
			if(dobreSrodki.size()>2)
			{
			for(unsigned int x=0;x<dobreSrodki.size();x++)
			{
				for(unsigned int y=x+1;y<dobreSrodki.size();y++)
				{
					if(dobreSrodki[x].y==dobreSrodki[y].y)
					{
						dobreSrodki.erase(dobreSrodki.begin()+y);
						dobrerectArea.erase(dobrerectArea.begin()+y);
						//--y;
						break;
					}
				}
			}
			}
		}
		}
		wektorGlowny.push_back(dobreSrodki);
		wektorGlownyDobreRectArea.push_back(dobrerectArea);
	}
	for(unsigned int i=0;i<wektorGlowny.size();i++)
	{
		if(wektorGlowny[i].size()==3)
		{
			circle(color_image,(wektorGlowny[i][0]),30,Scalar(255, 0, 255),1,8,0);
//std::cout<<"Znalazlem Twoje paliki na wysokosci "<<wektorGlowny[i][0].y<<" i szerokosci "
					//<<wektorGlowny[i][0].x<<std::endl;

			if(wektorGlownyDobreRectArea[i].size()==3)
			{
				std::sort (wektorGlownyDobreRectArea[i].begin(),wektorGlownyDobreRectArea[i].end());
				//for(auto it=wektorGlownyDobreRectArea[i].begin(); it!=wektorGlownyDobreRectArea[i].end();it++)
					//std::cout<<*it<<std::endl;

				if(wektorGlownyDobreRectArea[i][2]>(dmultiplyForArea*wektorGlownyDobreRectArea[i][1]))
				{
					std::cout<<"x palika:"<<wektorGlowny[i][0].x<<"y palika:"<<wektorGlowny[i][0].y<<std::endl;
					ifDetected=true;
					/*for(auto it=wektorGlownyDobreRectArea[i].begin();it!=wektorGlownyDobreRectArea.end();++it)
					{
						std::cout<<"Pole wynosi:"<<*it<<std::endl;
					}*/
				}
			}
			else
				ifDetected=false;
		}
		else
			ifDetected=false;
	}
	bool wyjsciowa=false;
	if(ifDetected)
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed_time = std::chrono::duration_cast
				< std::chrono::milliseconds > (now - current_time);
		current_time = now;
		time_count = time_count + elapsed_time;
		if (time_count > std::chrono::milliseconds(wait))
		{
			//std::cout<<time_count.count()<<std::endl;
			time_count = std::chrono::milliseconds(0);
			wyjsciowa = true;
		}
		else
			wyjsciowa=false;
	}

	out_contour_ptr->Resize(src.cols,src.rows,rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);

	src.copyTo(out_contour_img);
		this->out_contour_image.Publish(out_contour_ptr);
		out_pyllon_ptr->Resize(color_image.cols,color_image.rows,rrlib::coviroa::eIMAGE_FORMAT_RGB24, 0);
		color_image.copyTo(out_pyllon_img);
		this->out_pyllon_image.Publish(out_pyllon_ptr);
		return wyjsciowa;
}
/*void mCollisionDetection::PylonsDetection2(cv::Mat src)
{
	data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_pyllon_ptr = this->out_pyllon_image.GetUnusedBuffer();
		Mat out_pyllon_img = rrlib::coviroa::AccessImageAsMat(*out_pyllon_ptr);
	Mat mask_white;
	inRange(src,Scalar(240,240,240),Scalar(255,255,255),mask_white);
	Mat mask_orange;
	inRange(src,Scalar(20,30,180),Scalar(255,80,40),mask_orange);
	Mat mask_combined=mask_white | mask_orange;
	out_pyllon_ptr->Resize(src.cols,src.rows,rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
			src.copyTo(out_pyllon_img);
			this->out_pyllon_image.Publish(out_pyllon_ptr);

}*/




void mCollisionDetection::DistanceStopCalculation(Point middle_of_sign)
{
	double distance;
	double pixels=(double)middle_of_sign.y;
	//wczesniejsza funkcja
	//distance=((67505951.8-93952.655*pixels)/(1-1363.80303315*pixels+36.90129368*pixels*pixels));
	//pozniejsza
	distance=(22469622000000-34535611600*pixels)/(1-196457913*pixels+9382385.55*pixels*pixels);
	distance_to_stop.Publish(distance);
}
Mat mCollisionDetection::ColorBlobDetection(const cv::Mat &src, Point middle)
{



				Mat imgThresholded, imgDistance,imgYUV;
				data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_ptr = this->out_image.GetUnusedBuffer();
				Mat out_img = rrlib::coviroa::AccessImageAsMat(*out_ptr);
				cvtColor(src, imgYUV, COLOR_BGR2YUV);
				vector<Mat> yuv_v;
				split(imgYUV,yuv_v);
				//std::cout<<roi.x<<std::endl;
				inRange(yuv_v[2],iLowThreshold,iHighThreshold, imgThresholded); //180
				//GaussianBlur(imgThresholded, imgThresholded,{ 31, 31 }, CV_PI / 2, 1);
				//inRange(imgHSV, Scalar(0, 88, 214), Scalar(13, 255, 255), imgThresholded);
				//morphological opening (remove small objects from the foreground)
				erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

				//morphological closing (fill small holes in the foreground)
				//dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				out_ptr->Resize(imgThresholded.cols, imgThresholded.rows,rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
				imgThresholded.copyTo(out_img);
//std::cout<<"Ludzi jest duzo"<<std::endl;
				this->out_image.Publish(out_ptr);
				return imgThresholded;
}
bool mCollisionDetection::PylonsOntheRoad(rrlib::si_units::tLength<double> US, bool pylon)
{
	rrlib::si_units::tLength<double> value=1;
	if(US<value&&pylon==true)
	{
		//std::cout<<"Uwaga palik!";
		//stop_and_pylons_detected.Publish(true);
		return true;
	}
	else
	{
		//std::cout<<"Pusto";
		//stop_and_pylons_detected.Publish(false);
		return false;
	}
}
void mCollisionDetection::CameraDistance()
{
		Mat mirror,imgBirdView,imgThresholded,imgDistance,imgStop;
		Point object_middle;
		bool if_pylons;
//pobranie obrazu z rgb image, potem color blob detection, widok z gory i obliczenie odleglosci do najblizszego!
	data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in = this->input_rgb_image.GetPointer();
	rrlib::si_units::tLength<double> us_sensors_measured_distance;
	us_sensors_measured_distance=USSensors();

	Mat in_img = rrlib::coviroa::AccessImageAsMat(*in);

		//flip(in_img, mirror, 1);
		rrlib::math::tVec2i sr;
		sr=this->stop_center.Get();
		object_middle=Point(sr.X(),sr.Y());
		imgThresholded=ColorBlobDetection(in_img,object_middle);
		if_pylons=PylonsDetection(imgThresholded,in_img);
		stop_and_pylons_detected.Publish(PylonsOntheRoad(us_sensors_measured_distance,if_pylons));
		Rect roi(object_middle.x-50,object_middle.x+50,100,100);

//		for(int i=object_middle.x-50;i<object_middle.x+50;i++)
//		{
//			for(int j=object_middle.y-50;j<object_middle.y+50;j++)
//			{
//				if(imgThresholded.at<unsigned char>(j,i)==255)
//				{
////std::cout<<"Znalazlem bialy pixel tam gdzie chciales szefie!"<<std::endl;
//					//std::cout<<"x is "<<object_middle.x<<"and y is "<<object_middle.y<<std::endl;
//					if(object_middle.x!=0)
//						DistanceStopCalculation(object_middle);
//				}
//
//			}
//		}
		findNonZero(imgThresholded, imgDistance);
		float min_distance=FLT_MIN;
					for(unsigned int i=0;i<imgDistance.total();i++)
					{
						if(min_distance<imgDistance.at<Point>(i).y)
							min_distance=imgDistance.at<Point>(i).y;
					}

					distance.Publish(min_distance);

}
void mCollisionDetection::IRSensors()
{

}

rrlib::si_units::tLength<double> mCollisionDetection::USSensors()
{
	//measured value 1 is around 45/50cm in real distance
	//rrlib::si_units::tLength<double> value = 1;
	rrlib::si_units::tLength<double> mean=0;
	us_data.push_front(input_us_front.Get());

	if(us_data.size()>50)
		us_data.pop_back();
	auto i=0;
	for(auto it=us_data.begin();it!=us_data.end();it++)
	{
		mean=mean+*it;
		i++;

	}

	mean=mean/i;
	output_us_front.Publish(mean);
	return mean;



}
//----------------------------------------------------------------------
// mCollisionDetection Update
//----------------------------------------------------------------------
void mCollisionDetection::Update()
{
try
{
if(this->input_rgb_image.HasChanged())
{

	CameraDistance();

}
}
catch(...)
{
	std::cout<<"collision detection crash"<<std::endl;
}
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
