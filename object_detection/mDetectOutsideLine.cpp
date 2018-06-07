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
/*!\file    projects/rc_unimog_control_group2/object_detection/mDetectOutsideLine.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-09
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mDetectOutsideLine.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>
#include <cfloat>
#include <utility>
#include <algorithm>
//#include <iterator>

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

using namespace rrlib::math;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mDetectOutsideLine> cCREATE_ACTION_FOR_M_DETECTOUTSIDELINE(
		"DetectOutsideLine");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mDetectOutsideLine constructor
//----------------------------------------------------------------------
mDetectOutsideLine::mDetectOutsideLine(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), max_gap(50), in_range_down(180), in_range_up(
				255), blur_size(3), minimum_area_of_red_fields(10) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
}

//----------------------------------------------------------------------
// mDetectOutsideLine destructor
//----------------------------------------------------------------------
mDetectOutsideLine::~mDetectOutsideLine()
{
}

//----------------------------------------------------------------------
// mDetectOutsideLine OnStaticParameterChange
//----------------------------------------------------------------------
void mDetectOutsideLine::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mDetectOutsideLine OnParameterChange
//----------------------------------------------------------------------
void mDetectOutsideLine::OnParameterChange()
{
	if (blur_size.Get() % 2 == 1)
		blur = blur_size.Get();
	else
		blur = blur_size.Get() - 1;

	if (in_range_down.Get() < in_range_up.Get())
	{
		in_range_d = in_range_down.Get();
		in_range_u = in_range_up.Get();
	}

	m_gap = max_gap.Get();
	area = minimum_area_of_red_fields.Get();
}

//----------------------------------------------------------------------
// mDetectOutsideLine Update
//----------------------------------------------------------------------
void mDetectOutsideLine::Update()
{

	try
	{
	if (this->InputChanged())
	{


		data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_img =
				this->in_image.GetPointer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> red_fields_img_ptr =
				this->out_img_red_fields.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> red_fields_filtered_img_ptr =
				this->out_img_red_fields_filtered.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> outside_line_img_ptr =
				this->outside_lines.GetUnusedBuffer();
		data_ports::tPortDataPointer<std::vector<std::vector<rrlib::math::tVec2i>>> lines_vector_ptr = lines_vector.GetUnusedBuffer();


		cv::Mat in = rrlib::coviroa::AccessImageAsMat(*in_img);
		cv::Mat out1 = rrlib::coviroa::AccessImageAsMat(*red_fields_img_ptr);
		cv::Mat out2 = rrlib::coviroa::AccessImageAsMat(
				*red_fields_filtered_img_ptr);
		cv::Mat out3 = rrlib::coviroa::AccessImageAsMat(*outside_line_img_ptr);

		red_fields_img_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		red_fields_filtered_img_ptr->Resize(in_img->GetWidth(),
				in_img->GetHeight(), rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		outside_line_img_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);

//////////////////red color to binary image//////////////////
		cv::Mat YUV;
		cv::cvtColor(in, YUV, cv::COLOR_BGR2YUV);
		std::vector<cv::Mat> yuv_vec;
		cv::split(YUV, yuv_vec);
		cv::inRange(yuv_vec[2], in_range_d, in_range_u, YUV);
		//cv::GaussianBlur(YUV, temp,{ out_line_blur, out_line_blur }, CV_PI / 2, 1);
		cv::blur(YUV, out1,
		{ blur, blur });
		this->out_img_red_fields.Publish(red_fields_img_ptr);

//////////////////find and filter contours//////////////////
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(out1, contours, 0, 2, cv::Point(0, 0));

		if(contours.size()==0)
			return;


		std::vector<cv::Point> filtered_contours_position;
		cv::Mat temp = cv::Mat::zeros(in.size(), CV_8UC1);
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			if (abs(cv::contourArea(contours[i])) > area)
			{
				std::vector<cv::Point> polygon; ///////////////////////////////////
				cv::approxPolyDP(contours[i], polygon, 3, true);
				cv::Rect bound_rect = cv::boundingRect(polygon);

				cv::drawContours(temp, contours, i, cv::Scalar(255), 2);

				int mid_x, mid_y;

				mid_x = bound_rect.width/2+bound_rect.x;
				mid_y = bound_rect.height/2+bound_rect.y;

				filtered_contours_position.push_back(cv::Point(mid_x, mid_y));
			}
		}
		temp.copyTo(out2);
		this->out_img_red_fields_filtered.Publish(red_fields_filtered_img_ptr);


//////////////////connect contours//////////////////

		std::vector<std::vector<tVec2i>> lines_vec;
			lines_vec.push_back(std::vector<tVec2i>());


		int detected_line_counter = 0;

		while (filtered_contours_position.size() > 0)
		{
			int min_pos = 0;
			int min_y = 0;

			for (unsigned int i = 0; i < filtered_contours_position.size(); i++)
			{
				if (filtered_contours_position[i].y > min_y)
					min_pos = i;
			}

			lines_vec[detected_line_counter].push_back( tVec2i( filtered_contours_position[min_pos].x,filtered_contours_position[min_pos].y));

			filtered_contours_position.erase(
					filtered_contours_position.begin() + min_pos);

			int j = 0;

			while (filtered_contours_position.size() > 0)
			{
				cv::Point last_point(lines_vec[detected_line_counter][j].X(),lines_vec[detected_line_counter][j].Y());

				float current_gap = FLT_MAX;
				int index = -1;

				for (unsigned int i = 0; i < filtered_contours_position.size();
						i++)
				{
					cv::Point current_point = filtered_contours_position[i];
					float temp_gap = sqrt(
							pow(current_point.x - last_point.x, 2)
									+ pow(current_point.y - last_point.y, 2));

					if (current_gap > temp_gap)
					{
						current_gap = temp_gap;
						index = i;
					}
				}

				if (current_gap < m_gap)
				{
					lines_vec[detected_line_counter].push_back(
							tVec2i(filtered_contours_position[index].x,filtered_contours_position[index].y));
					filtered_contours_position.erase(
							filtered_contours_position.begin() + index);
					j++;

				}
				else
				{
					detected_line_counter++;
					lines_vec.push_back(std::vector<tVec2i>());
					break;
				}
			}
		}

		/*

		 for (unsigned int i = 1; i < filtered_contours_position.size(); i++)
		 {
		 cv::Point actual_point = filtered_contours_position[i];

		 cv::Point last_point = filtered_contours_position[i - 1];
		 float current_gap = sqrt(
		 pow(actual_point.x - last_point.x, 2)
		 + pow(actual_point.y - last_point.y, 2));

		 if (current_gap > m_gap)
		 {
		 lines_vec.push_back(std::vector<cv::Point>());
		 detected_line_counter++;
		 lines_vec[detected_line_counter].push_back(actual_point);
		 }
		 else
		 {
		 lines_vec[detected_line_counter].push_back(actual_point);
		 }
		 }

		 */
//////////////////erase contours//////////////////

		cv::Mat temp2 = cv::Mat::zeros(in.size(), CV_8UC1);
//
//		while(lines_vec.size()>2)
//		{
//			int index=-1;
//			unsigned int size=INT_MAX;
//			for(unsigned int i=0;i<lines_vec.size();i++)
//			{
//				if(lines_vec.at(i).size()<size)
//					index=i;
//			}
//			if(index==-1)
//				break;
//			else
//				lines_vec.erase(lines_vec.begin()+index);
//		}


//////////////////erase single contours//////////////////

		while(true)
		{
			bool erase=false;
			for(unsigned int i = 0; i < lines_vec.size(); i++)
			{
				if(lines_vec[i].size()<2)
				{
					lines_vec.erase(lines_vec.begin()+i);
					erase=true;
					break;
				}
			}
			if(!erase)
				break;
		}



//////////////////show result//////////////////
		for (unsigned int i = 0; i < lines_vec.size(); i++)
		{
			std::reverse(lines_vec[i].begin(),lines_vec[i].end());

			for (unsigned int j = 1; j < lines_vec[i].size(); j++)
			{
				line(temp2, {lines_vec[i][j - 1].X(),lines_vec[i][j - 1].Y()}, {lines_vec[i][j].X(),lines_vec[i][j].Y()},
						cv::Scalar(200),3);
			}
		}


		temp2.copyTo(out3);

		this->outside_lines.Publish(outside_line_img_ptr);

		*lines_vector_ptr=std::move(lines_vec);


		lines_vector_ptr.SetTimestamp(in_img.GetTimestamp());
		lines_vector.Publish(lines_vector_ptr);

	}
	}
	catch(...)
	{
		std::cout<<"Detect outside line crash"<<std::endl;
	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
