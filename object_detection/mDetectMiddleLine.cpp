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
/*!\file    projects/rc_unimog_control_group2/object_detection/mDetectMiddleLine.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-10
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mDetectMiddleLine.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>
#include <cfloat>
#include <climits>

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
runtime_construction::tStandardCreateModuleAction<mDetectMiddleLine> cCREATE_ACTION_FOR_M_DETECTMIDDLELINE(
		"DetectMiddleLine");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mDetectMiddleLine constructor
//----------------------------------------------------------------------
mDetectMiddleLine::mDetectMiddleLine(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), max_gap(50), in_range_down(240), in_range_up(
				255), blur_size(3), minimum_area_of_red_fields(10) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
}

//----------------------------------------------------------------------
// mDetectMiddleLine destructor
//----------------------------------------------------------------------
mDetectMiddleLine::~mDetectMiddleLine()
{
}

//----------------------------------------------------------------------
// mDetectMiddleLine OnStaticParameterChange
//----------------------------------------------------------------------
void mDetectMiddleLine::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mDetectMiddleLine OnParameterChange
//----------------------------------------------------------------------
void mDetectMiddleLine::OnParameterChange()
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

	if (minimum_area_of_red_fields.Get() < 1)
		area = 1;
	else
		area = minimum_area_of_red_fields.Get();
}

//----------------------------------------------------------------------
// mDetectMiddleLine Update
//----------------------------------------------------------------------
void mDetectMiddleLine::Update()
{
	if (this->InputChanged())
	{
		data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_img =
				this->in_image.GetPointer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> middle_in_range_ptr =
				this->middle_line_in_range.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> middle_line_contours_ptr =
				this->middle_line_contours.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> middle_line_ptr =
				this->middle_line.GetUnusedBuffer();
		data_ports::tPortDataPointer<std::vector<rrlib::math::tVec2i>> lines_vector_ptr =
				lines_vector.GetUnusedBuffer();

		cv::Mat in = rrlib::coviroa::AccessImageAsMat(*in_img);
		cv::Mat out1 = rrlib::coviroa::AccessImageAsMat(*middle_in_range_ptr);
		cv::Mat out2 = rrlib::coviroa::AccessImageAsMat(
				*middle_line_contours_ptr);
		cv::Mat out3 = rrlib::coviroa::AccessImageAsMat(*middle_line_ptr);

		middle_in_range_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		middle_line_contours_ptr->Resize(in_img->GetWidth(),
				in_img->GetHeight(), rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		middle_line_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);

		cv::inRange(in, cv::Scalar(in_range_d, in_range_d, in_range_d),
				cv::Scalar(in_range_u, in_range_u, in_range_u), out1);

		this->middle_line_in_range.Publish(middle_in_range_ptr);

		cv::Mat temp = cv::Mat::zeros(in.size(), CV_8UC1);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(out1, contours, 0, 2, cv::Point(0, 0));

		if (contours.size() == 0)
			return;

		for (unsigned int i = 0; i < contours.size(); i++)
		{
			if (abs(cv::contourArea(contours[i])) > area)
			{
				std::vector<cv::Point> polygon; ///////////////////////////////////
				cv::drawContours(temp, contours, i, cv::Scalar(255), CV_FILLED);

			}
		}

		temp.copyTo(out2);
		this->middle_line_contours.Publish(middle_line_contours_ptr);

		tVec2i p(-1, 0); //lowest point of line
		for (int i = in.rows - 1; i >= 0; i--)
		{

			for (int j = 0; j < in.cols / 2; j++)
			{
				if (temp.at<unsigned char>(i, in.cols / 2 + j) != 0)
				{
					int mid = 0;
					while (temp.at<unsigned char>(i, in.cols / 2 + j) != 0)
					{
						mid++;
						j++;
					}
					p = tVec2i(in.cols / 2 + j - mid / 2, i);
					break;
				}
				if (temp.at<unsigned char>(i, in.cols / 2 - 1 - j) != 0)
				{
					int mid = 0;
					while (temp.at<unsigned char>(i, in.cols / 2 - 1 - j) != 0)
					{
						mid++;
						j++;
					}
					p = tVec2i(in.cols / 2 - 1 - j + mid / 2, i);
					break;
				}
			}
			if (p.X() != -1)
			{
				break;
				//std::cout << p.Y() << "  v" << std::endl;

			}
		}

		cv::Mat temp2 = cv::Mat::zeros(in.size(), CV_8UC1);

		*lines_vector_ptr = std::vector<tVec2i>();
		if (p.X() != -1)
			lines_vector_ptr->push_back(p);


		tVec2i middle(p.X(), -1);
		for (int i = p.Y() - m_gap; i >= 0; i -= m_gap)
		{
			for (int j = m_gap * 1.5; j >= 0; j--)
			{
				if (temp.at<unsigned char>(i, middle.X() - j) == 255)
				{
					int mid = 0;
					while (temp.at<unsigned char>(i, middle.X() - j) == 255)
					{
						mid++;
						j--;
					}
					middle = tVec2i(middle.X() - j - mid / 2, i);
					break;
				}
				if (temp.at<unsigned char>(i, middle.X() + j) == 255)
				{
					int mid = 0;
					while (temp.at<unsigned char>(i, middle.X() + j) == 255)
					{
						mid++;
						j--;
					}
					middle = tVec2i(middle.X() + j + mid / 2, i);
					break;
				}
			}
			if (middle.Y() == -1)
				break;

			if (middle.X() != -1)
				lines_vector_ptr->push_back(middle);
			middle = tVec2i(middle.X(), -1);

		}


		for (unsigned int i = 1; i < lines_vector_ptr->size(); i++)
		{
			line(temp2,
					{ lines_vector_ptr->at(i - 1).X(), lines_vector_ptr->at(
							i - 1).Y() },
					{ lines_vector_ptr->at(i).X(), lines_vector_ptr->at(i).Y() },
					cv::Scalar(255), 3);
		}

		temp2.copyTo(out3);

		this->middle_line.Publish(middle_line_ptr);

		lines_vector_ptr.SetTimestamp(in_img.GetTimestamp());
		lines_vector.Publish(lines_vector_ptr);

	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
