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
/*!\file    projects/rc_unimog_control_group2/object_detection/mLineDetection.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mLineDetection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <vector>

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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mLineDetection> cCREATE_ACTION_FOR_M_LINEDETECTION(
		"LineDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void mLineDetection::LineDetect(const cv::Mat &src, cv::Mat &dst)
{

	cv::Mat temp;
	cv::Mat temp2;

	cv::cvtColor(src, temp, cv::COLOR_BGR2HLS);
	std::vector<cv::Mat> hls_vec;
	cv::split(temp, hls_vec);

	cv::inRange(hls_vec[1], in_line_in_range_d, in_line_in_range_u, temp);

	//GaussianBlur(temp, temp2,{ in_line_blur, in_line_blur }, M_PI / 2, 1);
	cv::blur(temp,temp2,{in_line_blur,in_line_blur});

	Canny(temp2, dst, in_line_canny_d, in_line_canny_u);

}

void mLineDetection::DetectOutsideLine(const cv::Mat &src, cv::Mat &dst)
{
	cv::Mat YUV, temp;
	cv::cvtColor(src, YUV, cv::COLOR_BGR2YUV);
	std::vector<cv::Mat> yuv_vec;
	cv::split(YUV, yuv_vec);
	cv::inRange(yuv_vec[2], out_line_in_range_d, out_line_in_range_u, YUV);
	//cv::GaussianBlur(YUV, temp,{ out_line_blur, out_line_blur }, CV_PI / 2, 1);
	cv::blur(YUV,temp,{out_line_blur,out_line_blur});
	//cv::Canny(temp, YUV, out_line_canny_d, out_line_canny_u);



	YUV.copyTo(dst);

	int left = 0;
	int right = 0;

	for (int i = 0; i <= src.rows / 2; i++)
	{
		for (int j = 0; j <= src.cols / 2; j++)
		{
			if (YUV.at<unsigned char>(src.rows - i, src.cols / 2 - j) != 0)
			{
				left++;
				break;
			}

		}
		for (int j = 0; j <= src.cols / 2; j++)
		{
			if (YUV.at<unsigned char>(src.rows - i, (src.cols / 2 - 1) + j)
					!= 0)
			{
				right++;
				break;
			}

		}

		if (left > (right + 15) or right > (left + 15))
		{
			if (left > right)
				outside_line = Line::left;
			else
				outside_line = Line::right;

			break;

		}
		else
		{
			outside_line = Line::none;
		}
	}
}

void mLineDetection::DetectInsideLine(const cv::Mat &src)
{

	if (outside_line == Line::left or outside_line == Line::right)
	{

		int detected_points = 0;

		for (int i = 0; i <= src.rows / 2; i--)
		{
			for (int j = 0; j <= src.cols / 2; j++)
			{
				int iter = j;
				if (outside_line == Line::right)
					iter = -j;

				if (src.cols / 2 + iter < src.cols
						and src.at<unsigned char>(src.rows - i,
								src.cols / 2 + iter) != 0)
				{
					detected_points++;
					break;
				}
			}

			if (detected_points > 15)
			{
				inside_line = Line::middle;
				break;
			}
		}
	}
	else
		inside_line = Line::none;

}

void mLineDetection::DistanceToOutsideLine(const cv::Mat &src)
{

	if (outside_line == Line::left or outside_line == Line::right)
	{
		int distance = 0;

		for (int i = 0; i <= src.cols / 2; i++)
		{
			int iter = i;
			if (outside_line == Line::left)
				iter = -i;

			if (src.cols / 2 + iter < src.cols
					and src.at<unsigned char>(src.rows - p,
							src.cols / 2 + iter) != 0)
			{
				break;
			}
			else
			{
				distance++;
			}
		}

		distance_to_outside = distance;

		this->to_out.Publish(distance);
	}
}

void mLineDetection::DistanceToInsideLine(const cv::Mat &src)
{

	if (inside_line == Line::middle)
	{

		int distance = 0;

		for (int i = 0; i <= src.cols / 2; i++)
		{
			int iter = i;
			if (outside_line == Line::right)
				iter = -i;

			if (src.cols / 2 + iter < src.cols
					and src.at<unsigned char>(src.rows - p,
							src.cols / 2 + iter) != 0)
			{
				break;
			}
			else
			{
				distance++;
			}
		}
		distance_to_inside = distance;

		this->to_in.Publish(distance);
	}

}

//----------------------------------------------------------------------
// mLineDetection constructor
//----------------------------------------------------------------------
mLineDetection::mLineDetection(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), inside_line_down_canny_threshold(220), inside_line_up_canny_threshold(
				350), inside_line_down_inRange_threshold(180), inside_line_up_inRangeThreshold(
				255), inside_line_Blur(31), outside_line_down_canny_threshold(
				220), outside_line_up_canny_threshold(350), outside_line_down_inRange_threshold(
				180), outside_line_up_inRangeThreshold(255), outside_line_Blur(
				31), hough_line_threshold(3), hough_line_length(10), hough_line_gap(
				50), point(20), inside_line(Line::none), outside_line(
				Line::none) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
}

//----------------------------------------------------------------------
// mLineDetection destructor
//----------------------------------------------------------------------
mLineDetection::~mLineDetection()
{
}

//----------------------------------------------------------------------
// mLineDetection OnStaticParameterChange
//----------------------------------------------------------------------
void mLineDetection::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mLineDetection OnParameterChange
//----------------------------------------------------------------------
void mLineDetection::OnParameterChange()
{
	if (inside_line_Blur.Get() % 2 == 1)
		in_line_blur = inside_line_Blur.Get();
	else
		in_line_blur = inside_line_Blur.Get() - 1;

	if (inside_line_down_canny_threshold.Get()
			< inside_line_up_canny_threshold.Get())
	{
		in_line_canny_d = inside_line_down_canny_threshold.Get();
		in_line_canny_u = inside_line_up_canny_threshold.Get();
	}
	if (inside_line_down_inRange_threshold.Get()
			< inside_line_up_inRangeThreshold.Get())
	{
		in_line_in_range_d = inside_line_down_inRange_threshold.Get();
		in_line_in_range_u = inside_line_up_inRangeThreshold.Get();
	}

	if (outside_line_Blur.Get() % 2 == 1)
		out_line_blur = outside_line_Blur.Get();
	else
		out_line_blur = outside_line_Blur.Get() - 1;

	if (outside_line_down_canny_threshold.Get()
			< outside_line_up_canny_threshold.Get())
	{
		out_line_canny_d = outside_line_down_canny_threshold.Get();
		out_line_canny_u = outside_line_up_canny_threshold.Get();
	}
	if (outside_line_down_inRange_threshold.Get()
			< outside_line_up_inRangeThreshold.Get())
	{
		out_line_in_range_d = outside_line_down_inRange_threshold.Get();
		out_line_in_range_u = outside_line_up_inRangeThreshold.Get();
	}

	hough_thresh = hough_line_threshold.Get();
	hough_lin_l = hough_line_length.Get();
	hough_lin_g = hough_line_gap.Get();

	p = point.Get();
}

//----------------------------------------------------------------------
// mLineDetection Update
//----------------------------------------------------------------------
void mLineDetection::Update()
{
	if (this->InputChanged())
	{
		data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_img =
				this->in_image.GetPointer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> outside_line_img_ptr =
				this->outside_line_img.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> inside_line_img_ptr =
				this->inside_line_img.GetUnusedBuffer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> hough_img_ptr =
				this->hough_img.GetUnusedBuffer();

		cv::Mat in = rrlib::coviroa::AccessImageAsMat(*in_img);
		cv::Mat temp_out1 = rrlib::coviroa::AccessImageAsMat(
				*outside_line_img_ptr);
		cv::Mat temp_out2 = rrlib::coviroa::AccessImageAsMat(
				*inside_line_img_ptr);
		cv::Mat temp_out3 = rrlib::coviroa::AccessImageAsMat(*hough_img_ptr);

		outside_line_img_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		inside_line_img_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);
		hough_img_ptr->Resize(in_img->GetWidth(), in_img->GetHeight(),
				rrlib::coviroa::eIMAGE_FORMAT_MONO8, 0);

		DetectOutsideLine(in, temp_out1);
		LineDetect(in, temp_out2);

		std::vector<cv::Vec4i> newlines;
		cv::HoughLinesP(temp_out2, newlines, 1, CV_PI / 180, hough_thresh,
				hough_lin_l, hough_lin_g);

		cv::Mat temp3(in.rows, in.cols, CV_8UC1, cv::Scalar(0));

		temp3.copyTo(temp_out3);

		for (size_t i = 0; i < newlines.size(); i++)
		{
			cv::Vec4i le = newlines[i];

			cv::line(temp_out3, cv::Point(le[0], le[1]),
					cv::Point(le[2], le[3]), cv::Scalar(255), 3,
					CV_AA);
		}

		DetectInsideLine(temp_out3);
		DistanceToInsideLine(temp_out3);
		DistanceToOutsideLine(temp_out3);

		this->outside_line_img.Publish(outside_line_img_ptr);
		this->inside_line_img.Publish(inside_line_img_ptr);
		this->hough_img.Publish(hough_img_ptr);
		this->distance_to_center.Publish(
				distance_to_inside - distance_to_outside);

		if (inside_line == Line::middle)
			this->inside.Publish("middle");
		else
			this->inside.Publish("none");

		if (outside_line == Line::left)
			this->outside.Publish("left");
		else if (outside_line == Line::right)
			this->outside.Publish("right");
		else
			this->inside.Publish("none");

	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
