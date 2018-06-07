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
/*!\file    projects/rc_unimog_control_group2/object_detection/mStopSignDetection.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mStopSignDetection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
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

using namespace cv;
using namespace std;

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
runtime_construction::tStandardCreateModuleAction<mStopSignDetection> cCREATE_ACTION_FOR_M_STOPSIGNDETECTION(
		"StopSignDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

bool mStopSignDetection::detectSign(cv::Mat& input, cv::Mat& output)
{
	std::vector<Rect_<int>> stops;
	Mat frame_gray;

	input.copyTo(output);

	cvtColor(output, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	stop_cascade.detectMultiScale(frame_gray, stops, 1.1, 2,
			0 | CASCADE_SCALE_IMAGE, Size(60, 60));
	if (stops.size() == 0)
	{
		detected = false;
	}

	for (size_t i = 0; i < stops.size(); i++)
	{
		Point center(stops[i].x + stops[i].width / 2,
				stops[i].y + stops[i].height / 2);
		ellipse(output, center, Size(stops[i].width / 2, stops[i].height / 2),
				0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
		detected = true;
	}

	if(stops.size()>0)
	{
		center=rrlib::math::tVec2i(stops[0].x + stops[0].width / 2,
				stops[0].y + stops[0].height / 2);
		this->stop_center.Publish(center);
	}

	return detected;
}

bool mStopSignDetection::wait()
{

	bool ok = false;

	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast < std::chrono::milliseconds
			> (now - current_time);
	current_time = now;
	time_count = time_count + elapsed_time;
	if (time_count > std::chrono::milliseconds(waiting_time))
	{
		//std::cout << time_count.count() << std::endl;
		time_count = std::chrono::milliseconds(0);
		ok = true;
	}

	return ok;
}

//----------------------------------------------------------------------
// mStopSignDetection constructor
//----------------------------------------------------------------------
mStopSignDetection::mStopSignDetection(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), stop_detected(false), time_to_wait(500), stop_cascade_name(
				"resources/stopsign_classifier.xml"), stop_cascade(), time_count(
				0), real_stop_detected(false)
{
	if (!stop_cascade.load(stop_cascade_name))
	{
		FINROC_LOG_PRINTF(ERROR, "lut >> could not open '%s'!\n",
				stop_cascade_name.c_str());
	}
	current_time = std::chrono::high_resolution_clock::now();
}

//----------------------------------------------------------------------
// mStopSignDetection destructor
//----------------------------------------------------------------------
mStopSignDetection::~mStopSignDetection()
{
}

//----------------------------------------------------------------------
// mStopSignDetection OnStaticParameterChange
//----------------------------------------------------------------------
void mStopSignDetection::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mStopSignDetection OnParameterChange
//----------------------------------------------------------------------
void mStopSignDetection::OnParameterChange()
{
	waiting_time = time_to_wait.Get();
}

//----------------------------------------------------------------------
// mStopSignDetection Update
//----------------------------------------------------------------------
void mStopSignDetection::Update()
{
	data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_ptr =
			this->in_image.GetPointer();
	data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_ptr =
			this->out_image.GetUnusedBuffer();

	if (this->InputChanged())
	{
		cv::Mat in_img = rrlib::coviroa::AccessImageAsMat(*in_ptr);
		cv::Mat out_img = rrlib::coviroa::AccessImageAsMat(*out_ptr);

		detected = detectSign(in_img, out_img);

		if (detected and !real_stop_detected)
		{
			if (wait())
				real_stop_detected = true;
		}
		else if (!detected and !real_stop_detected)
		{
			current_time = std::chrono::high_resolution_clock::now();
			time_count = std::chrono::milliseconds(0);
		}
		else if (!detected and real_stop_detected)
		{
			if (wait())
				real_stop_detected = false;
		}
		else if (detected and real_stop_detected)
		{
			current_time = std::chrono::high_resolution_clock::now();
			time_count = std::chrono::milliseconds(0);
		}

		out_ptr->Resize(in_img.cols, in_img.rows,
				rrlib::coviroa::eIMAGE_FORMAT_RGB24, 0);

		this->out_image.Publish(out_ptr);
		this->stop_detected.Publish(real_stop_detected);

	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
