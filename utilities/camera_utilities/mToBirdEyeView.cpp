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
/*!\file    projects/rc_unimog_control_group2/util/camera_utilities/mToBirdEyeView.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mToBirdEyeView.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rrlib/coviroa/opencv_utils.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
namespace utilities
{
namespace camera_utilities
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mToBirdEyeView> cCREATE_ACTION_FOR_M_TOBIRDEYEVIEW(
		"ToBirdEyeView");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mToBirdEyeView constructor
//----------------------------------------------------------------------
mToBirdEyeView::mToBirdEyeView(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), down_treshold(0), down_width(2500), up_width(
				640), height(290) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{

}

//----------------------------------------------------------------------
// mToBirdEyeView destructor
//----------------------------------------------------------------------
mToBirdEyeView::~mToBirdEyeView()
{
}

//----------------------------------------------------------------------
// mToBirdEyeView OnStaticParameterChange
//----------------------------------------------------------------------
void mToBirdEyeView::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mToBirdEyeView OnParameterChange
//----------------------------------------------------------------------
void mToBirdEyeView::OnParameterChange()
{
	down_t = down_treshold.Get();
	down_w = down_width.Get();
	up_w = up_width.Get();
	h = height.Get();
}

//----------------------------------------------------------------------
// mToBirdEyeView Update
//----------------------------------------------------------------------
void mToBirdEyeView::Update()
{
	if (this->InputChanged())
	{
		{
			data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_img =
					this->in_image.GetPointer();
			data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_img =
					this->out_image.GetUnusedBuffer();

			cv::Mat in = rrlib::coviroa::AccessImageAsMat(*in_img);
			cv::Mat out = rrlib::coviroa::AccessImageAsMat(*out_img);

			out_img->Resize(in_img->GetWidth(), in_img->GetHeight(),
					in_img->GetImageFormat(), 0);

			cv::Point2f inputQuad[4];
			cv::Point2f outputQuad[4];

			cv::Point2f dl, dr, tl, tr;

			dl = cv::Point2f(640 / 2 - down_w / 2, 480 - down_t);
			dr = cv::Point2f(640 / 2 + down_w / 2, 480 - down_t);
			tl = cv::Point2f(640 / 2 - up_w / 2, 480 - down_t - h);
			tr = cv::Point2f(640 / 2 + up_w / 2, 480 - down_t - h);

			inputQuad[1] = tl; //Point2f(210, 285);
			inputQuad[2] = tr; //Point2f(395, 285);
			inputQuad[3] = dr; //Point2f(570, 430);
			inputQuad[0] = dl; //Point2f(70, 430);

			outputQuad[0] = cv::Point2f(0, 480);
			outputQuad[1] = cv::Point2f(0, 0);
			outputQuad[2] = cv::Point2f(640, 0);
			outputQuad[3] = cv::Point2f(640, 480);

			cv::Mat transform = cv::getPerspectiveTransform(inputQuad,
					outputQuad);

			if (in.cols > 0 and in.rows > 0)
				cv::warpPerspective(in, out, transform,
				{ 640, 480 });

			this->out_image.Publish(out_img);
		}
	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
