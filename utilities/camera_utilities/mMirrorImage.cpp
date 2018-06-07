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
/*!\file    projects/rc_unimog_control_group2/utilities/camera_utilities/mMirrorImage.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mMirrorImage.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


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
runtime_construction::tStandardCreateModuleAction<mMirrorImage> cCREATE_ACTION_FOR_M_MIRRORIMAGE(
		"MirrorImage");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mMirrorImage constructor
//----------------------------------------------------------------------
mMirrorImage::mMirrorImage(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{
}

//----------------------------------------------------------------------
// mMirrorImage destructor
//----------------------------------------------------------------------
mMirrorImage::~mMirrorImage()
{
}

//----------------------------------------------------------------------
// mMirrorImage OnStaticParameterChange
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mMirrorImage OnParameterChange
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mMirrorImage Update
//----------------------------------------------------------------------
void mMirrorImage::Update()
{
	if (this->InputChanged())
	{
		data_ports::tPortDataPointer<const rrlib::coviroa::tImage> in_img =
				this->in_image.GetPointer();
		data_ports::tPortDataPointer<rrlib::coviroa::tImage> out_img =
				this->out_image.GetUnusedBuffer();


		cv::Mat in = rrlib::coviroa::AccessImageAsMat(*in_img);
		cv::Mat out = rrlib::coviroa::AccessImageAsMat(*out_img);

		out_img->Resize(in_img->GetWidth(), in_img->GetHeight(),
				in_img->GetImageFormat(), 0);

		cv::flip(in, out, 1);

		this->out_image.Publish(out_img);
	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
