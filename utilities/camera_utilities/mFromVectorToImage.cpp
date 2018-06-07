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
/*!\file    projects/rc_unimog_control_group2/utilities/camera_utilities/mFromVectorToImage.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-08
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mFromVectorToImage.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "rrlib/util/sStringUtils.h"

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
runtime_construction::tStandardCreateModuleAction<mFromVectorToImage> cCREATE_ACTION_FOR_M_FROMVECTORTOIMAGE(
		"FromVectorToImage");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mFromVectorToImage constructor
//----------------------------------------------------------------------
mFromVectorToImage::mFromVectorToImage(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{
}

//----------------------------------------------------------------------
// mFromVectorToImage destructor
//----------------------------------------------------------------------
mFromVectorToImage::~mFromVectorToImage()
{
}

//----------------------------------------------------------------------
// mFromVectorToImage OnStaticParameterChange
//----------------------------------------------------------------------
void mFromVectorToImage::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mFromVectorToImage OnParameterChange
//----------------------------------------------------------------------
void mFromVectorToImage::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mFromVectorToImage Update
//----------------------------------------------------------------------
void mFromVectorToImage::Update()
{
	if (this->InputChanged())
	{
		data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_imgs =
				this->in_images.GetPointer();

		if (in_imgs->size() != 0)
		{
			//rrlib::coviroa::tImage img(vec[0]);
			data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out =
					this->out_image.GetUnusedBuffer();
			rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy(
					(*in_imgs)[0], *img_out);
			img_out.SetTimestamp(in_imgs.GetTimestamp());
			this->out_image.Publish(img_out);
		}

		//data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> in_imgs = this->in_images.GetPointer();
		//data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
		//rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy((*in_imgs)[0], *img_out);

	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
