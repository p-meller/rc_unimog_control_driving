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
/*!\file    projects/rc_unimog_control_group2/util/camera_utilities/mToBirdEyeView.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 * \brief Contains mToBirdEyeView
 *
 * \b mToBirdEyeView
 *
 * changing perspective to bird eye view
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__util__camera_utilities__mToBirdEyeView_h__
#define __projects__rc_unimog_control_group2__util__camera_utilities__mToBirdEyeView_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/coviroa/tImage.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * changing perspective to bird eye view
 */
class mToBirdEyeView: public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::coviroa::tImage> in_image;
	tOutput<rrlib::coviroa::tImage> out_image;

	tParameter<float> down_treshold;
	tParameter<float> down_width;
	tParameter<float> up_width;
	tParameter<float> height;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mToBirdEyeView(core::tFrameworkElement *parent, const std::string &name =
			"ToBirdEyeView");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mToBirdEyeView();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	float down_t, down_w, up_w, h;

	virtual void OnStaticParameterChange() override;

	virtual void OnParameterChange() override;

	virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}

#endif
