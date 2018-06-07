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
/*!\file    projects/rc_unimog_control_group2/object_detection/mStopSignDetection.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 * \brief Contains mStopSignDetection
 *
 * \b mStopSignDetection
 *
 * stop sign detection module
 *
 *
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__mStopSignDetection_h__
#define __projects__rc_unimog_control_group2__object_detection__mStopSignDetection_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/coviroa/tImage.h"
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <chrono>

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
namespace object_detection
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * stop sign detection module
 * a
 * a b c d
 */
class mStopSignDetection: public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::coviroa::tImage> in_image;
	tOutput<rrlib::coviroa::tImage> out_image;
	tOutput<bool> stop_detected;

	tOutput<rrlib::math::tVec2i> stop_center;

	tParameter<int> time_to_wait;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mStopSignDetection(core::tFrameworkElement *parent,
			const std::string &name = "StopSignDetection");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mStopSignDetection();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	std::string stop_cascade_name;
	cv::CascadeClassifier stop_cascade;
	bool detected;

	std::chrono::time_point<std::chrono::high_resolution_clock> current_time;

	std::chrono::milliseconds time_count;

	int waiting_time;

	bool real_stop_detected;

	rrlib::math::tVec2i center;

	bool detectSign(cv::Mat& input, cv::Mat& output);

	virtual void OnStaticParameterChange() override;

	virtual void OnParameterChange() override;

	virtual void Update() override;

	bool wait();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
