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
/*!\file    projects/rc_unimog_control_group2/object_detection/mLineDetection.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-06
 *
 * \brief Contains mLineDetection
 *
 * \b mLineDetection
 *
 * detecting and calculating distance to the lines
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__mLineDetection_h__
#define __projects__rc_unimog_control_group2__object_detection__mLineDetection_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/coviroa/tImage.h"
#include "rrlib/si_units/si_units.h"
#include <opencv2/core/core.hpp>

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

enum class Line
{
	left, middle, right, none
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * detecting and calculating distance to the lines
 */
class mLineDetection: public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::coviroa::tImage> in_image;

	tOutput<rrlib::coviroa::tImage> outside_line_img, inside_line_img, hough_img;
	tOutput<int> distance_to_center,to_in,to_out;

	tOutput<std::string> inside,outside;

	tParameter<int> inside_line_down_canny_threshold, inside_line_up_canny_threshold,
			inside_line_down_inRange_threshold, inside_line_up_inRangeThreshold, inside_line_Blur;

	tParameter<int> outside_line_down_canny_threshold, outside_line_up_canny_threshold,
	outside_line_down_inRange_threshold, outside_line_up_inRangeThreshold, outside_line_Blur;

	tParameter<int> hough_line_threshold,hough_line_length,hough_line_gap;

	tParameter<int> point;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mLineDetection(core::tFrameworkElement *parent, const std::string &name =
			"LineDetection");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mLineDetection();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	int in_line_in_range_u, in_line_in_range_d, in_line_canny_u, in_line_canny_d, in_line_blur;

	int out_line_in_range_u, out_line_in_range_d, out_line_canny_u, out_line_canny_d, out_line_blur;

	int hough_thresh,hough_lin_l,hough_lin_g;

	Line inside_line, outside_line;

	int distance_to_inside, distance_to_outside;

	int p;

	virtual void OnStaticParameterChange() override;

	virtual void OnParameterChange() override;

	virtual void Update() override;

	void LineDetect(const cv::Mat &src, cv::Mat &dst);
	void DetectOutsideLine(const cv::Mat &src, cv::Mat &dst);
	void DetectInsideLine(const cv::Mat &src);
	void DistanceToOutsideLine(const cv::Mat &src);
	void DistanceToInsideLine(const cv::Mat &src);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
