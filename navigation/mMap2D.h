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
/*!\file    projects/rc_unimog_control_group2/navi/mMap2D.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-11
 *
 * \brief Contains mMap2D
 *
 * \b mMap2D
 *
 * visualization of objects and path calculation
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mMap2D_h__
#define __projects__rc_unimog_control_group2__navi__mMap2D_h__

#include "plugins/structure/tModule.h"
#include <vector>

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/canvas/tCanvas2D.h"

#include <vector>

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
namespace navigation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * visualization of objects and path calculation
 */
class mMap2D: public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<std::vector<rrlib::math::tVec2i>> middle_line_input;
	tInput<std::vector<std::vector<rrlib::math::tVec2i>>>outside_line_input;

	tOutput<float> distance_to_left_lane,distance_to_right_lane;

	tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> visualization;

	tParameter<float> max_gap;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mMap2D(core::tFrameworkElement *parent, const std::string &name = "Map2D");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mMap2D();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	std::vector<rrlib::math::tVec2i> outside_left_line;
	std::vector<rrlib::math::tVec2i> outside_right_line;
	std::vector<rrlib::math::tVec2i> middle_line;

	int distance_to_middle_line,distance_to_left_line,distance_to_right_line;

	void outside_line_filter();
	void inside_line_filter();

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

#endif
