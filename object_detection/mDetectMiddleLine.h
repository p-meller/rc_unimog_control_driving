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
/*!\file    projects/rc_unimog_control_group2/object_detection/mDetectMiddleLine.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-10
 *
 * \brief Contains mDetectMiddleLine
 *
 * \b mDetectMiddleLine
 *
 * this module can detect white line
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__mDetectMiddleLine_h__
#define __projects__rc_unimog_control_group2__object_detection__mDetectMiddleLine_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/coviroa/tImage.h"
#include "rrlib/si_units/si_units.h"

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
 * this module can detect white line
 */
class mDetectMiddleLine : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::coviroa::tImage> in_image;

	tOutput<rrlib::coviroa::tImage> middle_line_in_range;
	tOutput<rrlib::coviroa::tImage> middle_line_contours;
	tOutput<rrlib::coviroa::tImage> middle_line;
	tOutput<std::vector<rrlib::math::tVec2i>> lines_vector;

	tParameter<float> max_gap;
	tParameter<int> in_range_down,in_range_up,blur_size;
	tParameter<float> minimum_area_of_red_fields;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mDetectMiddleLine(core::tFrameworkElement *parent, const std::string &name = "DetectMiddleLine");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mDetectMiddleLine();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  float m_gap,area;

  int in_range_d,in_range_u,blur;

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
