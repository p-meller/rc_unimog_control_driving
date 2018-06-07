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
/*!\file    projects/rc_unimog_control_group2/libraries/tObstacle.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-22
 *
 * \brief   Contains tObstacle
 *
 * \b tObstacle
 *
 * compound data type which represents an object which is detected as obstacle in ObstacleDetection
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__libraries__tObstacle_h__
#define __projects__rc_unimog_control_group2__libraries__tObstacle_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include "rrlib/si_units/si_units.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/serialization/serialization.h"
#include "rrlib/localization/tPose.h"
//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc {
namespace rc_unimog_control_group2 {
namespace libraries {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * compound data type which represents an object which is detected as obstacle in ObstacleDetection
 */
class tObstacle {

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	tObstacle();

	~tObstacle();

	rrlib::localization::tPose3D<double> position_middle;// middle point of object
	rrlib::si_units::tLength<double> width;
	rrlib::si_units::tLength<double> length;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

};

std::ostream &operator <<(std::ostream &stream, const tObstacle &obstacle);

std::istream &operator >>(std::istream &stream, tObstacle &obstacle);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tObstacle &obstacle);

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tObstacle &obstacle);

rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tObstacle &obstacle);

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tObstacle &obstacle);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
