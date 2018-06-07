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
/*!\file    projects/rc_unimog_control_group2/libraries/tObstacle.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-22
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tObstacle.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace finroc {
namespace rc_unimog_control_group2 {
namespace libraries {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tObstacle constructors
//----------------------------------------------------------------------
tObstacle::tObstacle() :
//  If you have some member variables, please initialize them here.
//  Especially built-in types (like pointers!). Delete this line otherwise!
		width(0), length(0) {
//	rrlib::math::tAngleRad null_angle = (rrlib::math::tAngleRad) 0;
	rrlib::math::tAngleRad null_angle = static_cast<rrlib::math::tAngleRad>(0);
	position_middle.Set(0, 0, 0, null_angle, null_angle, null_angle);
}

//----------------------------------------------------------------------
// tObstacle destructor
//----------------------------------------------------------------------
tObstacle::~tObstacle() {
}

//----------------------------------------------------------------------
// tObstacle Method
//----------------------------------------------------------------------

std::ostream &operator <<(std::ostream &stream, const tObstacle &obstacle) {
	stream << '{' << obstacle.position_middle << ", " << obstacle.width << ", "
			<< obstacle.length << '}';

	return stream;
}

std::istream &operator >>(std::istream &stream, tObstacle &obstacle) {
	char temp(0);
	stream >> temp;
	if (temp == '{') {
		rrlib::localization::tPose3D<> newPose;
		stream >> newPose >> temp >> obstacle.width >> temp >> obstacle.length
				>> temp;
		obstacle.position_middle = newPose;
	} else {
		printf("FAIL: >%c<", temp);
		stream.setstate(std::ios_base::failbit);
	}
	return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tObstacle &obstacle)
{
	stream << obstacle.position_middle << obstacle.width << obstacle.length;

	return stream;
}

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tObstacle &obstacle)
{
	stream >> obstacle.position_middle >> obstacle.width >> obstacle.length;

	return stream;
}

rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tObstacle &obstacle)
{
	std::stringstream ss;
	ss << obstacle;

	return stream << ss.str();
}

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tObstacle &obstacle)
{
	stream.GetWrappedStringStream() >> obstacle;

	return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
