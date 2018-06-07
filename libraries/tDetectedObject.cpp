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
/*!\file    projects/rc_unimog_control_group2/libraries/tDetectedObject.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-22
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <string.h>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//using namespace rrlib::math;
//using namespace rrlib::localization;
//----------------------------------------------------------------------
//using namespace std;
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
// tDetectedObject constructors
//----------------------------------------------------------------------
tDetectedObject::tDetectedObject() :
//  If you have some member variables, please initialize them here.
//		Especially built-in types (like pointers!). Delete this line otherwise!
		width(0), length(0), detected_object_type("") ,
//		height(0)
		possible_object_type({"road_side_left_marking", "road_middle_marking", "road_side_right_marking", "bridge", "pylon", "stop_sign", "unimog"})
{
	rrlib::math::tAngleRad null_angle = static_cast<rrlib::math::tAngleRad>(0);
	position_middle.Set(0, 0, 0, null_angle, null_angle, null_angle);
//	possible_object_type = {"road_side_left_marking", "road_middle_marking", "road_side_right_marking", "bridge", "pylon", "stop_sign", "unimog", "other"};
}

//----------------------------------------------------------------------
// tDetectedObject destructor
//----------------------------------------------------------------------
tDetectedObject::~tDetectedObject() {
}

//----------------------------------------------------------------------
// tDetectedObject Method
//----------------------------------------------------------------------

std::ostream &operator << (std::ostream &stream, const tDetectedObject &detected_object) {
	stream << '{' << detected_object.position_middle << ", " << detected_object.width << ", " << detected_object.length << ", " << detected_object.detected_object_type << '}';
	return stream;
}

std::istream &operator >>(std::istream &stream, tDetectedObject &detected_object) {
	char temp(0);
	stream >> temp;
	if (temp == '{') {
		rrlib::localization::tPose3D<> newPose;
		stream >> newPose >> temp >> detected_object.width >> temp >> detected_object.length >> temp >> detected_object.detected_object_type >> temp;
		detected_object.position_middle = newPose;
	} else {
		printf("FAIL: >%c<", temp);
		stream.setstate(std::ios_base::failbit);
	}
	return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tDetectedObject &detected_object)
{
	stream << detected_object.position_middle << detected_object.width << detected_object.length << detected_object.detected_object_type;

	return stream;
}

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tDetectedObject &detected_object)
{
	stream >> detected_object.position_middle >> detected_object.width >> detected_object.length >> detected_object.detected_object_type;

	return stream;
}

rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tDetectedObject &detected_object)
{
	std::stringstream ss;
	ss << detected_object;

	return stream << ss.str();
}

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tDetectedObject &detected_object)
{
	stream.GetWrappedStringStream() >> detected_object;

	return stream;
}

#endif

//#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
//
//rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputSream& stream, const tDetectedObject& detected_object) {
//	stream << detected_object.position.middle << detected_object.width << detected_object.length << detected_object.detected_object_type;
//	return stream;
//}
//
//rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tDetectedObject& detected_object) {
//	stream >> detected_object.position_middle >> detected_object.width >> detected_object.length >> detected_object.detected_object_type;
//	return stream;
//}
//
//rrlib::xml::tNode& operator << (rrlib::xml::tNode& node, const tDetectedObject& detected_object) {
//	node.SetAttribute("position_middle", detected_object.position_middle);
//	node.SetAttribute("width", detected_object.width);
//	node.SetAttribute("length", detected_object.length);
//	node.SetAttribute("detected_object_type", detected_object.detected_object_type);
//	return node;
//}
//
//const rrlib::xml::tNode& operator >> (const rrlib::xml::tNode& node, tDetectedObject& detected_object) {
//	return node;
//}
//
//#endif
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
