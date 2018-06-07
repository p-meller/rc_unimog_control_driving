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
/*!\file    projects/rc_unimog_control_group2/libraries/tDetectedObject.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-22
 *
 * \brief   Contains tDetectedObject
 *
 * \b tDetectedObject
 *
 * the compound data type which represents a detected object
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__libraries__tDetectedObject_h__
#define __projects__rc_unimog_control_group2__libraries__tDetectedObject_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include <string.h>
#include "rrlib/si_units/si_units.h"
#include "rrlib/localization/tPose.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//# ifdef  _LIB_RRLIB_SERIALIZATION_PRESENT
#include "rrlib/serialization/serialization.h"
//#endif
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
 * the compound data type which represents a detected object
 */
class tDetectedObject {

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	tDetectedObject();

	~tDetectedObject();

	rrlib::localization::tPose3D<double> position_middle;// middle point of object
	rrlib::si_units::tLength<double> width;
	rrlib::si_units::tLength<double> length;
//  rrlib::si_units::tLength<double> height;
	std::string detected_object_type;
	std::vector<std::string> possible_object_type;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


//  Here is the right place for your variables. Replace this line by your declarations!
//	std::vector<std::string> possible_object_type;
//	friend rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tDetectedObject& detected_object);
//	friend rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, const tDetectedObject& detected_object);
//	friend rrlib::xml::tNode& operator << (rrlib::xml::tNode& node, tDetectedObject& detected_object);
//	friend const rrlib::xml::tNode& operator >> (const rrlib::xml::tNode& node, tDetectedObject detected_object);
};

std::ostream &operator << (std::ostream &stream, const tDetectedObject &detected_object);

std::istream &operator >> (std::istream &stream, tDetectedObject &detected_object);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tDetectedObject &detected_object);

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tDetectedObject &detected_object);

rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tDetectedObject &detected_object);

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tDetectedObject &detected_object);

#endif

//	rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tDetectedObject detected_object);
//	rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tDetectedObject& detected_object);
//	rrlib::xml::tNode& operator << (rrlib::xml::tNode& node, const tDetectedObject detected_object);
//	const rrlib::xml::tNode& operator >> (const rrlib::xml::tNode& node, tDetectedObject detected_object);
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
