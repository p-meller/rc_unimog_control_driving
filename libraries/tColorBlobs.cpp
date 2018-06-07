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
/*!\file    projects/rc_unimog_control_group2/libraries/tColorBlobs.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-24
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tColorBlobs.h"

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
namespace finroc
{
namespace rc_unimog_control_group2
{
namespace libraries
{

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
// tColorBlobs constructors
//----------------------------------------------------------------------
tColorBlobs::tColorBlobs() :
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!

	center_x(0),
	center_y(0),
	left_edge(0),
	right_edge(0),
	top_edge(0),
	bottom_edge(0),
	color("")
		{}

//----------------------------------------------------------------------
// tColorBlobs destructor
//----------------------------------------------------------------------
tColorBlobs::~tColorBlobs()
{}

//----------------------------------------------------------------------
// tColorBlobs Method
//----------------------------------------------------------------------
//void tColorBlobs::SomeExampleMethod()
//{
//  This is an example for a method. Replace it by your own methods!
//}

std::ostream &operator << (std::ostream &stream, const tColorBlobs &color_blobs)
{
  stream << '{' << color_blobs.center_x << ", " << color_blobs.center_y << ", " << color_blobs.left_edge
		  << color_blobs.right_edge << ", " << color_blobs.top_edge << ", " << color_blobs.bottom_edge
		  << ", " << color_blobs.color << '}';

  return stream;
}


std::istream &operator >> (std::istream &stream, tColorBlobs &color_blobs)
{
  char temp(0);
  stream >> temp;
  if (temp == '{')
  {
    stream >> color_blobs.center_x >> temp >> color_blobs.center_y >> temp >> color_blobs.left_edge >> temp
	>> color_blobs.right_edge >> temp >> color_blobs.top_edge >> temp >> color_blobs.bottom_edge >> temp
	>> color_blobs.color >> temp;

  }
  else
  {
    printf("FAIL: >%c<", temp);
    stream.setstate(std::ios_base::failbit);
  }
  return stream;
}


#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tColorBlobs &color_blobs)
{
  stream << color_blobs.center_x << color_blobs.center_y << color_blobs.left_edge
		  << color_blobs.right_edge << color_blobs.top_edge << color_blobs.bottom_edge
		  << color_blobs.color;

  return stream;
}

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tColorBlobs &color_blobs)
{
  stream >> color_blobs.center_x >> color_blobs.center_y >> color_blobs.left_edge
  >> color_blobs.right_edge >> color_blobs.top_edge >> color_blobs.bottom_edge
	>> color_blobs.color;


  return stream;
}


rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tColorBlobs &color_blobs)
{
  std::stringstream ss;
  ss << color_blobs;

  return stream << ss.str();
}

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tColorBlobs &color_blobs)
{
  stream.GetWrappedStringStream() >> color_blobs;

  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
