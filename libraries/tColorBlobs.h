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
/*!\file    projects/rc_unimog_control_group2/libraries/tColorBlobs.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-24
 *
 * \brief   Contains tColorBlobs
 *
 * \b tColorBlobs
 *
 * detected regions with coordinates an color, which are detected by the modul ColorBlobsDetection
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__libraries__tColorBlobs_h__
#define __projects__rc_unimog_control_group2__libraries__tColorBlobs_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <string>
#include <iostream>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/serialization/serialization.h"
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * detected regions with coordinates an color, which are detected by the modul ColorBlobsDetection
 */
class tColorBlobs
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tColorBlobs();

  ~tColorBlobs();
//  You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!
//
//  Here is the right place for your public methods. Replace this line by your declarations!
  int center_x;
  int center_y;
  int left_edge;
  int right_edge;
  int top_edge;
  int bottom_edge;
  std::string color;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

};

std::ostream &operator << (std::ostream &stream, const tColorBlobs &color_blobs);

std::istream &operator >> (std::istream &stream, tColorBlobs &color_blobs);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tColorBlobs &color_blobs);

rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tColorBlobs &color_blobs);


rrlib::serialization::tStringOutputStream &operator << (rrlib::serialization::tStringOutputStream &stream, const tColorBlobs &color_blobs);

rrlib::serialization::tStringInputStream &operator >> (rrlib::serialization::tStringInputStream &stream, tColorBlobs &color_blobs);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
