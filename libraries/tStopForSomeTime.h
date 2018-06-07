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
/*!\file    projects/rc_unimog_control_group2/libraries/tStopForSomeTime.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-14
 *
 * \brief   Contains tStopForSomeTime
 *
 * \b tStopForSomeTime
 *
 * do nothing for specified ammount of time
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__libraries__tStopForSomeTime_h__
#define __projects__rc_unimog_control_group2__libraries__tStopForSomeTime_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <chrono>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/libraries/tStateAbstract.h"

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
 * do nothing for specified ammount of time
 */
class tStopForSomeTime: public tStateAbstract
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tStopForSomeTime();

  ~tStopForSomeTime();

  void update(tStateManager* state) override;

  bool wait();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	std::chrono::time_point<std::chrono::high_resolution_clock> current_time;

	std::chrono::milliseconds time_count;


};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
