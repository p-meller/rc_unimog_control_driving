//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    gMainControl.h
 *
 * \author  Unknown
 *
 * \date    2011-11-07
 *
 * \brief Contains gMainControl
 *
 * \b gMainControl
 *
 */
//----------------------------------------------------------------------
#ifndef _rc_unimog__gMainControl_h_
#define _rc_unimog__gMainControl_h_


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_group2 
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of gMainControl
/*! A more detailed description of gMainControl which
 *  Max has not done yet!
 *
 */
class gMainControl : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  gMainControl(finroc::core::tFrameworkElement *parent, const std::string &name = "MainControl",
               const std::string &structure_config_file = __FILE__".xml");


//----------------------------------------------------------------------
// Protected methods (no fields/variables)
//----------------------------------------------------------------------
protected:

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
