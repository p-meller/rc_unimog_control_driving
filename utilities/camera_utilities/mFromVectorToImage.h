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
/*!\file    projects/rc_unimog_control_group2/utilities/camera_utilities/mFromVectorToImage.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-08
 *
 * \brief Contains mFromVectorToImage
 *
 * \b mFromVectorToImage
 *
 * converting from vector of images to image
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__utilities__camera_utilities__mFromVectorToImage_h__
#define __projects__rc_unimog_control_group2__utilities__camera_utilities__mFromVectorToImage_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/coviroa/tImage.h"

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
namespace utilities
{
namespace camera_utilities
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * converting from vector of images to image
 */
class mFromVectorToImage : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	  tInput<std::vector<rrlib::coviroa::tImage>> in_images;
	  tOutput<rrlib::coviroa::tImage> out_image;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mFromVectorToImage(core::tFrameworkElement *parent, const std::string &name = "FromVectorToImage");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mFromVectorToImage();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:



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
}



#endif
