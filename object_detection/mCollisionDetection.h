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
/*!\file    projects/rc_unimog_control_group2/object_detection/mCollisionDetection.h
 *
 * \author  Lukasz Owsianny
 *
 * \date    2018-01-24
 *
 * \brief Contains mCollisionDetection
 *
 * \b mCollisionDetection
 *
 * This module has to calculate distance to object using ultrasonic sensor, IR sensor, using distance data, position, width and length of obstacle
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__Final__mCollisionDetection_h__
#define __projects__Final__mCollisionDetection_h__

#include "plugins/structure/tModule.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/canvas/tCanvas3D.h"
#include "rrlib/point_clouds/point_cloud_utils.h"
#include "rrlib/canvas/tCanvas.h"
#include <list>
#include "rrlib/coviroa/tImage.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <chrono>
//#include "rrlib/distance_data/tDistance_data.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "libraries/signal_filters/mExponentialFilter.h"

using namespace cv;
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
 * This module has to calculate distance to object using ultrasonic sensor, IR sensor, using distance data, position, width and length of obstacle
 */
class mCollisionDetection : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:



tParameter<float> floor;

//ultrasonic sensors

tInput<rrlib::si_units::tLength<double>> input_us_front;
tInput<rrlib::si_units::tLength<>> input_us_rear;
tOutput<rrlib::si_units::tLength<>> output_us_rear;
tOutput<rrlib::si_units::tLength<>> output_us_front;

//point cloud and length of obstacles


tInput<rrlib::coviroa::tImage> input_rgb_image;
tOutput<rrlib::coviroa::tImage> out_image;
tOutput<rrlib::coviroa::tImage> out_contour_image;
tOutput<rrlib::coviroa::tImage> out_pyllon_image;

//tInput<rrlib::distance_data::tDistanceData> input_point_cloud;
//tOutput<rrlib::distance_data::tDistanceData> output_point_cloud;

//tOutput<rrlib::si_units::tLength<>> length_of_obstacle;
//tOutput<rrlib::si_units::tLength<>> width_of_obstacle;

//parameteres



	//tOutput<bool> obstacle_detected;

	tOutput<float> distance;


	//to bird view


//color blob detection


//do stopa
			tOutput<rrlib::coviroa::tImage> out_stop_image;
			tOutput<bool> stop_detected;
			tInput<rrlib::math::tVec2i> stop_center;
			tParameter<bool> mirror_flip;
			tOutput<double> distance_to_stop;
			tOutput<bool> stop_and_pylons_detected;



//time measuring
			tParameter<int> time_to_wait;
			tParameter<int> lowThreshold;
			tParameter<int> highThreshold;
			tParameter<int> pixelsForPylons;
			tParameter<double> multiplyForArea;
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mCollisionDetection(core::tFrameworkElement *parent, const std::string &name = "CollisionDetection");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mCollisionDetection();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:



  float floor_level;
  std::list<rrlib::si_units::tLength<double>> us_data;
  virtual void CameraDistance();

  rrlib::si_units::tLength<double> USSensors();
  virtual void OnStaticParameterChange() override;
  virtual void IRSensors();
  virtual void OnParameterChange() override;
  virtual void Update() override;
  bool PylonsDetection(cv::Mat src,cv::Mat color_image);
  cv::Mat ColorBlobDetection(const cv::Mat &src, Point middle);
  void DistanceStopCalculation(Point middle_of_sign);
  void PylonsDetection2(cv::Mat src);
  bool PylonsOntheRoad(rrlib::si_units::tLength<double> US, bool pylon);
  //do stopa
  	std::string stop_cascade_name;
    cv::CascadeClassifier stop_cascade;
    bool m;
	std::chrono::time_point<std::chrono::high_resolution_clock> current_time;

	std::chrono::milliseconds time_count;
	int wait;
	int iLowThreshold, iHighThreshold;
	int iPixelsToPylons;
	double dmultiplyForArea;
    //cv::Mat StopUpdate(const cv::Mat &temp, Point middle);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
