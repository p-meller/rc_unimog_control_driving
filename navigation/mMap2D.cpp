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
/*!\file    projects/rc_unimog_control_group2/navi/mMap2D.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-11
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navigation/mMap2D.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <cfloat>
#include <climits>
#include <cmath>

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

using namespace rrlib::math;

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_group2
{
namespace navigation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mMap2D> cCREATE_ACTION_FOR_M_MAP2D(
		"Map2D");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void mMap2D::outside_line_filter()
{
	data_ports::tPortDataPointer<
			const std::vector<std::vector<rrlib::math::tVec2i>>>in_vec_ptr=
	this->outside_line_input.GetPointer();

	if(in_vec_ptr->size()>1)
	{
		int min_left_y=0;
		int min_right_y=0;
		int index_left=-1;
		int index_right=-1;
		for(unsigned int i=0;i<in_vec_ptr->size();i++)
		{
			if(in_vec_ptr->at(i).size()>0)
			{
				if(in_vec_ptr->at(i).at(0).Y()>min_left_y and in_vec_ptr->at(i).at(0).X()<320)
				{
					min_left_y=in_vec_ptr->at(i).at(0).Y();
					index_left=i;
				}
				if(in_vec_ptr->at(i).at(0).Y()>min_right_y and in_vec_ptr->at(i).at(0).X()>320)
				{
					min_right_y=in_vec_ptr->at(i).at(0).Y();
					index_right=i;
				}
			}
		}
		if(index_left!=-1)
		{
			outside_left_line=in_vec_ptr->at(index_left);
			distance_to_left_line=outside_left_line[0].X() - 320;
		}
		if(index_right!=-1)
		{
			outside_right_line=in_vec_ptr->at(index_right);
			distance_to_right_line=outside_right_line[0].X() - 320;
		}

	}
	else if(in_vec_ptr->size()==1)
	{
		if(in_vec_ptr->at(0).size()>0)
		{
			if(in_vec_ptr->at(0).at(0).X()>320)
			{
				outside_right_line=in_vec_ptr->at(0);
				outside_left_line.clear();
				distance_to_right_line=outside_right_line[0].X() - 320;
			}
			else
			{
				outside_left_line=in_vec_ptr->at(0);
				outside_right_line.clear();
				distance_to_left_line=outside_left_line[0].X() - 320;
			}
		}
	}
	else if(in_vec_ptr->size()==0)
	{
		outside_right_line.clear();
		outside_left_line.clear();
	}

}

void mMap2D::inside_line_filter()
{
	data_ports::tPortDataPointer<const std::vector<rrlib::math::tVec2i>> in_vec_ptr =
			this->middle_line_input.GetPointer();

	middle_line = *in_vec_ptr;

	//middle_line.reserve(in_vec_ptr->size());
	//for (unsigned int i = in_vec_ptr->size(); i > 0; i--)
	//{
	//	middle_line.push_back(in_vec_ptr->at(i - 1));
	//}

	if (middle_line.size() > 0)
	{
		distance_to_middle_line = middle_line[0].X() - 320;
	}
	else
	{
		middle_line.clear();
	}
}

//----------------------------------------------------------------------
// mMap2D constructor
//----------------------------------------------------------------------
mMap2D::mMap2D(core::tFrameworkElement *parent, const std::string &name) :
		tModule(parent, name, false)// change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
}

//----------------------------------------------------------------------
// mMap2D destructor
//----------------------------------------------------------------------
mMap2D::~mMap2D()
{
}

//----------------------------------------------------------------------
// mMap2D OnStaticParameterChange
//----------------------------------------------------------------------
void mMap2D::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mMap2D OnParameterChange
//----------------------------------------------------------------------
void mMap2D::OnParameterChange()
{
}

//----------------------------------------------------------------------
// mMap2D Update
//----------------------------------------------------------------------
void mMap2D::Update()
{
	if (this->InputChanged())
	{

		try
		{
			outside_line_filter();
			inside_line_filter();
		} catch (...)
		{
			std::cout << "jebło" << std::endl;
		}

		//this->distance_to_middle.Publish(distance_to_middle_line);

		//distance to middle of right lane

		if (outside_right_line.size() > 0)
		{

			distance_to_right_lane.Publish(distance_to_right_line - 70);
			//std::cout << outside_right_line[0].X() << std::endl;
		}
		else if (middle_line.size() > 0)
		{
			distance_to_right_lane.Publish(distance_to_middle_line + 70);
		}
		else if (outside_left_line.size() > 0)
		{
			distance_to_right_lane.Publish(distance_to_left_line + 70 + 140);
		}
		else
		{
			distance_to_right_lane.Publish(0);
		}

		//distance to middle of right lane
		if (outside_left_line.size() > 0)
		{
			distance_to_left_lane.Publish(distance_to_left_line + 70);
		}
		else if (middle_line.size() > 0)
		{
			distance_to_left_lane.Publish(distance_to_middle_line - 70);
		}
		else if (outside_right_line.size() > 0)
		{
			distance_to_left_lane.Publish(distance_to_right_line - 70 - 140);
		}
		else
		{
			distance_to_left_lane.Publish(0);
		}

		if (visualization.IsConnected())
		{
			// visualize using tCanvas2D
			// obtain buffer
			data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> canvas =
					visualization.GetUnusedBuffer();
			canvas->Clear();

			for (unsigned int i = 1; i < middle_line.size(); i++)
			{
				canvas->SetColor(255, 255, 255);
				canvas->DrawArrow(middle_line[i - 1], middle_line[i]);

			}

			for (unsigned int i = 1; i < outside_left_line.size(); i++)
			{
				canvas->SetColor(0, 255, 0);
				canvas->DrawArrow(outside_left_line[i - 1],
						outside_left_line[i]);

			}

			for (unsigned int i = 1; i < outside_right_line.size(); i++)
			{
				canvas->SetColor(255, 0, 0);
				canvas->DrawArrow(outside_right_line[i - 1],
						outside_right_line[i]);

			}

			visualization.Publish(canvas);
		}
	}
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
