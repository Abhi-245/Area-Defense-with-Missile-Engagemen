/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/autonomy/BoundaryDefense/BoundaryDefense.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/common/Waypoint.h>
#include <scrimmage/plugins/autonomy/WaypointGenerator/WaypointList.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <iostream>
#include <limits>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sci = scrimmage::interaction;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::BoundaryDefense,
                BoundaryDefense_plugin)

namespace scrimmage {
namespace autonomy {

BoundaryDefense::BoundaryDefense() {
}

void BoundaryDefense::init(std::map<std::string, std::string> &params) {
   boundary_id_ = sc::get<int>("boundary_id", params, boundary_id_);
pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");
    auto callback = [&] (scrimmage::MessagePtr<sp::Shape> msg) {
        std::shared_ptr<sci::BoundaryBase> boundary = sci::Boundary::make_boundary(msg->data);
        boundaries_[msg->data.id().id()] = std::make_pair(msg->data, boundary);
    };
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", callback);

    output_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
    output_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
    output_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);
}

bool BoundaryDefense::step_autonomy(double t, double dt) {
   // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
 for (auto it = contacts_->begin(); it != contacts_->end(); it++) 
{
    // if this contact is not on the same team
    if (it->second.id().team_id() != parent_->id().team_id()) 
    {
		follow_id_ = it->first;    
        ent_state = contacts_->at(follow_id_).state();   
    }
}   
// double min_dist = std::numeric_limits<double>::infinity();
//     for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

//         // Skip if this contact is on the same team
//         if (it->second.id().team_id() == parent_->id().team_id()) {
//             continue;
//         }

//         // Calculate distance to entity
//         double dist = (it->second.state()->pos() - state_->pos()).norm();

//         if (dist < min_dist) {
//             // If this is the minimum distance, save distance and reference to
//             // entity
//             min_dist = dist;
//             follow_id_ = it->first;
//         }
//     }
    // ent_state = contacts_->at(follow_id_).state();
 //  cout<<"desiredsdsd"<<endl;
	 
	
    auto it = boundaries_.find(boundary_id_);
    if (it == boundaries_.end()) 
    {
        return true;
    }

    // Is the entity within the boundary?
    if (std::get<1>(it->second)->contains(ent_state->pos())) 
    {       
         Eigen::Vector3d desired_point = ent_state->pos() + ent_state->vel() * time_->dt();
         Eigen::Vector3d stored_point =ent_state->pos() + ent_state->vel()*56;
                // Calculate WEZ for entity
        double dist = (ent_state->pos() - state_->pos()).norm();
        //angle calculation
        Eigen::Vector3d own_point = state_->pos();
        Eigen::Vector3d opponent_point = ent_state->pos();
        Eigen::Vector3d position_vector =  own_point  - opponent_point;
        double dot_vector =  position_vector.dot(own_point);
        double own_magnitude = own_point.norm();
        double opponent_magnitude = opponent_point.norm();
        double cos_angle = dot_vector/(own_magnitude * opponent_magnitude);
        
        if(dist<260 and !entity_generate)
         {
            
            // Generate the missile
            // Create a state for the entity and place it at position (10, 10, 100) with
            // a roll, pitch, and yaw of (0 deg, 0 deg, 0 deg).
            State s;
            s.pos() << state_->pos()(0), state_->pos()(1), state_->pos()(2)+100;
            s.quat() = scrimmage::Quaternion(0, 0, 0);
            // Create the GenerateEntity message
            auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();
            // Copy the new state into the message
            sc::set(msg->data.mutable_state(), s);
            // Set the entity_tag that references the entity to be generated in the
            // mission XML file.
            msg->data.set_entity_tag("gen_straight");
            // Load parameters from trajectorymissile
            // auto kv_autonomy = msg->data.add_entity_param();
            // kv_autonomy->set_key("autonomy");
            // kv_autonomy->set_value("trajectorymissile");        
            // Publish the GenerateEntity message
            // cout<<"hello"<<endl;
            entity_generate=true;
            pub_gen_ents_->publish(msg);   
            Eigen::Vector3d desired_velocity ;
            desired_velocity.setZero();
            vars_.output(output_vel_x_idx_, -desired_velocity(0));
            vars_.output(output_vel_y_idx_, -desired_velocity(1));
            vars_.output(output_vel_z_idx_, -desired_velocity(2));                  
         }
         
         if(entity_generate==true){
             Eigen::Vector3d desired_velocity = stored_point - state_->pos();
                    //   Eigen::Vector3d desired_velocity = desired_velocity.setZero();
            vars_.output(output_vel_x_idx_, desired_velocity(0));
            vars_.output(output_vel_y_idx_, desired_velocity(1));
            vars_.output(output_vel_z_idx_, desired_velocity(2));
         }
         else{
            Eigen::Vector3d desired_velocity = desired_point - ent_state->pos();
            // desired_velocity.setZero();
            vars_.output(output_vel_x_idx_, -desired_velocity(0));
            vars_.output(output_vel_y_idx_, -desired_velocity(1));
            vars_.output(output_vel_z_idx_, -desired_velocity(2)); 
         }
       
        
        }  
    else 
    {   
			Eigen::Vector3d desired_velocity;
            desired_velocity.setZero();
            vars_.output(output_vel_x_idx_, -desired_velocity(0));
    vars_.output(output_vel_y_idx_, -desired_velocity(1));
    vars_.output(output_vel_z_idx_, -desired_velocity(2));
	}
    return true;
}
//          else{
            
//             Eigen::Vector3d desired_velocity = desired_point - state_->pos();

//             vars_.output(output_vel_x_idx_, desired_velocity(0));
//             vars_.output(output_vel_y_idx_, desired_velocity(1));
//             vars_.output(output_vel_z_idx_, desired_velocity(2));           
            
//          }
        
//         }  
//     else 
//     {   
// 			Eigen::Vector3d desired_velocity;
//             desired_velocity.setZero();
//             vars_.output(output_vel_x_idx_, desired_velocity(0));
//     vars_.output(output_vel_y_idx_, desired_velocity(1));
//     vars_.output(output_vel_z_idx_, desired_velocity(2));
// 	}
//     return true;
// }

}          
}