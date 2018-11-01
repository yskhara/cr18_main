/*
 * ControllerCommands.hpp
 *
 *  Created on: Nov 1, 2018
 *      Author: yusaku
 */

#pragma once

const std::vector<ControllerCommands> CrMain::route1_pp1_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp1
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp1, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp1 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp1, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp2
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp2, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp1 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp1, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route1_pp2_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp2
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp2, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp1 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp1, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route1_pp3_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route1_pp4_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 3rd floor
        ControllerCommands::set_lift_3, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route2_pp1_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp1
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp1, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 1st floor
        ControllerCommands::set_lift_1, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp2
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp2, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp4 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp4, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route2_pp2_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp2
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp2, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp2 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp2, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp4 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp4, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route2_pp3_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp3
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp3, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp3 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp3, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp4 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp4, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

const std::vector<ControllerCommands> CrMain::route2_pp4_op_commands( {
//standby at sz
        ControllerCommands::standby,

        // pickup at pp4
        ControllerCommands::set_lift_p, ControllerCommands::move_to_pp4, ControllerCommands::pp_pickup, ControllerCommands::delay, ControllerCommands::pp_backup,
        // deliver at dp4 @ 2nd floor
        ControllerCommands::set_lift_2, ControllerCommands::move_to_dp4, ControllerCommands::dp_deliver, ControllerCommands::delay, ControllerCommands::dp_backup,

        // return to base
        ControllerCommands::set_lift_p, ControllerCommands::rtb,

        ControllerCommands::shutdown, });

