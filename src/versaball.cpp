#include <phidgets_interface_kit/setStates.h> // TODO: remove this include
#include <phidgets_interface_kit/setState.h>

#include <versaball/versaball.hpp>

namespace versaball
{
    VersaballNode::VersaballNode():
        _nh("~"),
        _start_pressure(-1), _open_presure_valve(-1), _close_pressure_valve(-1),
        _stop_pressure(-1), _start_void(-1), _open_void_valve(-1),
        _close_void_valve(-1), _stop_void(-1)
    {
        // Set the callback for dynamic reconfiguration of the node's parameters
        _dynamic_reconfigure_cb_t = boost::bind(
            &VersaballNode::dynamic_reconfigure_cb,
            this,
            _1,
            _2);
        _dynamic_reconfigure_server.setCallback(_dynamic_reconfigure_cb_t);
    }

    bool VersaballNode::advertise_services()
    {
        bool success = false;
        std::string phidgets_service = "phidgets_ik/set_states";

        ROS_INFO_STREAM("Waiting for \""<<phidgets_service<<"\" service"
            "(2 seconds each).");
        if (ros::service::waitForService(phidgets_service, 2000))
        {

            _grasp_service = _nh.advertiseService("grasp",
                &VersaballNode::grasp_callback, this);
            _release_service = _nh.advertiseService("release",
                &VersaballNode::release_callback, this);

            if (_grasp_service && _release_service)
                success = true;
        }
        else
        {
            ROS_ERROR_STREAM("The service \""<<phidgets_service<<"\" was not"
                "found");
        }

        return success;
    }

    bool VersaballNode::grasp_callback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        // static bool new_state = false;
        // new_state = !new_state;
        // std::vector<uint16_t> indices;
        // std::vector<uint8_t> states;
        // indices.push_back(2);
        // states.push_back(new_state);
        // ROS_INFO_STREAM("Requested state " << (new_state?"true":"false") << " for output 2");
        //
        // phidgets_interface_kit::setStates srv;
        // srv.request.indices = indices;
        // srv.request.states = states;

        // std::string service_name = "/phidgets_ik/set_states";
        // ros::ServiceClient client =
        //     _nh.serviceClient<phidgets_interface_kit::setStates>(service_name);
        // if (client.call(srv))
        //     ROS_DEBUG_STREAM("Call to service "<<service_name<<" successful");
        // else
        //     ROS_DEBUG_STREAM("Call to service "<<service_name<<" failed");

        // struct action{
        //     std::string description;
        //     ros::Duration delay_start;
        //     ros::Duration delay_stop;
        //     uint16_t index;
        // };

        action_t versaball_actions[4];
        versaball_actions[0].description = "pressure pump";
        versaball_actions[0].delay_start = _start_pressure;
        versaball_actions[0].delay_stop = _stop_pressure;
        versaball_actions[0].index = 0;
        versaball_actions[1].description = "pressure valve";
        versaball_actions[1].delay_start = _open_presure_valve;
        versaball_actions[1].delay_stop = _close_pressure_valve;
        versaball_actions[1].index = 1;
        versaball_actions[2].description = "void pump";
        versaball_actions[2].delay_start = _start_void;
        versaball_actions[2].delay_stop = _stop_void;
        versaball_actions[2].index = 2;
        versaball_actions[3].description = "void valve";
        versaball_actions[3].delay_start = _open_void_valve;
        versaball_actions[3].delay_stop = _close_void_valve;
        versaball_actions[3].index = 3;

        // Wait until ROS gives time information
        while(ros::Time::now().toSec() == 0)
        {
            // Wait a duration of 10 milliseconds
            ros::Duration d = ros::Duration(0.010);
            d.sleep();
        }
        // Get the time and store it in the time variable
        ros::Time start_time = ros::Time::now();
        // The loop bellow will run each millisecond (1000Hz), in theory
        ros::Rate rate(1000);

        std::vector<bool> task_done(8, false);

        unsigned short remaining_actions = 8;
        while(remaining_actions > 0)
        {
            ros::Time now = ros::Time::now();

            for (int i=0; i<4; i++)
            {
                if (!task_done[i])
                {
                    if (_do_action(start_time, now, versaball_actions[i], true))
                    {
                        --remaining_actions;
                        task_done[i] = true;
                    }
                }
                if (!task_done[i+4])
                {
                    if (_do_action(start_time, now, versaball_actions[i], false))
                    {
                        --remaining_actions;
                        task_done[i+4] = true;
                    }
                }
            }

            rate.sleep();
        }

        return true;
    }

    bool VersaballNode::release_callback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        return true;
    }

    bool VersaballNode::_set_relay_state(uint8_t index, uint16_t state)
    {
        phidgets_interface_kit::setState service_call;
        service_call.request.index = index;
        service_call.request.state = state;

        std::string service_name = "/phidgets_ik/set_state"; // TODO: make an attribute
        ros::ServiceClient client =
            _nh.serviceClient<phidgets_interface_kit::setState>(service_name);

        return client.call(service_call);
    }

    bool VersaballNode::_do_action(ros::Time start_time,
        ros::Time now, action_t current_action, bool state)
    {
        ros::Duration delay = state?current_action.delay_start:current_action.delay_stop;
        std::string description = std::string(state?"enable":"disable") + " "
            + current_action.description;

        if (start_time + delay <= now)
        {
            if (_set_relay_state(current_action.index, state))
                ROS_INFO_STREAM("successfully " << description);
            else
                ROS_INFO_STREAM("unable to " << description);

            return true;
        }
        return false;
    }

    void VersaballNode::dynamic_reconfigure_cb(versaball::versaballConfig &config, uint32_t level)
    {
        _start_pressure			= ros::Duration(config.start_pressure*1e-3);
        _open_presure_valve		= ros::Duration(config.open_presure_valve*1e-3);
        _close_pressure_valve	= ros::Duration(config.close_pressure_valve*1e-3);
        _stop_pressure			= ros::Duration(config.stop_pressure*1e-3);
        _start_void				= ros::Duration(config.start_void*1e-3);
        _open_void_valve		= ros::Duration(config.open_void_valve*1e-3);
        _close_void_valve		= ros::Duration(config.close_void_valve*1e-3);
        _stop_void				= ros::Duration(config.stop_void*1e-3);
    }

}