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
        // Wait until ROS gives time information
        while(ros::Time::now().toSec() == 0)
        {
            // Wait a duration of 10 milliseconds
            ros::Duration d = ros::Duration(0.010);
            d.sleep();
        }

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
        action_t versaball_actions[4];
        versaball_actions[0].description = "pressure pump";
        versaball_actions[0].delay_start = _start_pressure;
        versaball_actions[0].delay_stop = _stop_pressure;
        versaball_actions[0].index = 1;
        versaball_actions[1].description = "pressure valve";
        versaball_actions[1].delay_start = _open_presure_valve;
        versaball_actions[1].delay_stop = _close_pressure_valve;
        versaball_actions[1].index = 2;
        versaball_actions[2].description = "void pump";
        versaball_actions[2].delay_start = _start_void;
        versaball_actions[2].delay_stop = _stop_void;
        versaball_actions[2].index = 0;
        versaball_actions[3].description = "void valve";
        versaball_actions[3].delay_start = _open_void_valve;
        versaball_actions[3].delay_stop = _close_void_valve;
        versaball_actions[3].index = 3;

        unsigned int actions_done = 0;

        // time at which we start the grasping sequence
        ros::Time start_time = ros::Time::now();
        // The loop bellow will run each millisecond (1000Hz), in theory
        ros::Rate rate(1000);

        while(actions_done != 0b11111111)
        {
            ros::Time now = ros::Time::now();

            for (int i=0; i<4; i++)
            {
                if (!(actions_done & 0b1<<i))
                {
                    if (_timed_action(start_time, now, versaball_actions[i], true))
                        actions_done = actions_done | 0b1<<i;
                }
                if (!(actions_done & 0b1<<i+4))
                {
                    if (_timed_action(start_time, now, versaball_actions[i], false))
                        actions_done = actions_done | 0b1<<i+4;
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

    bool VersaballNode::_timed_action(ros::Time start_time, ros::Time now,
        action_t current_action, bool state)
    {
        ros::Duration delay = state?current_action.delay_start:current_action.delay_stop;
        std::string description = std::string(state?"enable":"disable") + " "
            + current_action.description;

        if (start_time + delay <= now)
        {
            if (_set_phidgets_state(current_action.index, state))
                ROS_DEBUG_STREAM("successfully " << description);
            else
                ROS_DEBUG_STREAM("unable to " << description);

            return true;
        }
        return false;
    }

    bool VersaballNode::_set_phidgets_state(uint8_t index, uint16_t state)
    {
        phidgets_interface_kit::setState service_call;
        service_call.request.index = index;
        service_call.request.state = state;

        ros::ServiceClient client =
            _nh.serviceClient<phidgets_interface_kit::setState>("/phidgets_ik/set_state");

        return client.call(service_call);
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

    versaball_state state()
    {
        return _current_state;
    }

}