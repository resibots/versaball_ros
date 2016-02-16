#ifndef VERSABALL_H
#define VERSABALL_H

#include <ros/ros.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <versaball/versaballConfig.h>
// Standard messages
#include <std_srvs/Empty.h>

namespace versaball
{
    class VersaballNode
    {
    public:
        VersaballNode();

        /** Wait for the needed services to be advertised and in turn advertise ours,
            i.e. "grasp" and "release" in the private namespace.

            @return true if succeeded to advertise the services, and false otherwise
        **/
        bool advertise_services();

        bool grasp_callback(std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res);
        bool release_callback(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res);

        void dynamic_reconfigure_cb(versaball::versaballConfig &config, uint32_t level);

    private:
        struct action_t{
            std::string description;
            ros::Duration delay_start;
            ros::Duration delay_stop;
            uint16_t index;
        };

        bool _set_relay_state(uint8_t index, uint16_t state);
        bool _do_action(ros::Time start_time, ros::Time now,
            action_t current_action, bool state);

        ros::NodeHandle _nh;;
        // Handles for the service we advertise; deletion will unadvertise the service
        ros::ServiceServer _grasp_service, _release_service;

        // times at which each step of a grasping or release phase has to be
        // executed
        ros::Duration _start_pressure, _open_presure_valve, _close_pressure_valve,
            _stop_pressure, _start_void, _open_void_valve, _close_void_valve,
            _stop_void;

        dynamic_reconfigure::Server<versaball::versaballConfig> _dynamic_reconfigure_server;
        dynamic_reconfigure::Server<versaball::versaballConfig>::CallbackType _dynamic_reconfigure_cb_t;
    };
}

#endif // VERSABALL_H