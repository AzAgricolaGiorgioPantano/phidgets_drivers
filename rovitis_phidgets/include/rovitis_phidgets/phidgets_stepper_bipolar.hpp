/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_stepper_bipolar_hpp___
#define phidgets_stepper_bipolar_hpp___ //header is included only once

//basic
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>

//phidgets stuff
#include <libphidgets/phidget21.h>
//#include <phidgets_api/phidget.h>

//messagess
#include "rovitis_phidgets/phidget_stepper_bipolar.h"
#include "rovitis_phidgets/phidget_stepper_bipolar_set.h"
#include <geometry_msgs/Twist.h>

//ifkit
#include "rovitis_phidgets/phidget_interface_kit_888_set.h"
#include "rovitis_phidgets/phidget_interface_kit_888.h"


using namespace std;

namespace phidgets_stepper_bipolar  //split up code more classes whit the same name
{
    class phidgets_stepper_bipolar
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_stepper_bipolar(const phidgets_stepper_bipolar &src);


        protected:
        ros::NodeHandle nh_;

        //service handlers
        ros::ServiceServer reset_service_;

        //topics handlers
        ros::Publisher pub_;

        //service handlers
        ros::ServiceServer service_;

        ros::Subscriber ik_sub_;

        //cmd_vel
        ros::Subscriber cmd_vel_sub_;

        public:

        //global device handler
        CPhidgetStepperHandle phandle;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        bool initialised_;
        string device_label_;
        string topic_name_;
        string pfc_frame_;
        string vel_cmd_topic_name_;
        string service_name_;
        int wait_for_dev_delay_;
        int publish_frequency_;
        bool reset_;
        double current_linear_velocity_;
        double current_angular_velocity_;
        double max_angular_velocity_;
        double max_linear_velocity_;
        bool motors_active_;
        int motor_contorller_id_;
        double motor_acceleration_;
        double motor_velocity_;
        double motor_set_pose_;
        double robot_dist_from_center_to_wheel_;
        int contelec_X28_;

        /** \brief Standard construktor.
         *
         */
        phidgets_stepper_bipolar(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_stepper_bipolar(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //publish data
        void publish_data();
        //velocity callback
        void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr);
        //disconnecte phidgets
        void disconnect();
        //service callback
        bool set_pose(rovitis_phidgets::phidget_stepper_bipolar_set::Request &req, rovitis_phidgets::phidget_stepper_bipolar_set::Response &res);
        void ifkCallback(const rovitis_phidgets::phidget_interface_kit_888 data);

    };
}

#endif
