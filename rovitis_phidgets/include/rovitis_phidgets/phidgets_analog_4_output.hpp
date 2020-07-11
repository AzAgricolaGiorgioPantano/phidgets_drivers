/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_analog_4_output_hpp___
#define phidgets_analog_4_output_hpp___ //header is included only once

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
#include "rovitis_phidgets/phidget_analog_4_output.h"
//services
#include "rovitis_phidgets/phidget_analog_4_output_set.h"

using namespace std;

namespace phidgets_analog_4_output  //split up code more classes whit the same name
{
    class phidgets_analog_4_output
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_analog_4_output(const phidgets_analog_4_output &src);


        protected:
        ros::NodeHandle nh_;

        //topics handlers
        ros::Publisher pub_;

        //service handlers
        ros::ServiceServer service_;

        public:

        //global device handler
        CPhidgetAnalogHandle phandle;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        bool initialised_;
        string device_label_;
        string topic_name_;
        string pfc_frame_;
        string set_service_name_;
        int wait_for_dev_delay_;
        int publish_frequency_;

        int ae_ch1;
        int ae_ch2;
        int ae_ch3;
        int ae_ch4;

        /** \brief Standard construktor.
         *
         */
        phidgets_analog_4_output(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_analog_4_output(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //publish data
        void publish_data();
        //service
        bool analog_4_output_set(rovitis_phidgets::phidget_analog_4_output_set::Request &req, rovitis_phidgets::phidget_analog_4_output_set::Response &res);
        //disconnecte phidgets
        void disconnect();

    };
}

#endif
