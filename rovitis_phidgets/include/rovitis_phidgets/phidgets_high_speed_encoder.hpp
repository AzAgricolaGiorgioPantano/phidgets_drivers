/** \brief
 *  \file phidgets_interface_kit_888.cpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#ifndef phidgets_high_speed_encoder_hpp___
#define phidgets_high_speed_encoder_hpp___ //header is included only once

//basic
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <sstream>

//phidgets stuff
#include <libphidgets/phidget21.h>
//#include <phidgets_api/phidget.h>

////messagess
#include "rovitis_phidgets/phidget_high_speed_encoder.h"

using namespace std;

namespace phidgets_high_speed_encoder  //split up code more classes whit the same name
{
    class phidgets_high_speed_encoder
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        phidgets_high_speed_encoder(const phidgets_high_speed_encoder &src);

        protected:
        ros::NodeHandle nh_;

        //service handlers
        ros::ServiceServer reset_service_;

        //topics handlers
        ros::Publisher pcf_pub_;

        public:

        //global device handler
        CPhidgetEncoderHandle phandle;

        //global arguments
        int serial_number_;
        int curr_serial_number_;
        bool initialised_;
        string device_label_;
        string topic_name_;
        string pfc_frame_;
        int wait_for_dev_delay_;
        int publish_frequency_;
        bool reset_;

        /** \brief Standard construktor.
         *
         */
        phidgets_high_speed_encoder(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~phidgets_high_speed_encoder(); //always virtual

        //initialization function
        void init();
        //connect device
        bool connect_dev();
        //display device info
        void display_generic_properties(CPhidgetHandle phid);
        //publish data
        void publish_data();
        //disconnecte phidgets
        void disconnect();
        //reset phidgets
        void reset_encoder();
    };
}

#endif
