/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_high_speed_encoder.hpp"

int position_ch1;
int position_ch2;
int position_ch3;
int position_ch4;

namespace phidgets_high_speed_encoder
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_high_speed_encoder::phidgets_high_speed_encoder(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    phandle(0)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 54494); // 54405
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_high_speed_encoder"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_hs"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 10);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_high_speed_encoder"));
    }
    /****************************************************************
     *
     */
    phidgets_high_speed_encoder::~phidgets_high_speed_encoder()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_high_speed_encoder::init()
    {
        //connect device
        ROS_INFO("Connecting device ... high_speed_encoder");
        if (connect_dev())
        {
            ROS_INFO("Connection to high_speed_encoder successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no high_speed_encoder connected! Node shutdown!");
            initialised_ = false;

            CPhidget_close((CPhidgetHandle)phandle);
            CPhidget_delete((CPhidgetHandle)phandle);

            ros::shutdown();
        }

        if(initialised_)
        {
            if(serial_number_ == curr_serial_number_)
            {
                //set device label
                const char* cname = device_label_.c_str();
                CPhidget_setDeviceLabel((CPhidgetHandle)phandle, cname);

                //create topics to publish data
                pcf_pub_ = nh_.advertise<rovitis_phidgets::phidget_high_speed_encoder>(topic_name_, 1);

            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached high_speed_encoder handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach high_speed_encoder handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event high_speed_encoder: %s\n",errorStr);
        return 0;
    }

    void phidgets_high_speed_encoder::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);
        return;
    }

    int PositionChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int Time, int RelativePosition)
    {
        int Position;
        CPhidgetEncoder_getPosition(ENC, Index, &Position);
        //ROS_INFO("Encoder %d Count %d", Index, Position);
	//ROS_INFO("Phidgets ENC: Relative pose: %i", RelativePosition);

        if(Index == 0)
        {
            position_ch1 = Position;
        }
        if(Index == 1)
        {
            position_ch2 = Position;
        }
        if(Index == 2)
        {
            position_ch3 = Position;
        }
        if(Index == 3)
        {
            position_ch4 = Position;
        }

        return 0;
    }

    int InputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State)
    {
        int Position;
        CPhidgetEncoder_getPosition(ENC, Index, &Position);
        //ROS_INFO("Encoder %d Count %d", Index, Position);

        if(Index == 0)
        {
            position_ch1 = Position;
        }
        if(Index == 1)
        {
            position_ch2 = Position;
        }
        if(Index == 2)
        {
            position_ch3 = Position;
        }
        if(Index == 3)
        {
            position_ch4 = Position;
        }

        return 0;
    }

    bool phidgets_high_speed_encoder::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetEncoder_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        //register callbacks
        CPhidgetEncoder_set_OnInputChange_Handler(phandle, InputChangeHandler, NULL);
        CPhidgetEncoder_set_OnPositionChange_Handler(phandle, PositionChangeHandler, NULL);

        CPhidget_open((CPhidgetHandle)phandle, serial_number_);

        //Wait for 10 seconds, otherwise exit
        if(result = CPhidget_waitForAttachment((CPhidgetHandle)phandle, wait_for_dev_delay_))
        {
            CPhidget_getErrorDescription(result, &err);
            printf("Problem waiting for attachment: %s\n", err);
            return false;
        }

        display_generic_properties((CPhidgetHandle)phandle);
        return true;
    }

    void phidgets_high_speed_encoder::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_high_speed_encoder data;

        data.header.frame_id = pfc_frame_;

        data.count_ch1 = position_ch1;
        data.count_ch2 = position_ch2;
        data.count_ch3 = position_ch3;
        data.count_ch4 = position_ch4;

        pcf_pub_.publish(data);
    }

    void phidgets_high_speed_encoder::disconnect()
    {
        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)phandle);
        CPhidget_delete((CPhidgetHandle)phandle);
    }

    void phidgets_high_speed_encoder::reset_encoder()
    {
        ROS_INFO("Reseting encoder...");
        CPhidgetEncoder_setPosition(phandle, 0, 0);
        CPhidgetEncoder_setPosition(phandle, 1, 0);
        CPhidgetEncoder_setPosition(phandle, 2, 0);
        CPhidgetEncoder_setPosition(phandle, 3, 0);
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_frequency_counter");
    ros::NodeHandle nh;
    bool reset;

    //For node registering
    phidgets_high_speed_encoder::phidgets_high_speed_encoder phidgets_high_speed_encoder_handler(nh);

    //initialization
    phidgets_high_speed_encoder_handler.init();

    ros::Rate loop_rate(phidgets_high_speed_encoder_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_high_speed_encoder_handler.publish_data();

       /* nh.getParam("/odom_init", reset);
        if (reset)
        {
            phidgets_high_speed_encoder_handler.reset_encoder();
            nh.setParam("/odom_init",false);
        }
        //ROS_INFO("Reset status: %i", reset);*/
    }

    phidgets_high_speed_encoder_handler.disconnect();

    return 0;
}
