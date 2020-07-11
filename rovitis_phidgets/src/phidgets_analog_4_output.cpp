/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_analog_4_output.hpp"

double av1, av2, av3, av4;
int ae1, ae2, ae3, ae4;

namespace phidgets_analog_4_output
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_analog_4_output::phidgets_analog_4_output(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 275374);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_analog_4_output"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_ik"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 10);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_analog_4_output"));
        private_nh.param<std::string>("set_service_name", set_service_name_, string("/phidgets_analog_4_output_set"));

        private_nh.param<int>("enable_ch1", ae_ch1, 1);
        private_nh.param<int>("enable_ch2", ae_ch2, 1);
        private_nh.param<int>("enable_ch3", ae_ch3, 1);
        private_nh.param<int>("enable_ch4", ae_ch4, 1);
    }
    /****************************************************************
     *
     */
    phidgets_analog_4_output::~phidgets_analog_4_output()
    {
    }

    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_analog_4_output::init()
    {
        //connect device
        ROS_INFO("Connecting device ... analog_4_output");
        if (connect_dev())
        {
            ROS_INFO("Connection to analog_4_output successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no analog_4_output connected! Node shutdown!");
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
                pub_ = nh_.advertise<rovitis_phidgets::phidget_analog_4_output>(topic_name_, 1);

                //create service
                service_ = nh_.advertiseService(set_service_name_, &phidgets_analog_4_output::analog_4_output_set, this);

            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    bool phidgets_analog_4_output::analog_4_output_set(rovitis_phidgets::phidget_analog_4_output_set::Request &req, rovitis_phidgets::phidget_analog_4_output_set::Response &res)
    {

        //ROS_INFO("Request received:  index: %i value: %i", req.index, req.value);

        //ROS_INFO("Output %d Enabled %i Value: %f", req.index, req.enable, req.value);

        //enable channels
        CPhidgetAnalog_setEnabled(phandle, (int)req.index, (int)req.enable);
        CPhidgetAnalog_setVoltage(phandle, (int)req.index, (double)req.value);

        if(req.index == 0)
        {
            av1 = req.value;
        }
        if(req.index == 1)
        {
            av2 = req.value;
        }
        if(req.index == 2)
        {
            av3 = req.value;
        }
        if(req.index == 3)
        {
            av4 = req.value;
        }

            return true;
        }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached analog_4_output handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach analog_4_output handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event analog_4_output: %s\n",errorStr);
        return 0;
    }

    void phidgets_analog_4_output::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);

        int num_outputs;

        CPhidgetAnalog_getOutputCount((CPhidgetAnalogHandle)phid, &num_outputs);
        //ROS_INFO("Number of digital outputs %d", num_outputs);

        int enabled;
        double max, min;
        for(int i=0; i < 4; i++)
        {
            CPhidgetAnalog_getEnabled((CPhidgetAnalogHandle)phid, i, &enabled);
            CPhidgetAnalog_getVoltageMax((CPhidgetAnalogHandle)phid, i, &max);
            CPhidgetAnalog_getVoltageMin((CPhidgetAnalogHandle)phid, i, &min);
            ROS_INFO("Max voltage: %f min voltage: %f", max, min);

        }

        return;
    }

    bool phidgets_analog_4_output::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetAnalog_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        CPhidget_open((CPhidgetHandle)phandle, serial_number_);

        //Wait for 10 seconds, otherwise exit
        if(result = CPhidget_waitForAttachment((CPhidgetHandle)phandle, wait_for_dev_delay_))
        {

            CPhidget_getErrorDescription(result, &err);
            printf("Problem waiting for attachment: %s\n", err);
            return false;
        }

        //enable channels

       if(ae_ch1 == 1)
       {
           ae1 = ae_ch1;
       }
       if(ae_ch2 == 1)
       {
           ae2 = ae_ch2;
       }
       if(ae_ch3 == 1)
       {
           ae3 = ae_ch3;
       }
       if(ae_ch4 == 1)
       {
           ae4 = ae_ch4;
       }

        display_generic_properties((CPhidgetHandle)phandle);

        return true;
    }

    void phidgets_analog_4_output::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_analog_4_output data;

        data.header.frame_id = pfc_frame_;
        data.av_ch1 = av1;
        data.av_ch2 = av2;
        data.av_ch3 = av3;
        data.av_ch4 = av4;
        data.ae_ch1 = ae1;
        data.ae_ch2 = ae2;
        data.ae_ch3= ae3;
        data.ae_ch4 = ae4;

        pub_.publish(data);
    }

    void phidgets_analog_4_output::disconnect()
    {
        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)phandle);
        CPhidget_delete((CPhidgetHandle)phandle);
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_interface_kit_888");
    ros::NodeHandle nh;

    //for registering the node
    phidgets_analog_4_output::phidgets_analog_4_output phidgets_analog_4_output_handler(nh);

    phidgets_analog_4_output_handler.init();

    ros::Rate loop_rate(phidgets_analog_4_output_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        phidgets_analog_4_output_handler.publish_data();

        loop_rate.sleep();
    }

    phidgets_analog_4_output_handler.disconnect();

    return 0;
}
