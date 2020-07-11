/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_interface_kit_004.hpp"

int do1, do2, do3, do4;

namespace phidgets_interface_kit_004
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_interface_kit_004::phidgets_interface_kit_004(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 288557);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_interface_kit_004"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_ik"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 10);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_interface_kit_004"));
        private_nh.param<std::string>("set_service_name", set_service_name_, string("/phidgets_interface_kit_004_set"));
    }
    /****************************************************************
     *
     */
    phidgets_interface_kit_004::~phidgets_interface_kit_004()
    {
    }

    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_interface_kit_004::init()
    {
        //connect device
        ROS_INFO("Connecting device ... interface_kit_004");
        if (connect_dev())
        {
            ROS_INFO("Connection to interface_kit_004 successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no interface_kit_004 connected! Node shutdown!");
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
                pub_ = nh_.advertise<rovitis_phidgets::phidget_interface_kit_004>(topic_name_, 1);

                //create service
                service_ = nh_.advertiseService(set_service_name_, &phidgets_interface_kit_004::ik_set, this);

            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    bool phidgets_interface_kit_004::ik_set(rovitis_phidgets::phidget_interface_kit_004_set::Request &req, rovitis_phidgets::phidget_interface_kit_004_set::Response &res)
    {
        //ROS_INFO("Request received: serial %i index: %i value: %i", serial_number_, req.index, req.value);

        CPhidgetInterfaceKit_setOutputState(phandle, req.index, req.value);
        //ROS_INFO("Output %d State %d", req.index, req.value);

        return true;
    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached interface_kit_004 handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach interface_kit_004 handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event interface_kit_004: %s\n",errorStr);
        return 0;
    }

    void phidgets_interface_kit_004::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);

        int num_outputs;

        CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle)phid, &num_outputs);
        ROS_INFO("Number of digital outputs %d", num_outputs);
        return;
    }

    //callback that will run if an output changes.
    //Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
    int OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
    {
        //ROS_INFO("Digital output %d State %d", Index, State);
        if(Index == 0)
        {
            do1 = State;
        }
        if(Index == 1)
        {
            do2 = State;
        }
        if(Index == 2)
        {
            do3 = State;
        }
        if(Index == 3)
        {
            do4 = State;
        }
        return 0;
    }

    bool phidgets_interface_kit_004::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetInterfaceKit_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        CPhidgetInterfaceKit_set_OnOutputChange_Handler(phandle, OutputChangeHandler, NULL);

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

    void phidgets_interface_kit_004::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_interface_kit_004 data;

        data.header.frame_id = pfc_frame_;
        data.do_ch1 = do1;
        data.do_ch2 = do2;
        data.do_ch3 = do3;
        data.do_ch4 = do4;

        pub_.publish(data);
    }

    void phidgets_interface_kit_004::disconnect()
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
    phidgets_interface_kit_004::phidgets_interface_kit_004 phidgets_interface_kit_004_handler(nh);

    phidgets_interface_kit_004_handler.init();

    ros::Rate loop_rate(phidgets_interface_kit_004_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        phidgets_interface_kit_004_handler.publish_data();

        loop_rate.sleep();
    }

    phidgets_interface_kit_004_handler.disconnect();

    return 0;
}
