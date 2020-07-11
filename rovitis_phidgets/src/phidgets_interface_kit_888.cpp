/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_interface_kit_888.hpp"

int di1, di2, di3, di4, di5, di6, di7, di8;
int do1, do2, do3, do4, do5, do6, do7, do8;
double an1, an2, an3, an4, an5, an6, an7, an8;

namespace phidgets_interface_kit_888
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_interface_kit_888::phidgets_interface_kit_888(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 281641);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_interface_kit_888"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_ik"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 10);
        private_nh.param<std::string>("topic_name", topic_name_, string("/phidgets_interface_kit_888"));
        private_nh.param<std::string>("set_service_name", set_service_name_, string("/phidgets_interface_kit_888_set"));
    }
    /****************************************************************
     *
     */
    phidgets_interface_kit_888::~phidgets_interface_kit_888()
    {
    }

    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_interface_kit_888::init()
    {
        //connect device
        ROS_INFO("Connecting device ... interface_kit_888");
        if (connect_dev())
        {
            ROS_INFO("Connection to interface_kit_888 successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no interface_kit_888 connected! Node shutdown!");
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
                pub_ = nh_.advertise<rovitis_phidgets::phidget_interface_kit_888>(topic_name_, 1);

                //create service
                service_ = nh_.advertiseService(set_service_name_, &phidgets_interface_kit_888::ik_set, this);

            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    bool phidgets_interface_kit_888::ik_set(rovitis_phidgets::phidget_interface_kit_888_set::Request &req, rovitis_phidgets::phidget_interface_kit_888_set::Response &res)
    {
        //ROS_INFO("Request received:  index: %i value: %i", req.index, req.value);

        CPhidgetInterfaceKit_setOutputState(phandle, req.index, req.value);
        //ROS_INFO("Output %d State %d", req.index, req.value);

        return true;
    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        ROS_INFO("Run attached interface_kit_888 handler!");
        return 0;
    }

    int CCONV DetachHandler(CPhidgetHandle phid, void *userptr)
    {
        ROS_INFO("Detach interface_kit_888 handler!");
        return 0;
    }

    int CCONV ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
    {
        ROS_ERROR("Error event interface_kit_888: %s\n",errorStr);
        return 0;
    }

    void phidgets_interface_kit_888::display_generic_properties(CPhidgetHandle phid)
    {
        int sernum, version;
        const char *deviceptr;
        CPhidget_getDeviceType(phid, &deviceptr);
        CPhidget_getSerialNumber(phid, &sernum);
        CPhidget_getDeviceVersion(phid, &version);
        curr_serial_number_ = sernum;
        ROS_INFO("Device Name: %s Version: %d SerialNumber: %d",deviceptr, version, sernum);

        int ratiometric, num_sensors, num_inputs, num_outputs, triggerVal;

        CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle)phid, &num_inputs);
        CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle)phid, &num_outputs);
        CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle)phid, &num_sensors);
        CPhidgetInterfaceKit_getRatiometric((CPhidgetInterfaceKitHandle)phid, &ratiometric);

        //ROS_INFO("Number of digital inputs %d", num_inputs);
        //ROS_INFO("Number of digital outputs %d", num_outputs);
        //ROS_INFO("Number of sensors %d", num_sensors);
        //ROS_INFO("Ratiometric %d", ratiometric);

        for (int i = 0; i < num_sensors; i++)
        {
            CPhidgetInterfaceKit_getSensorChangeTrigger ((CPhidgetInterfaceKitHandle)phid, i, &triggerVal);
            //CPhidgetInterfaceKit_setSensorChangeTrigger (phid, i, 10);


            //ROS_INFO("Sensor %d Sensitivity Trigger %d", i, triggerVal);
        }

        return;
    }

    //callback that will run if an input changes.
    //Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
    int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
    {
        //ROS_INFO("Digital input %d State %d", Index, State);
        if(Index == 0)
        {
            di1 = State;
        }
        if(Index == 1)
        {
            di2 = State;
        }
        if(Index == 2)
        {
            di3 = State;
        }
        if(Index == 3)
        {
            di4 = State;
        }
        if(Index == 4)
        {
            di5 = State;
        }
        if(Index == 5)
        {
            di6 = State;
        }
        if(Index == 6)
        {
            di7 = State;
        }
        if(Index == 7)
        {
            di8 = State;
        }
        return 0;
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
        if(Index == 4)
        {
            do5 = State;
        }
        if(Index == 5)
        {
            do6 = State;
        }
        if(Index == 6)
        {
            do7 = State;
        }
        if(Index == 7)
        {
            do8 = State;
        }
        return 0;
    }

    //callback that will run if the sensor value changes by more than the OnSensorChange trigger.
    //Index - Index of the sensor that generated the event, Value - the sensor read value
    int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
    {

        //ROS_INFO("Sensor %d Value %d", Index, Value);
        if(Index == 0)
        {
            an1 = Value;
        }
        if(Index == 1)
        {
            an2 = Value;
        }
        if(Index == 2)
        {
            an3 = Value;
        }
        if(Index == 3)
        {
            an4 = Value;
        }
        if(Index == 4)
        {
            an5 = Value;
        }
        if(Index == 5)
        {
            an6 = Value;
        }
        if(Index == 6)
        {
            an7 = Value;
        }
        if(Index == 7)
        {
            an8 = Value;
        }
        return 0;
    }

    bool phidgets_interface_kit_888::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetInterfaceKit_create(&phandle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)phandle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)phandle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)phandle, ErrorHandler, NULL);

        CPhidgetInterfaceKit_set_OnInputChange_Handler(phandle, InputChangeHandler, NULL);
        CPhidgetInterfaceKit_set_OnSensorChange_Handler(phandle, SensorChangeHandler, NULL);
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

    void phidgets_interface_kit_888::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_interface_kit_888 data;

        data.header.frame_id = pfc_frame_;
        data.di_ch1 = di1;
        data.di_ch2 = di2;
        data.di_ch3 = di3;
        data.di_ch4 = di4;
        data.di_ch5 = di5;
        data.di_ch6 = di6;
        data.di_ch7 = di7;
        data.di_ch8 = di8;

        data.do_ch1 = do1;
        data.do_ch2 = do2;
        data.do_ch3 = do3;
        data.do_ch4 = do4;
        data.do_ch5 = do5;
        data.do_ch6 = do6;
        data.do_ch7 = do7;
        data.do_ch8 = do8;

        data.an_ch1 = an1;
        data.an_ch2 = an2;
        data.an_ch3 = an3;
        data.an_ch4 = an4;
        data.an_ch5 = an5;
        data.an_ch6 = an6;
        data.an_ch7 = an7;
        data.an_ch8 = an8;

        pub_.publish(data);
    }

    void phidgets_interface_kit_888::disconnect()
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
    phidgets_interface_kit_888::phidgets_interface_kit_888 phidgets_interface_kit_888_handler(nh);

    phidgets_interface_kit_888_handler.init();

    ros::Rate loop_rate(phidgets_interface_kit_888_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        phidgets_interface_kit_888_handler.publish_data();

        loop_rate.sleep();
    }

    phidgets_interface_kit_888_handler.disconnect();

    return 0;
}
