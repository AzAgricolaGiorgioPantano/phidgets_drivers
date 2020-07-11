/** \brief
 *  \file phidgets_interface_kit_888.hpp
 *  \author Peter Lepej
 *  \date 21.9.2015
 *  \version 1.0
 *  \place: FKBV, Pivola 10, Hoče, Slovenia
 */

#include "rovitis_phidgets/phidgets_frequency_counter.hpp"

bool enable_ch0;
bool enable_ch1;
string filter_ch0;
string filter_ch1;
bool reset;
float fre_ch0;
float fre_ch1;
int rpm_ch0;
int rpm_ch1;

#define SIZE_BUFFER 9

bool first_ch0 = true;
int count_first_ch0 = 0;
int max_ind_ch0 = 0;
double sum_value_ch0 = 0;
double mean_value_ch0 = 0;
int rpm_int_ch0;

int buffer_rpm_ch0[SIZE_BUFFER][2];

bool first_ch1 = true;
int count_first_ch1 = 0;
int max_ind_ch1 = 0;
double sum_value_ch1 = 0;
double mean_value_ch1 = 0;
int rpm_int_ch1;

int buffer_rpm_ch1[SIZE_BUFFER][2];



namespace phidgets_frequency_counter
{
    /****************************************************************
     * Here we can set up the parameters
     */
    phidgets_frequency_counter::phidgets_frequency_counter(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
    pfc_handle(0)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<int>("dev_serial_number", serial_number_, 137778);
        private_nh.param<std::string>("dev_serial_label", device_label_, string("phidgets_frequency_counter"));
        private_nh.param<std::string>("frame", pfc_frame_, string("/phidget_fc"));
        private_nh.param<int>("wait_for_dev_delay", wait_for_dev_delay_, 10000);
        private_nh.param<int>("publish_frequency", publish_frequency_, 10);
        private_nh.param<std::string>("topic_name", pfc_topic_name_, string("/phidget_frequency_counter"));
        private_nh.param<std::string>("reset_service_name", pfc_reset_service_name_, string("phidgets_frequency_counter_reset"));
        private_nh.param<bool>("enable_ch0", enable_ch0_, true);
        private_nh.param<bool>("enable_ch1", enable_ch1_, true);
        private_nh.param<std::string>("filter_ch0", filter_ch0_, string("zero_crossing"));     //zero_crossing
        private_nh.param<std::string>("filter_ch1", filter_ch1_, string("zero_crossing"));   //logic_level
    }
    /****************************************************************
     *
     */
    phidgets_frequency_counter::~phidgets_frequency_counter()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void phidgets_frequency_counter::init()
    {
        //initialize parameters from launch file
        enable_ch0 = enable_ch0_;
        enable_ch1 = enable_ch1_;
        filter_ch0 = filter_ch0_;
        filter_ch1 = filter_ch1_;

        //connect device
        ROS_INFO("Connecting device ... frequency_counter");
        if (connect_dev())
        {
            ROS_INFO("Connection to frequency_counter successfull!");
            initialised_ = true;
        }
        else
        {
            ROS_ERROR("There is no frequency_counter connected! Node shutdown!");
            initialised_ = false;

            CPhidget_close((CPhidgetHandle)pfc_handle);
            CPhidget_delete((CPhidgetHandle)pfc_handle);

            ros::shutdown();
        }

        if(initialised_)
        {
            if(serial_number_ == curr_serial_number_)
            {
                //set device label
                const char* cname = device_label_.c_str();
                CPhidget_setDeviceLabel((CPhidgetHandle)pfc_handle, cname);

                //create service
                reset_service_ = nh_.advertiseService(pfc_reset_service_name_, &phidgets_frequency_counter::pfc_reset, this); //&BallBinSegmentor::serviceCallback, this

                //create topics to publish data
                pcf_pub_ = nh_.advertise<rovitis_phidgets::phidget_frequency_counter>(pfc_topic_name_, 1);

            }else
            {
                ROS_ERROR("Defined device serial number: %i dose not match to current connected device serial number: %i!", serial_number_,curr_serial_number_);
            }
        }
    }

    bool phidgets_frequency_counter::pfc_reset(rovitis_phidgets::phidget_frequency_counter_reset::Request &req, rovitis_phidgets::phidget_frequency_counter_reset::Response &res)
    {
        ROS_INFO("Request received: %d", (bool)req.reset);
        reset_ = (bool)req.reset;
        if(reset_)
        {
            CPhidgetFrequencyCounter_reset(pfc_handle, 0);
            CPhidgetFrequencyCounter_reset(pfc_handle, 1);
            ROS_INFO("Phidget Frequncy Board Channels Reset");
        }
        return true;
    }

    int CCONV AttachHandler(CPhidgetHandle phid, void *userptr)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;

        if(enable_ch0)
        {
            CPhidgetFrequencyCounter_setEnabled(pfc_handle, 0, PTRUE);

            if(filter_ch0 == "logic_level")
            {
                CPhidgetFrequencyCounter_setFilter(pfc_handle, 0, PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_LOGIC_LEVEL);
                ROS_INFO("Channel 0 enabled, filter: LOGIC LEVEL!");
            }
            if(filter_ch0 == "zero_crossing")
            {
                CPhidgetFrequencyCounter_setFilter(pfc_handle, 0, PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_ZERO_CROSSING);
                ROS_INFO("Channel 0 enabled, filter: ZERO_CROSSING!");
            }
        }
        if(enable_ch1)
        {
            CPhidgetFrequencyCounter_setEnabled(pfc_handle, 1, PTRUE);

            if(filter_ch1 == "logic_level")
            {
                CPhidgetFrequencyCounter_setFilter(pfc_handle, 1, PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_LOGIC_LEVEL);
                ROS_INFO("Channel 1 enabled, filter: LOGIC LEVEL!");
            }
            if(filter_ch1 == "zero_crossing")
            {
                CPhidgetFrequencyCounter_setFilter(pfc_handle, 1, PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_ZERO_CROSSING);
                ROS_INFO("Channel 1 enabled, filter: ZERO_CROSSING!");
            }
        }

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
        ROS_ERROR("Error event frequency_counter: %s\n",errorStr);
        return 0;
    }

    #define FREQS_SIZE 5
    double freqs[FREQS_SIZE] = {0};
    int CCONV Count(CPhidgetFrequencyCounterHandle phid, void *userPtr, int index, int time, int counts)
    {
        CPhidgetFrequencyCounterHandle pfc_handle = (CPhidgetFrequencyCounterHandle)phid;
        double f, ms0, ms1;
        int i;

        if (index == 0)
        {
            CPhidgetFrequencyCounter_getFrequency(pfc_handle, index, &f);
            ms0 = time/1000.0;

            //Smooth out the frequency a bit
            for(i=0;i<FREQS_SIZE-1;i++)
            {
                freqs[i] = freqs[i+1];
            }
            freqs[FREQS_SIZE-1] = f;
            f=0;
            for(i=0;i<FREQS_SIZE;i++)
            {
                f+=freqs[i];
            }
            f/=FREQS_SIZE;

            int ms0_int = (int)(ms0 < 0 ? (ms0 - 0.5) : (ms0 + 0.5));

            if (counts == 1 && ms0_int > 35 && ms0_int < 70)
            {
                if (first_ch0 == true)
                {
                    // calcola media primi cinque valori
                    sum_value_ch0 += ms0_int;

                    mean_value_ch0 = (sum_value_ch0/(count_first_ch0+1))+0.5;

                    for(int i = 0; i < SIZE_BUFFER ; i++)
                    {
                        buffer_rpm_ch0[i][0] = (int)mean_value_ch0;
                    }

                    if (count_first_ch0 + 1 == SIZE_BUFFER)
                            first_ch0 = false;

                     count_first_ch0++;

                }
                else
                {

                    for(int i = SIZE_BUFFER - 2 ; i>=0 ; i--)
                    {
                        buffer_rpm_ch0[i+1][0] = buffer_rpm_ch0[i][0];
                    }

                    buffer_rpm_ch0[0][0] = ms0_int;

                    //azzeramento contatori
                    for(int i = 0 ; i < SIZE_BUFFER ; i++)
                    {
                        buffer_rpm_ch0[i][1] = 0;
                    }


                    // conteggio frequenza numeri
                    for (int i = 0; i < SIZE_BUFFER; i++)
                    {
                       for (int j = 0; j < SIZE_BUFFER; j++)
                       {
                         if(buffer_rpm_ch0[i][0]==buffer_rpm_ch0[j][0])
                            buffer_rpm_ch0[i][1] = buffer_rpm_ch0[i][1] + 1;
                       }
                    }

                    int max_value = buffer_rpm_ch0[0][1];

                    for(int i = 0 ; i < SIZE_BUFFER ; i++)
                    {
                        //cout<<"Math "<<buffer_rpm_ch0[i][0]<<" "<<buffer_rpm_ch0[i][1]<<endl;

                        if(buffer_rpm_ch0[i][1]>max_value)
                        {
                            max_value   = buffer_rpm_ch0[i][1];
                            max_ind_ch0 = i;
                        }
                    }

                     //cout<<"Filtered value: "<<buffer_rpm_ch0[max_ind_ch0][0]<<endl;

                     double rpm = 60000/buffer_rpm_ch0[max_ind_ch0][0];
                     rpm_int_ch0 = (int)rpm;

                }

                if (abs(rpm_ch0 - rpm_int_ch0) > 3)
                ROS_INFO("ch0 1 counts in ;%d;ms0 - ;%0.2lf;Hz filter;%i; rpm; %i",ms0_int,f,buffer_rpm_ch0[max_ind_ch0][0],rpm_int_ch0);

                fre_ch0 = f;
                rpm_ch0 = rpm_int_ch0;
            }
        }

        if (index == 1 )
        {

            CPhidgetFrequencyCounter_getFrequency(pfc_handle, index, &f);
            ms1 = time/1000.0;

            int ms1_int = (int)(ms1 < 0 ? (ms1 - 0.5) : (ms1 + 0.5));

            if (first_ch1 == true)
            {
                // calcola media primi cinque valori
                sum_value_ch1 += ms1_int;

                mean_value_ch1 = (sum_value_ch1/(count_first_ch1+1))+0.5;

                for(int i = 0; i < SIZE_BUFFER ; i++)
                {
                    buffer_rpm_ch1[i][0] = (int)mean_value_ch1;
                }

                if (count_first_ch1 + 1 == SIZE_BUFFER)
                        first_ch1 = false;

                 count_first_ch1++;

            }
            else
            {

                for(int i = SIZE_BUFFER - 2 ; i>=0 ; i--)
                {
                    buffer_rpm_ch1[i+1][0] = buffer_rpm_ch1[i][0];
                }

                buffer_rpm_ch1[0][0] = ms1_int;

                //azzeramento contatori
                for(int i = 0 ; i < SIZE_BUFFER ; i++)
                {
                    buffer_rpm_ch1[i][1] = 0;
                }


                // conteggio frequenza numeri
                for (int i = 0; i < SIZE_BUFFER; i++)
                {
                   for (int j = 0; j < SIZE_BUFFER; j++)
                   {
                     if(buffer_rpm_ch1[i][0]==buffer_rpm_ch1[j][0])
                        buffer_rpm_ch1[i][1] = buffer_rpm_ch1[i][1] + 1;
                   }
                }

                int max_value = buffer_rpm_ch1[0][1];

                for(int i = 0 ; i < SIZE_BUFFER ; i++)
                {
                    //cout<<"Math "<<buffer_rpm_ch1[i][0]<<" "<<buffer_rpm_ch1[i][1]<<endl;

                    if(buffer_rpm_ch1[i][1]>max_value)
                    {
                        max_value   = buffer_rpm_ch1[i][1];
                        max_ind_ch1 = i;
                    }
                }

                 //cout<<"Filtered value: "<<buffer_rpm_ch1[max_ind_ch1][0]<<endl;

                 double rpm = 60000/buffer_rpm_ch1[max_ind_ch1][0];
                 rpm_int_ch1 = (int)rpm;


            }

            ROS_INFO("ch1 1 counts in ;%d;ms1 - ;%0.2lf;Hz filter;%i; rpm; %i",ms1_int,f,buffer_rpm_ch1[max_ind_ch1][0],rpm_int_ch1);

            fre_ch1 = f;
            rpm_ch1 = rpm_int_ch1;

        }

        return 0;
    }

    void phidgets_frequency_counter::display_generic_properties(CPhidgetHandle phid)
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

    bool phidgets_frequency_counter::connect_dev()
    {

        const char *err;
        int result;

        CPhidgetFrequencyCounter_create(&pfc_handle);

        CPhidget_set_OnAttach_Handler((CPhidgetHandle)pfc_handle, AttachHandler, NULL);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)pfc_handle, DetachHandler, NULL);
        CPhidget_set_OnError_Handler((CPhidgetHandle)pfc_handle, ErrorHandler, NULL);

        CPhidgetFrequencyCounter_set_OnCount_Handler(pfc_handle, Count, NULL);

        CPhidget_open((CPhidgetHandle)pfc_handle, serial_number_);

        //Wait for 10 seconds, otherwise exit
        if(result = CPhidget_waitForAttachment((CPhidgetHandle)pfc_handle, wait_for_dev_delay_))
        {

            CPhidget_getErrorDescription(result, &err);
            printf("Problem waiting for attachment: %s\n", err);
            return false;
        }

        display_generic_properties((CPhidgetHandle)pfc_handle);
        return true;
    }

    void phidgets_frequency_counter::publish_data()
    {
        //publish data
        rovitis_phidgets::phidget_frequency_counter data;

        data.header.frame_id = pfc_frame_;
        data.frequency_ch0 = fre_ch0;
        data.frequency_ch1 = fre_ch1;
        data.rpm_ch0 = rpm_ch0;
        data.rpm_ch1 = rpm_ch1;

        pcf_pub_.publish(data);
    }

    void phidgets_frequency_counter::disconnect()
    {
        ROS_INFO("Closing...");
        CPhidget_close((CPhidgetHandle)pfc_handle);
        CPhidget_delete((CPhidgetHandle)pfc_handle);
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "phidgets_frequency_counter");
    ros::NodeHandle nh;

    //For node registering
    phidgets_frequency_counter::phidgets_frequency_counter phidgets_frequency_counter_handler(nh);

    //initialization
    phidgets_frequency_counter_handler.init();

    ros::Rate loop_rate(phidgets_frequency_counter_handler.publish_frequency_);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        phidgets_frequency_counter_handler.publish_data();
    }

    phidgets_frequency_counter_handler.disconnect();

    return 0;
}
