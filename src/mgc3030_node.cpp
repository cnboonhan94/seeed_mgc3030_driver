extern "C" {
    #include <curses.h>
    #include "src/platform_basic_func.h"
    #include "src/Seeed_3D_touch_mgc3030.h"
}

#include "ros/ros.h"
#include "std_msgs/String.h"    
#include "geometry_msgs/PointStamped.h"    
#include <sstream>

extern Sensor_info_t mgc_info;

uint8_t ges_cnt;
uint8_t pos_cnt;
uint8_t air_wheel_cnt;
uint8_t touch_cnt;


static const char *touch_info[16] ={"South","West","North","East","Center","South","West","North","East","Center","South",
"West","North","East","Center"};

void clear_data_periodly(void)
{
    static uint32_t debug_cnt = 0;

    ges_cnt++;
    pos_cnt++;
    air_wheel_cnt++;
    touch_cnt++;

    debug_cnt++;
    uint8_t act_flag = 0;

    if(ges_cnt >= 10 ){
        ges_cnt = 0;
         act_flag = 1;
    }
    if(pos_cnt >= 10){
         pos_cnt = 0;
          act_flag = 1;
    }
    if(air_wheel_cnt >= 10){
         air_wheel_cnt = 0;
         act_flag = 1;
    }
    if(touch_cnt >= 10){
        touch_cnt = 0;
        act_flag = 1;
    }
    if(act_flag){
        refresh();
    }
}

void publish_location_xyz(uint8_t *data, ros::Publisher *pub)
{
    uint16_t x=0,y=0,z=0;
    
    x = data[GESTIC_XYZ_DATA+1] << 8 | data[GESTIC_XYZ_DATA];
    y = data[GESTIC_XYZ_DATA+3] << 8 | data[GESTIC_XYZ_DATA+2];
    z = data[GESTIC_XYZ_DATA+5] << 8 | data[GESTIC_XYZ_DATA+4];

    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq++;
    msg.header.frame_id = "map";

    msg.point.x = x / 65535.0 * 0.1;
    msg.point.y = y / 65535.0 * 0.1;
    msg.point.z = z / 65535.0 * 0.1;
    pub->publish(msg);
}

void publish_touch(uint8_t gesture, ros::Publisher *pub)
{
    if(gesture >= 0 && gesture < 5)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Touch: " << touch_info[gesture];
        msg.data = ss.str();
        pub->publish(msg);
    }
    else if(gesture >= 5 && gesture < 10)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Tap: " << touch_info[gesture];
        msg.data = ss.str();
        pub->publish(msg);
    }
    else if(gesture >= 10 && gesture < 15)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Double Tap: " << touch_info[gesture];
        msg.data = ss.str();
        pub->publish(msg);
    }
    else{

    }
    touch_cnt = 0;
}

void publish_touch_event(uint8_t *data, ros::Publisher *pub)
{
    uint16_t touch_action = 0;
    touch_action = data[10] | data[11] << 8;
    for(int i=0;i<15;i++){
        if(touch_action & 1<<i){
            publish_touch(i, pub);
        }
    }
}

int32_t publish_sensor_msg(uint8_t *data, ros::Publisher *pos_pub, ros::Publisher *touch_pub)
{   
    uint8_t *payload = &data[4];
    /*0x91,indicate sensor data output!*/
    if(SENSOR_OUTPUT_DATA == data[3] ){
        uint8_t action = payload[3];
        
        /*position data */
        if((payload[0] & 1<<4) && (action & 0x01)){
            publish_location_xyz(payload, pos_pub);
        }
        /* touch data */
        if(payload[0] & 0x04){
            publish_touch_event(payload, touch_pub);
        }
        refresh();
    }
    else{
        return -1;
    }
    return 0;
}

static uint8_t data[MAX_RECV_LEN];
int main(int argc, char **argv)
{
    //int32_t ret = 0;

    /*hardware init(i2c gpio...)*/
    if(basic_init()){
        printf("Basic init failed!!\n");
        return -1;
    }

    read_version_info(data);

    if(calibration_select(DISABLE)){
        printf("Select calibration failed!\n");
        mgc_exit();
        return -1;
    }


    if(set_lock_mask()){
        printf("Set lock mask failed!\n");
        mgc_exit();
        return -1;
    }
    mgc3030_init();


    /* ros initialization */
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PointStamped>("mgc3030/pos", 100);
    ros::Publisher touch_pub = n.advertise<std_msgs::String>("mgc3030/touch", 100);

    while(ros::ok()){
        // Nothing is sensed
        if(mg3030_read_data(data) < 3)
        {
            ROS_INFO("Waiting for input...");
            usleep(50000);
            clear_data_periodly();
            continue;
        }

        // Something is sensed
        ROS_INFO("Input is Found.");
        publish_sensor_msg(data, &pos_pub, &touch_pub);
        memset(data,0,sizeof(data));
        usleep(50000);
        clear_data_periodly();
    }

    return 1;
}
