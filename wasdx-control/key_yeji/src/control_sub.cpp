//s 누르면 모든 변수 0

#include "ros/ros.h"

#include "key_yeji/key_msg.h"

#include "key_yeji/control_msg.h"

 

#define w 0x77 //119

#define d 0x64 //100

#define a 0x61 //97

#define s 0x73 //115

#define x 0x78 //120

 

int speed=0;

int steer=0;

int gear=1;

int temp_gear=1;

 

void callback(const key_yeji::key_msg::ConstPtr& msg)

{

    /*전후좌우, 리셋*/

    if(msg->key == 119) //w,전진

    {

        gear = 0;

        if(temp_gear==2)//후진에서 전진으로 넘어올 경우 멈췄다가 속도 올림

        {   speed = 0;

            speed += 5;

        }

        else

            speed+=5;

        temp_gear = 0;

    }

    

    else if(msg->key == 97) //a, 좌회전(전진 기준)

        steer -= 50;

    

    else if(msg->key == 115) //s, 후진

    {

        gear=2;

        if(temp_gear==0)

        {   speed = 0;

            speed += 5;

        }

        else

            speed +=5;

        temp_gear = 2;

    }

    

    else if(msg->key ==100) //d, 우회전(전진 기준)

        steer +=50;

        

    else if(msg->key ==120) //전부 0

    {

        speed=0;

        steer=0;

        gear=1;

        temp_gear = 1;

    }

    else printf("input wasdx!\n");

    

    /*최대최소 조건*/

    if(speed>=200) speed = 200;

    if(steer>=2000) steer =2000;

    else if(steer <=-2000) steer = -2000;

    

    ROS_INFO("speed: %d, steer: %d, gear: %d",speed,steer,gear);

}

 

int main(int argc, char **argv)

{

    ros::init(argc,argv,"control_sub");

    ros::NodeHandle nh;

    

    ros::Subscriber control = nh.subscribe("key",100,callback);

    ros::spin();

    

    ros::Publisher control_pub = nh.advertise<key_yeji::control_msg>("control",1);

    key_yeji::control_msg msg;

    

    while(ros::ok())

    {

        msg.write_gear = 0;
        msg.write_speed = speed;
        msg.write_steer = steer;
        msg.write_E_stop = 0x00;
        msg.write_brake = 1;

     

        control_pub.publish(msg);

    }

    return 0;

}
