#include "eprobot_joy.h"



eprobot_joy::eprobot_joy()
{
    joystick_fd = -1;
    leftVertical    = rightVertical  = 0;
    rightHorizontal = leftHorizontal = 0;

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("joystick_device", joystick_device, "/dev/input/js0");
    nh_private.param<float>("maxLinear_x", maxLinear_x, 0.4);
    nh_private.param<float>("maxLinear_y", maxLinear_y, 0.4);
    nh_private.param<float>("maxAngular_z", maxAngular_z, 1.5);
	nh_private.param<int>("joystick_mode", joystick_mode, JS_MODE_1);

    ros::NodeHandle n;
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    joystick_fd = open(joystick_device.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
    if (joystick_fd < 0)
    {
        ROS_INFO("Open joystick device success!");
    }
}

 eprobot_joy::~eprobot_joy()
{
    close(joystick_fd);
}


void eprobot_joy::publish_joystick_event()
{
    ros::Rate rosSleep(20);
    while(ros::ok())
    {
        int bytes = read(joystick_fd, &jse, sizeof(jse));
        if(bytes != sizeof(jse) && bytes != -1)
        {
            ROS_INFO("Read joystick file sizeof error!");
        }
        jse.type &= ~JS_EVENT_INIT;
        if (jse.type == JS_EVENT_AXIS)
        {
            switch (jse.number)
            {
                case 0: this->leftHorizontal  = jse.value; break;
                case 1: this->leftVertical    = jse.value; break;
                case 2: this->rightHorizontal = jse.value; break;
                case 3: this->rightVertical   = jse.value; break;

                default: break;
            }
        }
        if (jse.type == JS_EVENT_BUTTON && jse.value == JS_EVENT_BUTTON_DOWN)
        {
            if(joystick_mode == JS_MODE_1)
			{
				switch(jse.number)
				{				
					// if joystick's red led is not light
					case 6:maxLinear_x += 0.1;
						break;
					case 8:(maxLinear_x > 0.1)?(maxLinear_x -= 0.1):(maxLinear_x = maxLinear_x);
						break;
					case 7:maxAngular_z += 0.1;
						break;
					case 9:(maxAngular_z > 0.1)?(maxAngular_z -= 0.1):(maxAngular_z = maxAngular_z);
						break;
					default: break;
				}
			}
			else if(joystick_mode == JS_MODE_2)	
			{				
				switch(jse.number)
				{
					// if joystick's red led is light
					case 4:maxLinear_x += 0.1;
						break;
					case 6:(maxLinear_x > 0.1)?(maxLinear_x -= 0.1):(maxLinear_x = maxLinear_x);
						break;
					case 5:maxAngular_z += 0.1;
						break;
					case 7:(maxAngular_z > 0.1)?(maxAngular_z -= 0.1):(maxAngular_z = maxAngular_z);
						break;	
					default: break;
				}			
            }
            ROS_INFO("number %d maxLinear_x,maxAngular_z = [%f  %f]",jse.number,maxLinear_x, maxAngular_z);
        }
        memset(&jse, 0, sizeof(jse));

        vel_msg_.linear.y  = maxLinear_y /32767 * leftHorizontal  * (-1);
        vel_msg_.linear.x  = maxLinear_x /32767 * leftVertical    * (-1);
		if(vel_msg_.linear.x<0.09 && vel_msg_.linear.x>-0.09)
			vel_msg_.linear.x = 0;
		if(vel_msg_.linear.x>0)
			vel_msg_.angular.z = maxAngular_z/32767 * rightHorizontal * (-1);
		else
			vel_msg_.angular.z = maxAngular_z/32767 * rightHorizontal * (1);
        cmd_vel_pub.publish(vel_msg_);

        rosSleep.sleep();
        ros::spinOnce();
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_controller");
    ROS_INFO("[ZHAIWEIFENG] joy controller node start! ");

    eprobot_joy  eprobot_joystick;
    eprobot_joystick.publish_joystick_event();
}
