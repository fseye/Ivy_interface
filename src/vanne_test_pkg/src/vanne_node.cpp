#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/msg/joy.hpp>
#include <wiringPi.h>

class JoyRelayNode : public rclcpp::Node {
public:
    JoyRelayNode() : Node("joy_relay_node")
    {
        // WiringPi pin 1 = GPIO18
        wiringPiSetup();
        pinMode(1, OUTPUT);
        digitalWrite(1, LOW);

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/vannes/cmd", 10,
            std::bind(&JoyRelayNode::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Node joy_relay_node prêt. Appuie sur un bouton !");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    void joy_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        //if (msg->data.size() > 0 && msg->data[0] ==255) {
            //if (!button_pressed_) {
                //relay_state_ = true;
                //digitalWrite(1, relay_state_ ? HIGH : LOW);
                //RCLCPP_INFO(this->get_logger(), "Relais %s", relay_state_ ? "ON" : "OFF");
                //button_pressed_ = true;
	if (msg->data.size() >0){
		if  (msg->data[0] ==255){
			digitalWrite(1, HIGH);
			RCLCPP_INFO(this->get_logger(), "Relai ON");
		}else{
			digitalWrite(1, LOW); 
			RCLCPP_INFO(this->get_logger(), "Relai OFF");
		}
	// else {
          //  button_pressed_ = false;
        //}
    
	}

    //bool relay_state_;
    //bool button_pressed_ = false;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyRelayNode>());
  rclcpp::shutdown();
  return 0;
}













