#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"



#include <algorithm>

using std::placeholders::_1;



class RembobineurControl: public rclcpp::Node
{
    public:
        RembobineurControl(): Node("rembobineur_Control") //nom du noeud
        {
            //paramètres initiaux
            /*
            this->declare_parameter("default_speed", 0.5f);
            this->declare_parameter("num_motors", 4);
            this->declare_parameter("joy_axis_index", 1);

            //getter
            
            this->get_parameter("default_speed", speed_target_);
            this->get_parameter("num_motors", num_motors_);
            this->get_parameter("joy_axis_index", axis_index_);
            */
            
            //souscription

            
            mode_remb_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/rembobineur/mode", 10, std::bind(&RembobineurControl::mode_remb_callback, this, _1));
            remb_cmd_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/rembobineur/cmd", 10, std::bind(&RembobineurControl::remb_cmd_callback, this, _1));
            remb_speed_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>( "/rembobineur/speed", 10, std::bind(&RembobineurControl::speed_callback, this, _1));
    
            //publisher

            /*
            cmd_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/rembobineur/cmd", 10);
            speed_pub_ = this->create_publisher <std_msgs::msg::Float32MultiArray> ("rembobineur/speed", 10);
            */

            RCLCPP_INFO (this->get_logger(), "Rembobineur control initialisé.");
        }

    private:


        //attributs


        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr mode_remb_sub_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr remb_cmd_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr remb_speed_sub_;


        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;

        void mode_remb_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            
            
        }

        void remb_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {

        }

        void speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {


        }


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RembobineurControl>());
  rclcpp::shutdown();
  return 0;
}
