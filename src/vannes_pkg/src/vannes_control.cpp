#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include <algorithm>

using std::placeholders::_1;

class vannes_control : public rclcpp::Node

{
    private:

    bool previous_coussin_state_= false;
    bool previous_gonflage_state_= false;
    bool previous_surpression_state_=false; 
    bool previous_motor_state_= false;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>:: SharedPtr vannes_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vannes_cmd_pub_; //speed 
    
    /*
    void vannes_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
        if (msg->data.size() < 4)
        {
        RCLCPP_WARN(this->get_logger(), "Message vannes invalide (moins de 3 valeurs)");
        return;
        }
        bool coussin_state_= static_cast<bool>(msg->data[0]);
        bool gonflage_state_= static_cast<bool>(msg->data[1]);
        bool surpression_state_ = static_cast<bool>(msg->data[2]) ;
        bool motor_state_= static_cast<bool>(msg->data[3]);
        
        std_msgs::msg::Int32MultiArray vannes_cmd_pub;

        if (coussin_state_ && !previous_coussin_state_){
            coussin_state_= !coussin_state_;
            if (coussin_state_){
                vannes_cmd_pub.data.push_back(255);
            }else {
                vannes_cmd_pub.data.push_back(0);
            }
        }

        if (gonflage_state_ && !previous_gonflage_state_){
            gonflage_state_= !gonflage_state_;
            if (gonflage_state_){
                vannes_cmd_pub.data.push_back(255);
            }else {
                vannes_cmd_pub.data.push_back(0);
            }
        }

        if (surpression_state_ && !previous_surpression_state_){
            surpression_state_= !surpression_state_;
            if (surpression_state_){
                vannes_cmd_pub.data.push_back(255);
            }else {
                vannes_cmd_pub.data.push_back(0);
            }
        }

        if (motor_state_ && !previous_motor_state_){
            motor_state_= !motor_state_;
            if (motor_state_){
                vannes_cmd_pub.data.push_back(255);
            }else {
                vannes_cmd_pub.data.push_back(0);
            }
        }


        vannes_cmd_pub_->publish(vannes_cmd_pub);
        
        // Mettre à jour les précédents
        previous_coussin_state_ = coussin_state_;
        previous_gonflage_state_ = gonflage_state_;
        previous_surpression_state_ = surpression_state_;
        previous_motor_state_ = motor_state_;



    }
    */

    void vannes_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Message vannes invalide (moins de 4 valeurs)");
            return;
        }

        // Valeurs entrantes (état des boutons appuyés ou non)
        bool current_states[4] = {
            static_cast<bool>(msg->data[0]),
            static_cast<bool>(msg->data[1]),
            static_cast<bool>(msg->data[2]),
            static_cast<bool>(msg->data[3])
        };

        // États précédents
        bool previous_states[4] = {
            previous_coussin_state_,
            previous_gonflage_state_,
            previous_surpression_state_,
            previous_motor_state_
        };

        // États logiques internes 
        static int toggled_outputs[4] = {0, 0, 0, 0};

        std_msgs::msg::Int32MultiArray cmd_msg;

        for (int i = 0; i < 4; ++i) {
            // Si front montant (false -> true), on toggle l’état
            if (current_states[i] && !previous_states[i]) {
                toggled_outputs[i] = (toggled_outputs[i] == 0) ? 255 : 0;
            }

            // Ajoute la valeur actuelle (255 ou 0)
            cmd_msg.data.push_back(toggled_outputs[i]);
        }

        // Mise à jour des états précédents
        previous_coussin_state_    = current_states[0];
        previous_gonflage_state_   = current_states[1];
        previous_surpression_state_= current_states[2];
        previous_motor_state_      = current_states[3];

        // Publication
        vannes_cmd_pub_->publish(cmd_msg);
    }


    /* data */
    public:

        vannes_control(): Node("vannesControl"){
            vannes_sub_= this->create_subscription<std_msgs::msg::Int32MultiArray>("/vannes/mode", 10, std::bind(&vannes_control::vannes_callback, this, _1));

            vannes_cmd_pub_= this->create_publisher <std_msgs::msg::Int32MultiArray> ("/vannes/cmd", 10); 

            RCLCPP_INFO (this->get_logger(), "Vannes control started! ");

    }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vannes_control>());
  rclcpp::shutdown();
  return 0;
}
