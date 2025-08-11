#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>

using std::placeholders::_1;



class StepperControl: public rclcpp::Node
{

    public:

        StepperControl(): Node("stepper_node") //nom du noeud
        {
            //paramètres initiaux
            
            //souscription
            
            joystick_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/stepper/Joystick", 10, std::bind(&StepperControl::joystick_callback, this, _1));

            //publisher
            speed_stepper_pub_= this->create_publisher<std_msgs::msg::Float32MultiArray>("/stepper/speed", 10);

            RCLCPP_INFO (this->get_logger(), "Stepper control initialisé.");
        }

        std::array<int, 6> gestion_joystick(float X, float Y)
        {
            std::array<int, 6> joystick;

            joystick[0] = static_cast<int>(0.5 * X * 255 + 0.5 * Y * 255);
            joystick[1] = static_cast<int>(0.5 * X * 255 - 0.5 * Y * 255);
            joystick[2] = static_cast<int>(-0.5 * X * 255 - 0.5 * Y * 255);
            joystick[3] = static_cast<int>(-0.5 * X * 255 + 0.5 * Y * 255);

            float norme = 100 * std::sqrt(X * X + Y * Y);
            float angle = std::atan2(Y, -X); 
            joystick[4] = static_cast<int>(norme);
            joystick[5] = static_cast<int>(angle * 180.0 / M_PI);  

            return joystick;

        }

        std::array<int, 6> filtrage_joystick (std::array<int, 6> joystick)
        
        {
            for (int j=0; j<4; j++) {
                if (joystick[j]<=100) {
                joystick[j]=0;
                }
            }
            if (joystick[4]<30) {
                for (int j=0; j<4; j++) {
                joystick[j]=0;
                }
            }
            return joystick;
        }


        

    private:


        //attributs
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr joystick_sub_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_stepper_pub_;  

        std::array<int, 4> joystick_L;
        std::array<int, 4> joystick_R;


        void joystick_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                if (msg->data.size() < 6) {
                    RCLCPP_WARN(this->get_logger(), "Message joystick invalide, trop court.");
                    return;
                }
                float X_L = msg->data[0];
                float Y_L = msg->data[1];
                float X_R = msg->data[2];
                float Y_R = msg->data[3];

                float BT_click_joy_L= msg->data[4];
                float BT_click_joy_R= msg->data[5];


                // Calcul des vecteurs joystick et mise à zéro
                std::array<int, 6> joyL = filtrage_joystick(gestion_joystick(X_L, Y_L));
                std::array<int, 6> joyR = filtrage_joystick(gestion_joystick(X_R, Y_R));

                // Mise à jour des directions moteurs
                for (int m = 0; m < 4; m++) {
                    joystick_L[m] = joyL[m];
                    joystick_R[m] = joyR[m];
                }

                // y du message à publier
                std_msgs::msg::Float32MultiArray msg_out;

                if(BT_click_joy_L==0 && BT_click_joy_R==0)
                {
                    if (joyL[4] > 50 && joyR[4] < 50) 
                    {
                        for (int i = 0; i < 4; ++i) { //relacher
                            msg_out.data.push_back( -1 * static_cast<float>(joystick_L[i]));
                        }
                    } else {
                        for (int i = 0; i < 4; ++i) {
                            msg_out.data.push_back( static_cast<float>(joystick_R[i])); //tirer
                        }
                    }
                }

                //////condition de blocage à mettre en place


                else if(BT_click_joy_L==1 && BT_click_joy_R==0){ // condition pour relacher les filins
                    for (int i = 0; i < 4; ++i) { //relacher
                            msg_out.data.push_back(-255);
                    }
                }

                else if(BT_click_joy_L==0 && BT_click_joy_R==1){ // condition pour relacher les filins
                    for (int i = 0; i < 4; ++i) { //relacher
                            msg_out.data.push_back(255);
                    }
                }

                speed_stepper_pub_->publish(msg_out);
            }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepperControl>());
  rclcpp::shutdown();
  return 0;
}
