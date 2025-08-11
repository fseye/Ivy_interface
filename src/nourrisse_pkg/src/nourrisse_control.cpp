#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"


#include <algorithm>

using std::placeholders::_1;

class Nourrice_control: public rclcpp::Node
{
    private:
    bool previous_mode_nourrice_ = false;
    bool mode_nourrice_active_ = false;

    //subscriber
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr gachette_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr commande_mode_sub_; // pour la mise en place du bouton bidirectionnel , recup l'état et l'envoie sur le topic

    //publisher
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gachette_speed_pub_; //on ou off


    void gachette_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Message gachette invalide (moins de 3 valeurs)");
            return;
        }

        float bouton_LB = msg->data[0];
        bool bouton_actuel = static_cast<bool>(bouton_LB);

        // Détection du front montant pour basculer l'état toggle
        if (bouton_actuel && !previous_mode_nourrice_) {
            mode_nourrice_active_ = !mode_nourrice_active_;
            RCLCPP_INFO(this->get_logger(), "Mode nourrice %s", mode_nourrice_active_ ? "activé" : "désactivé");
        }

        previous_mode_nourrice_ = bouton_actuel;

        float gachette_LT = static_cast<float>((1.0f - msg->data[1]) * 127.5f);
        float gachette_RT = static_cast<float>((1.0f - msg->data[2]) * 127.5f);

        float vitesse_LT = 0.0f;
        float vitesse_RT = 0.0f;

        // Traitement des gâchettes uniquement si mode nourrice activé
        if (mode_nourrice_active_) {
            if (gachette_LT > 0 && gachette_RT == 0) {
                vitesse_LT = gachette_LT;
            }
            else if (gachette_RT > 0 && gachette_LT == 0) {
                vitesse_RT = gachette_RT;
            }
            // Si les deux sont enfoncées ou aucune, vitesses restent à 0
        }

        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data.push_back(static_cast<float>(mode_nourrice_active_)); // 1 ou 0
        msg_out.data.push_back(vitesse_LT);
        msg_out.data.push_back(vitesse_RT);

        gachette_speed_pub_->publish(msg_out);
    }


    
    public:

        Nourrice_control(): Node("nourrice_control")
        {
            
            gachette_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/nourrice/gachette", 10, std::bind(&Nourrice_control::gachette_callback, this, _1));

            gachette_speed_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/nourrice/speed", 10);

            //ajout pour bpouton bidirectionel

            commande_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/nourrice/commande_mode", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (mode_nourrice_active_ != msg->data) {
                        mode_nourrice_active_ = msg->data;
                        RCLCPP_INFO(this->get_logger(), "Mode nourrice changé par interface: %s", mode_nourrice_active_ ? "activé" : "désactivé");
                    }
                }
            );


            RCLCPP_INFO (this->get_logger(), "Nourrice control started! ");

        }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nourrice_control>());
  rclcpp::shutdown();
  return 0;
}
