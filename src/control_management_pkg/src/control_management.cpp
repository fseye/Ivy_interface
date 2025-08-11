#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include <algorithm>

using std::placeholders::_1;



class control_management: public rclcpp::Node
{
    private:

    //attributs à rajouter
        // Membres de la classe
        size_t axis_index_;
        float speed_target_;
        int num_motors_= 4;

        //parametres rembobineur
        bool prev_btn_mode_ = false;
        bool prev_btn_dir_ = false;

        bool mode_remb_active_ = false;
        int cmd_remb_direction_ = 0;  // 1 = avance, -1 = recul, 0 = rien

        
        rclcpp::Subscription<sensor_msgs::msg::Joy>:: SharedPtr joy_sub_;

        //subscriber à l'IHM à rajouter 


        //publisher

        //Rembobineur_publisher
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mode_remb_pub_; //on ou off
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_remb_pub_; // avance ou recul
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_remb_pub_; //speed 

        //stepper_publisher

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr Joystick_pub_;
        //nourisse_publisher

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gachette_pub_;

        //vanne_publisher
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vannes_pub_;


        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            if (axis_index_  >= msg->axes.size()){
                RCLCPP_WARN (this->get_logger(), "index d'axe invalide");
                return;
            }

            
            ////////////////////////////////////////// Rembobineur //////////////////////////////////////////////////


            

            bool mode_remb = static_cast<bool>(msg->buttons[2]); //mode on ou off
            bool cmd_remb= static_cast<bool>(msg->buttons[5]);; //direction
            float motor_remb_speed= 200 * cmd_remb; //à définir dans l'  IHM plus tard

            // gestion bouton On/Off
            if (mode_remb && !prev_btn_mode_){
                mode_remb_active_ = !mode_remb_active_; //Toggle
                if (!mode_remb_active_) {
                    cmd_remb_direction_ = 0;  // arrêt total si OFF
                }
            }

            //gestion sens avance/recul

            if (cmd_remb &&  !prev_btn_dir_ && mode_remb_active_){
                if (cmd_remb_direction_==1){
                    cmd_remb_direction_=-1;
                }else{
                    cmd_remb_direction_=1;
                }
            }
            prev_btn_mode_= mode_remb;
            prev_btn_dir_=cmd_remb;

            std_msgs::msg::Float32MultiArray mode_remb_pub; //le type de message 
            std_msgs::msg::Float32MultiArray cmd_remb_pub; //le type de message 
            std_msgs::msg::Float32MultiArray speed_remb_pub; //le type de message 
            

            for (int i=0; i < num_motors_; i++){
                mode_remb_pub.data.push_back(mode_remb_active_);
                cmd_remb_pub.data.push_back(cmd_remb_direction_);
                speed_remb_pub.data.push_back(std::abs(motor_remb_speed));
            } 

            mode_remb_pub_->publish(mode_remb_pub);
            cmd_remb_pub_->publish(cmd_remb_pub);
            speed_remb_pub_->publish(speed_remb_pub);


            ////////////////////////////////////////// Stepper ////////////////////////////////////////////////////////

            float Joystick_X_L = msg->axes[0]; //recupération de donnée JOY aux axes qui nous interessent
            float Joystick_Y_L = msg->axes[1];
            float Joystick_X_R = msg->axes[3];
            float Joystick_Y_R = msg->axes[4];
            // boutons click joy pour tirer ou relacher tous les steppers
            float BT_Click_Joy_L= msg->buttons[9];
            float BT_Click_Joy_R= msg->buttons[10];



            std_msgs::msg::Float32MultiArray Joystick_pub; // création dn tableau de données
            Joystick_pub.data.push_back(Joystick_X_L); // rajout des valeurs de  joystick
            Joystick_pub.data.push_back(Joystick_Y_L);
            Joystick_pub.data.push_back(Joystick_X_R);
            Joystick_pub.data.push_back(Joystick_Y_R);
            Joystick_pub.data.push_back(BT_Click_Joy_L); // tout relacher
            Joystick_pub.data.push_back(BT_Click_Joy_R); // tout tirer

            Joystick_pub_->publish(Joystick_pub); //publication du tableau



            ///////////////////////////////////////////nourrice /////////////////////////////////////////////////////////

            float gachette_LT= msg->axes[2]; 
            float gachette_RT= msg->axes[5];
            float btn_LB= msg-> buttons[4];

            std_msgs::msg::Float32MultiArray gachette_pub ;
            gachette_pub.data.push_back(btn_LB);
            gachette_pub.data.push_back(gachette_LT);
            gachette_pub.data.push_back(gachette_RT);

            gachette_pub_->publish(gachette_pub);

            //////////////////////////////////////////// Vanne /////////////////////////////////////////////////////////////

            int etat_vanne_coussin= msg->buttons[0]; //bouton A mannette
            int etat_vanne_gonflage= msg->buttons[1]; //bouton B
            int etat_vanne_surpression= msg->buttons[3]; //bouton Y
            int etat_moteur = msg->buttons[2]; //bouton X

            std_msgs::msg::Int32MultiArray vannes_pub;
            vannes_pub.data.push_back(etat_vanne_coussin);
            vannes_pub.data.push_back(etat_vanne_gonflage);
            vannes_pub.data.push_back(etat_vanne_surpression);
            vannes_pub.data.push_back(etat_moteur);


            vannes_pub_->publish(vannes_pub);




        }


    public:

        control_management(): Node("control_management") //nom du noeud 
        {

            // paramètres initiaux à rajouter

            //souscription

            joy_sub_= this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&control_management::joy_callback, this, _1));
            // Souscription IHM à rajouter 


            // publisher
            mode_remb_pub_ = this->create_publisher <std_msgs::msg::Float32MultiArray> ("/rembobineur/mode", 10); //on ou off
            cmd_remb_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/rembobineur/cmd", 10); // avance ou recul
            speed_remb_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/rembobineur/speed", 10); //speed 

            
            Joystick_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/stepper/Joystick", 10); 


            gachette_pub_= this->create_publisher <std_msgs::msg::Float32MultiArray> ("/nourrice/gachette", 10);

            vannes_pub_=this->create_publisher <std_msgs::msg::Int32MultiArray> ("/vannes/mode", 10);
            

            RCLCPP_INFO (this->get_logger(), "Control management started! ");


        }
    //~control_management();

};


// ajout pour test git
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_management>());
  rclcpp::shutdown();
  return 0;
}
