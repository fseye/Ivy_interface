#include <QApplication>
#include <QWidget>
#include <QFile>
#include <QUiLoader>
#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QObject>
#include <QTimer>
#include <QString>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp> 


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"



#include <algorithm>
using std::placeholders::_1;

class interface_node: public QObject, public rclcpp::Node //herite de Qobject et de ROS2
{
    Q_OBJECT //pour la connexion signaux slots
    public:

        interface_node(): QObject(),Node("interface_node")
        {
            //parametre initiaux

            timer_ = new QTimer(this); //timer pour récupérer les valeurs des spinboxes
            connect(timer_, &QTimer::timeout, this, &interface_node::publishSpinboxValues);
            timer_->start(100); // 100 ms

            //souscription
            mode_remb_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/rembobineur/mode", 10, std::bind(&interface_node::remb_mode_callback, this, _1));
            cmd_remb_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/rembobineur/cmd", 10, std::bind(&interface_node::remb_cmd_callback, this, _1));
            speed_remb_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/rembobineur/speed", 10, std::bind(&interface_node::remb_speed_callback, this, _1));

            Joystick_sub_= this->create_subscription<std_msgs::msg::Float32MultiArray>("/stepper/speed", 10, std::bind(&interface_node::stepper_callback, this, _1));

            gachette_sub_=this->create_subscription<std_msgs::msg::Float32MultiArray>("/nourrice/speed", 10, std::bind(&interface_node::nour_gachette_callback, this, _1));

            vannes_sub_= this->create_subscription<std_msgs::msg::Int32MultiArray>("/vannes/cmd", 10, std::bind(&interface_node::vanne_callback, this, _1));

            //publisher

            stepper_modeIHM_pub_= this->create_publisher <std_msgs::msg::Int32MultiArray> ("/stepper/mode_IHM", 10); 

            remb_speedIHM_pub_= this->create_publisher <std_msgs::msg::Int32MultiArray> ("/rembobineur/speed_IHM", 10); 
            
            vanne_cmdIHM_pub_= this-> create_publisher <std_msgs::msg::Int32MultiArray> ("/vannes/cmd_IHM", 10); 

            ihm_button_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/ihm/bidirectional_cmd", 10);

            // ajout pour boutons bidirectionel
            sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/nourrice/speed", 10,
                [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    if (msg->data.size() < 1) return;
                    bool state = static_cast<bool>(msg->data[0]);
                    if (state != current_state_) {
                        current_state_ = state;
                        QMetaObject::invokeMethod(this, [this]() { updateUI(current_state_); }, Qt::QueuedConnection);
                    }
                });

            pub_ = this->create_publisher<std_msgs::msg::Bool>("/nourrice/commande_mode", 10);
            
            //


            RCLCPP_INFO (this->get_logger(), "Interface lancé!");

        }

        void setLabel(const QString& name, QLabel* label) {
            labels_[name] = label;
        }

        void setPushButton(const QString& name, QPushButton* button){
            buttons_[name] = button ;
        }

        void setSpinBox(const QString& name, QSpinBox* spinbox){
            spinboxes_[name]= spinbox;
        }

        int getSpinBoxValue(const QString& name) {
            return (spinboxes_.count(name) && spinboxes_[name]) ? spinboxes_[name]->value() : -1;
        }

        void setStepperColor(QLabel* label , float value_stepper){

            if (!label) return; 

                float clamped = std::clamp(value_stepper, -255.0f, 255.0f);
                int intensity = static_cast<int>(std::abs(clamped));
                intensity = std::max(30, intensity); // minimum d’intensité
                int r = 0, g = 0, b = 0;

                if (clamped < 0) {
                    // Recul donc rouge
                    r = intensity;
                    g = 0;
                } else if(clamped > 0) {
                    // Avance donc vert
                    r = 0;
                    g = intensity;
                }else {
                    // couleur grise si vitesse = 0
                    r = g = b = 200; // gris clair
                }
                //définition du style
                
               QString style = QString(
                    "QLabel {"
                    "background-color: rgb(%1, %2, %3);"
                    "border-radius: 20px;"
                    "min-width: 40px; min-height: 40px;"
                    "max-width: 40px; max-height: 40px;"
                    "color: white; font-weight: bold;"
                    "}").arg(r).arg(g).arg(b);

                QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection,
                                        Q_ARG(QString, style));            

        }

        void setButtonColor(QPushButton* button, const QString& color, bool bold = false) {
            if (!button) return;

            QString style = QString(
                "QPushButton {"
                "background-color: %1;"
                "%2"
                "}").arg(color, bold ? "font-weight: bold;" : "");

            QMetaObject::invokeMethod(button, "setStyleSheet", Qt::QueuedConnection,
                                    Q_ARG(QString, style));
        }


        void updateButtonPairState(const QString& key_avance,
                           const QString& key_recul,
                           int direction) {
            // Définir les couleurs selon la direction
            QString avance_color = "lightgray";
            QString recul_color = "lightgray";

            if (direction == 1) {
                avance_color = "lightgreen";
            } else if (direction == -1) {
                recul_color = "lightgreen";
            }

            // Appliquer la couleur à chaque bouton si présent
            const std::map<QString, QString> color_map = {
                { key_avance, avance_color },
                { key_recul, recul_color }
            };
            //parcourir la map pour mettre les couleurs à chaque bouton
            for (const auto& [key, color] : color_map) {
                auto it = buttons_.find(key);
                if (it != buttons_.end()) {
                    setButtonColor(it->second, color, true);
                }
            }
        }

        void setLabelText(QLabel* label, const QString& text) {
            if (!label) return;
            QMetaObject::invokeMethod(label, [label, text]() {
                label->setText(text);
            }, Qt::QueuedConnection);
        }


    private:

        //attributs

        std::unordered_map<QString, QLabel*> labels_;
        std::map<QString, QPushButton*> buttons_;
        std::map<QString, QSpinBox*> spinboxes_; 
        
        //gestion synchro bouton
        //enrouleur
        bool current_state_ =false;
        bool user_input_blocked_=false;


        // ajout pour timer 
        QTimer* timer_ ;

        //subscription
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>:: SharedPtr mode_remb_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_remb_sub_; // avance ou recul
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_remb_sub_; //speed
        
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;


        //steppers
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr Joystick_sub_;
        
        //nourisse
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gachette_sub_;

        //vanne
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr vannes_sub_;

        //publisher des commandes de l'IHM

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr stepper_modeIHM_pub_; // 3 modes de controles
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr remb_speedIHM_pub_;// deux consignes de vitesse Avance et recul
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vanne_cmdIHM_pub_; //deux consignes de pression
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ihm_button_pub_;


        //callbaks
        void remb_mode_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){ // ON/OFF
            
        }

        void remb_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            int direction = 0;
            if (msg->data.size() > 0) {
                direction = static_cast<int>(msg->data[0]); // -1, 0, ou 1
            }

            updateButtonPairState("avance_remb", "recul_remb", direction);
        }


        void remb_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){ //vitesse

        }

        void stepper_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            for(int i=1; i<5;i++){
                float value = msg->data[i-1]; // entre 0 et 255
                QString key = QString("stepper%1").arg(i);  // clé = "stepper0", "stepper1", etc.
                if (labels_.count(key)) {
                    setStepperColor(labels_[key], value);
                }
            }

        }

        void nour_gachette_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            int speed_nourrice=0;
            if (msg->data.size() < 3) {
                RCLCPP_WARN(this->get_logger(), "Message reçu invalide : taille < 3");
                return;
            }
            //affichage du mode de la nourrice
            int mode_nourrice = (msg->data[0] > 0) ? 1 : -1;
            updateButtonPairState("enrouleur_manuel", "enrouleur_auto", mode_nourrice);
            //affichage de la direction en fonction de la vitesse de gachette reçu

            int direction = 0;
            if (msg->data[1] > 0 && msg->data[2] == 0) {
                direction = -1;
                speed_nourrice= static_cast<int>(msg->data[1]);
            } else if (msg->data[2] > 0 && msg->data[1] == 0) {
                direction = 1;
                speed_nourrice= msg->data[2];
            }
            updateButtonPairState("enrouleur_avance", "enrouleur_recul", direction);
            //affichage de la vitesse
            if (labels_.count("speed_nourrice")){
                setLabelText(labels_["speed_nourrice"],QString::number(speed_nourrice));
            }

        }

        void vanne_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 4) {
                RCLCPP_WARN(this->get_logger(), "Message vannes invalide (data.size()<4) : %zu", msg->data.size());
                return;
            }

            for (int i = 0; i < 4; i++) {
                int value = msg->data[i];

                auto it = buttons_.find(QString::number(i));  // selon ta clé dans buttons_
                if (it == buttons_.end()) continue;

                QPushButton* button = it->second;
                if (!button) continue;

                if (value == 255) {
                    QColor color;

                    switch (i) {
                        case 0: color = QColor(0, 255, 0);     break; // vert vif
                        case 1: color = QColor(255, 0, 0);     break; // rouge vif
                        case 2: color = QColor(255, 255, 0);   break; // jaune vif
                        case 3: color = QColor(0, 0, 255);     break; // bleu vif
                        default: color = QColor(200, 200, 200);break; // gris par défaut
                    }

                    QString style = QString(
                        "QPushButton {"
                        "background-color: rgb(%1, %2, %3);"
                        "border-radius: 30px;"
                        "min-width: 60px; min-height: 60px;"
                        "max-width: 60px; max-height: 60px;"
                        "color: white;"
                        "font-weight: bold;"
                        "}")
                        .arg(color.red())
                        .arg(color.green())
                        .arg(color.blue());

                    QMetaObject::invokeMethod(button, "setStyleSheet", Qt::QueuedConnection,
                                            Q_ARG(QString, style));
                }
                else {
                    // Remet à une couleur neutre si valeur différente de 1
                    QString style = QString(
                        "QPushButton {"
                        "background-color: none;"
                        "}");
                    QMetaObject::invokeMethod(button, "setStyleSheet", Qt::QueuedConnection,
                                            Q_ARG(QString, style));
                }
            }
        }
        void updateUI(bool state) {
            user_input_blocked_ = true;
            auto manu_enr = buttons_.at("enrouleur_manuel");
            auto auto_enr = buttons_.at("enrouleur_auto");
            if(manu_enr && auto_enr) {
                manu_enr->setChecked(state);
                auto_enr->setChecked(!state);
            }
            user_input_blocked_ = false;
        }



    public slots: //connexion pour les spinBOx en signal slots

        void publishSpinboxValues() {
            std_msgs::msg::Int32MultiArray msg_spin_remb;
            std_msgs::msg::Int32MultiArray msg_spin_pression;

            // Rembobineur
            for (int i = 1; i <= 2; ++i) {
                QString key_remb = QString("sb_cmd_remb_V%1").arg(i);
                if (!spinboxes_.count(key_remb)) continue;

                QSpinBox* sb = spinboxes_[key_remb];
                if (!sb) continue;

                msg_spin_remb.data.push_back(sb->value());
            }

            // Pression
            for (int i = 1; i <= 2; ++i) {
                QString key_pression = QString("sB_cmd_P%1").arg(i);  // Vérifie bien ce nom
                if (!spinboxes_.count(key_pression)) continue;

                QSpinBox* sb = spinboxes_[key_pression];
                if (!sb) continue;

                msg_spin_pression.data.push_back(sb->value());
            }

            // Publication (si les messages ont au moins une valeur)
            if (!msg_spin_remb.data.empty()) {
                remb_speedIHM_pub_->publish(msg_spin_remb);
            }

            if (!msg_spin_pression.data.empty()) {
                vanne_cmdIHM_pub_->publish(msg_spin_pression);
            }
        }

            //bidirectionel
        void handleManuClicked() {
            if (user_input_blocked_) return;
            std_msgs::msg::Bool msg;
            msg.data = true;
            pub_->publish(msg);
        }
        void handleAutoClicked() {
            if (user_input_blocked_) return;
            std_msgs::msg::Bool msg;
            msg.data = false;
            pub_->publish(msg);
        }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("interface_pkg");
    QString ui_path = QString::fromStdString(pkg_share_dir + "/resources/mainwindow.ui");

    QFile uiFile(ui_path);
    if (!uiFile.open(QFile::ReadOnly)) {
        qFatal("Impossible de charger le fichier .ui");
        return -1;
    }


    //QFile uiFile("/home/fseye/mainwindow.ui");
    uiFile.open(QFile::ReadOnly);

    QUiLoader loader;
    QMainWindow* mainWindow = qobject_cast<QMainWindow*>(loader.load(&uiFile));

    uiFile.close();
/*
    if (!mainWindow) {
        qFatal("Impossible de charger le fichier .ui");
        return -1;
    }*/


    // charger les objets qt

    // Crée une instance de ta classe interface_node
    auto node = std::make_shared<interface_node>();
    
    // Labels
    //steppers

    for (int i = 1; i <= 5  ; ++i) {
        QString qt_name = QString("lb_ind_stp%1").arg(i); // structure qui remplace le %1 par i, pour les 4 steppers
        QLabel* label_stepper = mainWindow->findChild<QLabel*>(qt_name);
        if (label_stepper) {
            node->setLabel(QString("stepper%1").arg(i), label_stepper); //stepper1 comme nom
        }
    }

    //nourrice
    QLabel* lb_speed_nourrice = mainWindow->findChild<QLabel*>("lb_vitesse_omb");
    if (lb_speed_nourrice){
        node->setLabel("speed_nourrice", lb_speed_nourrice);
    }

    //Pression
    for (int i = 1; i <= 3  ; ++i) {
        QString qt_name = QString("lb_P%1").arg(i); // structure qui remplace le %1 par i, pour les 4 steppers
        QLabel* label_pression = mainWindow->findChild<QLabel*>(qt_name);
        if (label_pression) {
            node->setLabel(QString("lb_pression%1").arg(i), label_pression); //pression1 comme nom
        }
    }

    // coiffe
    QLabel* lb_contact = mainWindow->findChild<QLabel*>("lb_contact");
    if (lb_contact){
        node->setLabel("contact", lb_contact);
    }

    QLabel* lb_dist_omb = mainWindow->findChild<QLabel*>("lb_dist_omb");
    if (lb_dist_omb){
        node->setLabel("dist_omb", lb_dist_omb);
    }

    //boutons
    // Pneumatique

    QPushButton* qt_pbCoussin = mainWindow->findChild<QPushButton*>("pb_coussin");
    if (qt_pbCoussin){
        node->setPushButton("coussin",qt_pbCoussin );
    }

    QPushButton* qt_pbgonflage = mainWindow->findChild<QPushButton*>("pb_gonflage");
    if (qt_pbgonflage){
        node->setPushButton("gonflage",qt_pbgonflage);
    }

    QPushButton* qt_pbSurpression = mainWindow->findChild<QPushButton*>("pb_surpression");
    if (qt_pbSurpression)
    {        
        node->setPushButton("surpression", qt_pbSurpression);
    }

    QPushButton* qt_pbMoteur = mainWindow->findChild<QPushButton*>("pb_moteur");
    if (qt_pbMoteur){
        node->setPushButton("moteur", qt_pbMoteur);
    }

    //rembobineur

    QPushButton* av_remb = mainWindow->findChild<QPushButton*>("btn_remb_avance");
    if (av_remb){
        node->setPushButton("avance_remb",av_remb);       
    }

    QPushButton* rec_remb = mainWindow->findChild<QPushButton*>("btn_remb_recul");
    if (rec_remb){
        node->setPushButton("recul_remb",rec_remb);
        
    }
    //enrouleur

    QPushButton* manu_enr = mainWindow->findChild<QPushButton*>("btn_enrouleur_manu");
    if (manu_enr){
        node->setPushButton("enrouleur_manuel",manu_enr);        
    }

    QPushButton* auto_enr = mainWindow->findChild<QPushButton*>("btn_enrouleur_auto");
    if (auto_enr){
        node->setPushButton("enrouleur_auto",auto_enr);  
    }

    //stepper

    QPushButton* btn_NoControl = mainWindow->findChild<QPushButton*>("btn_NoControl");
    if (btn_NoControl){
        node->setPushButton("stepper_no_control",btn_NoControl);
    }

    QPushButton* btn_reinit = mainWindow->findChild<QPushButton*>("btn_reinit");
    if (btn_reinit){
        node->setPushButton("stepper_reinit",btn_reinit);
    }

    QPushButton* btn_controle = mainWindow->findChild<QPushButton*>("btn_controle");
    if (btn_controle){
        node->setPushButton("stepper_control",btn_controle);
    }




    //assignation des boutons dans la MAP, à revoir pour garder soit les int ou les strings
    
    node->setPushButton("0", qt_pbCoussin);
    node->setPushButton("1", qt_pbgonflage);
    node->setPushButton("2", qt_pbSurpression);
    node->setPushButton("3", qt_pbMoteur);


    //boutons enrouleur
    
    node-> setPushButton("4", manu_enr);
    node-> setPushButton("5", auto_enr);

    QObject::connect(manu_enr, &QPushButton::clicked, node.get(), &interface_node::handleManuClicked);
    QObject::connect(auto_enr, &QPushButton::clicked, node.get(), &interface_node::handleAutoClicked);


    //steppers buttons
    node-> setPushButton("6", btn_NoControl);
    node-> setPushButton("7", btn_reinit);
    node-> setPushButton("8", btn_controle);
    //rembobineur
    node->setPushButton("9", av_remb);
    node->setPushButton("10", rec_remb);

    //spinBoxes
    // spinbox pour rembobineur

    for (int i = 1; i <= 2; ++i) {
        QString qt_name = QString("sb_cmd_remb_V%1").arg(i);
        QSpinBox* spinbox = mainWindow->findChild<QSpinBox*>(qt_name);
        if (spinbox) {
            node->setSpinBox(qt_name, spinbox);  // Clé = même nom que Qt Designer
        }
    }

    // spinbox pour pression
    for (int i = 1; i <= 2; ++i) {
        QString qt_name = QString("sB_cmd_P%1").arg(i);
        QSpinBox* spinbox = mainWindow->findChild<QSpinBox*>(qt_name);
        if (spinbox) {
            node->setSpinBox(qt_name, spinbox);  // Même nom
        }
    }



    // Lance un thread séparé pour ROS 2
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });
    
    mainWindow->show(); //QmainWindow

    // Exécute l'application Qt
    int result = app.exec();

    // Arrête proprement ROS 2
    rclcpp::shutdown();
    ros_thread.join();

    return result;

}


#include "interface_node.moc"
