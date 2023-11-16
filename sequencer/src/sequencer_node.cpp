#include "sequencer/sequencer_node.hpp"
#include <queue>

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options)
    
{

    _subscription_seedling_collection = this->create_subscription<std_msgs::msg::Bool>(
        "Seedling_Collection",
        _qos,
        std::bind(&Sequencer::callback_seedling_collection, this, std::placeholders::_1)
    );

    _subscription_seedling_installation = this->create_subscription<std_msgs::msg::Bool>(
        "Seedling_Installation",
        _qos,
        std::bind(&Sequencer::callback_seedling_installation, this, std::placeholders::_1)
    );

    _subscription_ball_collection = this->create_subscription<std_msgs::msg::Bool>(
        "Ball_Collection",
        _qos,
        std::bind(&Sequencer::callback_ball_collection, this, std::placeholders::_1)
    );

    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "convergence",
        _qos,
        std::bind(&Sequencer::callback_convergence, this, std::placeholders::_1)

    );

    _subscription_color_information = this->create_subscription<controller_interface_msg::msg::Colorball>(
        "color_information",
        _qos,
        std::bind(&Sequencer::callback_color_information, this, std::placeholders::_1)

    );

    _publisher_in_process = this->create_publisher<std_msgs::msg::Bool>("in_process", _qos);

    set_in_process(false);

}

void Sequencer::callback_seedling_collection(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::seedling;
    }
}

void Sequencer::callback_seedling_installation(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::planting;
    }
}

void Sequencer::callback_ball_collection(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::harvesting;
    }
}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){

    if(sequence_mode == SEQUENCE_MODE::seedling && seedling_step < 4){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "苗回収シーケンス[%s]_起動", seedling_order[seedling_step].c_str());
            //move_node
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "苗回収シーケンス[%s]_終了", seedling_order[seedling_step].c_str());
            seedling_step++;
            set_in_process(false);
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::planting && planting_step < 8){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "苗設置シーケンス[%s]_起動", planting_order[planting_step].c_str());
            //move_node
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "苗設置シーケンス[%s]_終了", planting_order[planting_step].c_str());
            planting_step++;
            set_in_process(false);
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::harvesting && harvesting_step < 12){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "籾回収シーケンス[%s]_起動", harvesting_order[harvesting_step].c_str());
            //move_node
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "籾回収シーケンス[%s]_終了", harvesting_order[harvesting_step].c_str());
            harvesting_step++;
            set_in_process(false);
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }
}

void Sequencer::callback_color_information(const controller_interface_msg::msg::Colorball::SharedPtr msg){
    if(selsect_algorithm == 0){
        int count = 0;
        bool color = false;
        std::queue<int> own_color;
        std::queue<int> purple_color;
        for(int i = 6; i < 12; i++){
            if(msg->color_info[i]) own_color.push(i);
            else purple_color.push(i);
        }
        for(int i = 0; i < 6; i++){
            color = msg->color_info[i];
            harvesting_order[count] = "H" + std::to_string(i);
            count++;
            if(color) {
                harvesting_order[count] = "H" + std::to_string(purple_color.front());
                purple_color.pop();
            }
            else {
                harvesting_order[count] = "H" + std::to_string(own_color.front());
                own_color.pop();        
            }
            RCLCPP_INFO(this->get_logger(), "%s", harvesting_order[count].c_str());
            count++;
        }
        RCLCPP_INFO(this->get_logger(), "籾の回収順");
        for(int i = 0; i < 12; i++){
            RCLCPP_INFO(this->get_logger(), "%s", harvesting_order[i].c_str());
        }
    }
}

void Sequencer::set_in_process(bool flag){
    in_process = flag;
    auto msg_in_process = std::make_shared<std_msgs::msg::Bool>();
    msg_in_process->data = flag;
    _publisher_in_process->publish(*msg_in_process);
    sequence_process = 0;
}

}  // namespace sequencer