#include <random>  
#include <chrono>  
#include <thread>  
#include <deque>
#include <queue>
#include <rclcpp/rclcpp.hpp>  
#include <geometry_msgs/msg/pose.hpp>  
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
// #include "geometry_msgs/msg/pose.hpp"
  
using namespace std::chrono_literals;  

double median_filter(std::deque<double>& data_queue){
    std::vector<double> temp_vec(data_queue.begin(), data_queue.end());
    std::sort(temp_vec.begin(), temp_vec.end());
    size_t mid = temp_vec.size() / 2;
    return temp_vec[mid];
}
double moving_average_filter(const std::deque<double>& data_deque){
    double sum = std::accumulate(data_deque.begin(), data_deque.end(), 0.0);
    return sum / data_deque.size();
}
  
class RandomTargetPublisher : public rclcpp::Node  
{  
public:  
    RandomTargetPublisher()  
    : Node("random_target_publisher") 
    {  
        command_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("slid_test", 10);
        command_subscribe_ = this->create_subscription<geometry_msgs::msg::Pose>("person_goal_pose", 10, std::bind(&RandomTargetPublisher::command_callback, this, std::placeholders::_1));
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    }  
  
private:  
    void command_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::Pose slide_pub;
        // queue_.push_back(msg);
        // if(queue_.size() > 5){
        //     queue_.pop_front();
        // }
        // std::vector<geometry_msgs::msg::Pose::SharedPtr> temp_vec(queue_.begin(), queue_.end());
        // auto mid = temp_vec.begin() + temp_vec.size() / 2;
        // auto compare = [](const geometry_msgs::msg::Pose::SharedPtr& a, const geometry_msgs::msg::Pose::SharedPtr& b){
        //     return a->position.x < b->position.x;
        // };
        // std::nth_element(temp_vec.begin(), mid, temp_vec.end(), compare);
        // const  geometry_msgs::msg::Pose::SharedPtr& median_msg = *mid;
        queue_x.push_back(msg->position.x);
        queue_y.push_back(msg->position.y);
        if(queue_x.size() > 5){
            queue_x.pop_front();
        }
        if(queue_y.size() > 5){
            queue_y.pop_front();
        }
        double median_x = median_filter(queue_x);
        double median_y = median_filter(queue_y);
        queue_x_update.push_back(median_x);
        queue_y_update.push_back(median_y);
        if(queue_x_update.size() > 6){
            queue_x_update.pop_front();
        }
        if(queue_y_update.size() > 6){
            queue_y_update.pop_front();
        }
        double moving_average_x = moving_average_filter(queue_x_update);
        double moving_average_y = moving_average_filter(queue_y_update);

        slide_pub.position.x = moving_average_x;
        slide_pub.position.y = moving_average_y;

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x =  moving_average_x;
        goal_msg.pose.pose.position.y = moving_average_y;
        goal_msg.pose.pose.position.z = msg->position.z;
        goal_msg.pose.pose.orientation = msg->orientation;
        command_publisher_->publish(slide_pub);
        // RCLCPP_INFO(rclcpp::get_logger("pub"), "asysnc goal: x: %f, y: %f", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);
        action_client_->async_send_goal(goal_msg);

    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr command_subscribe_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    // std::deque<geometry_msgs::msg::Pose::SharedPtr> queue_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr command_publisher_;
    std::deque<double> queue_x;
    std::deque<double> queue_x_update;
    std::deque<double> queue_y;
    std::deque<double> queue_y_update;
};
  
int main(int argc, char * argv[])  
{  
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<RandomTargetPublisher>();  
    rclcpp::spin(node);  
    rclcpp::shutdown();  
    return 0;  
}