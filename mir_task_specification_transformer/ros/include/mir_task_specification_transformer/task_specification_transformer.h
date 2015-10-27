#ifndef TASK_SPECIFICATION_TRANSFORMER_H_
#define TASK_SPECIFICATION_TRANSFORMER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <csignal>

#include <at_work_robot_example_ros/Inventory.h>
#include <at_work_robot_example_ros/OrderInfo.h>
#include <at_work_robot_example_ros/Order.h>
#include <at_work_robot_example_ros/Item.h>
#include <string>


using std::string;
using std::signal;

class TaskSpecificationTransformer
{
public:
    TaskSpecificationTransformer(const ros::NodeHandle &nh);
    ~TaskSpecificationTransformer();

    void executeCycle();

private:
    enum State {
        INIT,
        IDLE,
        RUNNING
    };

    void cbEventIn(const std_msgs::String::ConstPtr& msg);

    void cbInventory(at_work_robot_example_ros::Inventory msg);

    void cbOrder(at_work_robot_example_ros::OrderInfo msg);

    std::string getSourceLocation(at_work_robot_example_ros::Inventory &inventory, std::string object_name);

    std::string getDestinationLocation(at_work_robot_example_ros::OrderInfo order_info, std::string object_name);

    void remap(string &location);

    string parseIntoRoboCupTask(at_work_robot_example_ros::OrderInfo order_info, 
                                at_work_robot_example_ros::Inventory inventory);

    void initState();
    void idleState();
    void runningState();

    ros::NodeHandle nh_;

    // internal state variable
    State state_;

    at_work_robot_example_ros::OrderInfo order_info_msg_;
    at_work_robot_example_ros::Inventory inventory_msg_;
    std::string event_in_;

    bool is_order_recieved_;
    bool is_inventory_recieved_;


    // ROS Publishers
    ros::Publisher event_out_pub_;
    ros::Publisher task_spec_pub_;

    // ROS Subscribers
    ros::Subscriber event_in_sub_;
    ros::Subscriber inventory_sub_;

    ros::Subscriber order_sub_;

};
#endif /* TASK_SPECIFICATION_TRANSFORMER_H_ */
