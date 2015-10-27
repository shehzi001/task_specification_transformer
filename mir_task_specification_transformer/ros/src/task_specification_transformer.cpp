#include <mir_task_specification_transformer/task_specification_transformer.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>


TaskSpecificationTransformer::TaskSpecificationTransformer(const ros::NodeHandle &nh):
    nh_(nh), is_order_recieved_(false), is_inventory_recieved_(false), event_in_("")
{
    // ROS Subscribers
    event_in_sub_ = nh_.subscribe<std_msgs::String>("event_in", 1, &TaskSpecificationTransformer::cbEventIn, this);
    inventory_sub_ = nh_.subscribe<at_work_robot_example_ros::Inventory>("/robot_example_ros/inventory", 1, 
                                                            &TaskSpecificationTransformer::cbInventory, this);
    order_sub_ = nh_.subscribe<at_work_robot_example_ros::OrderInfo>("/robot_example_ros/order_info", 1, 
                                                            &TaskSpecificationTransformer::cbOrder, this);

    // ROS Publishers
    task_spec_pub_ = nh_.advertise<std_msgs::String>("task_specification", 1);
    event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 1);
}

TaskSpecificationTransformer::~TaskSpecificationTransformer()
{
}

void TaskSpecificationTransformer::initState()
{
    ROS_DEBUG("State: INIT");
    if (is_order_recieved_ && is_inventory_recieved_)
        state_ = IDLE;
    else 
        state_ = INIT;
}

void TaskSpecificationTransformer::idleState()
{
    ROS_DEBUG("State: IDLE");

    if (event_in_ == "e_start") {
        state_ = RUNNING;
    } else {
        state_ = IDLE;
    }
}

void TaskSpecificationTransformer::runningState()
{
    ROS_DEBUG("State: RUNNING");

    std::string task_spec_data = parseIntoRoboCupTask(order_info_msg_ , inventory_msg_);

    std_msgs::String event_out_msg;
    event_out_msg.data = "e_failed";

    if(!task_spec_data.empty()) {
        std_msgs::String task_spec_msg;
        task_spec_msg.data = task_spec_data;
        task_spec_pub_.publish(task_spec_msg);
        event_out_msg.data = "e_success";
    }

    event_out_pub_.publish(event_out_msg);

    state_ = INIT;
    is_order_recieved_ = false;
    is_inventory_recieved_ = false;
    event_in_ = "";
}

string TaskSpecificationTransformer::parseIntoRoboCupTask(at_work_robot_example_ros::OrderInfo order_info, 
    at_work_robot_example_ros::Inventory inventory)
{
    std::map<std::string, std::vector<std::string> > dest_obj;
    std::map<std::string, std::vector<std::string> > source_obj;
    for (int i = 0; i < order_info.orders.size(); i++)
    {
        at_work_robot_example_ros::Order order = order_info.orders[i];
        if (order.quantity_requested.data != 0) {
            for (int cnt = 0; cnt < order.quantity_requested.data; cnt++)
            {
                std::string source_loc = getSourceLocation(inventory, order.object.description.data);
                std::string dest_loc = getDestinationLocation(order_info, order.object.description.data);

                if (!source_loc.empty() && !dest_loc.empty()) {
                    remap(source_loc);
                    source_obj[source_loc];
                    source_obj[source_loc].push_back(order.object.description.data);

                    remap(dest_loc);
                    dest_obj[dest_loc];
                    dest_obj[dest_loc].push_back(order.object.description.data);
                } else {
                    ROS_WARN("Inventory or Order is melformed.");
                    return "";
                }
            }
        } else {
                std::string source_loc = getSourceLocation(inventory, order.object.description.data);
                std::string dest_loc = order.destination.description.data;

                if (!source_loc.empty() && !dest_loc.empty()) {
                    remap(source_loc);
                    source_obj[source_loc];
                    source_obj[source_loc].push_back(order.object.description.data);

                    remap(dest_loc);
                    dest_obj[dest_loc];
                    dest_obj[dest_loc].push_back(order.object.description.data);
                } else {
                    ROS_WARN("Inventory or Order is melformed.");
                    return "";
                }
        }

    }
    
    std::string task_spec = "BTT<initialsituation(";

    std::map<std::string, std::vector<std::string> >::iterator map_iter;
    for (map_iter = source_obj.begin(); map_iter != source_obj.end(); ++map_iter)
    {
        // add source location to task spec
        task_spec += "<";
        task_spec += map_iter->first + ",(";
        std::vector<std::string> v = map_iter->second;
        // add objects at source location
        for (int i = 0; i < v.size(); i++)
        {
            if (i != 0)
            {
                task_spec += ",";
            }
            task_spec += v.at(i);
        }
        task_spec += ")>";
    }
    task_spec += ");goalsituation(";

    for (map_iter = dest_obj.begin(); map_iter != dest_obj.end(); ++map_iter)
    {
        // add destination location to task spec
        task_spec += "<";
        task_spec += map_iter->first + ",line(";
        std::vector<std::string> v = map_iter->second;
        // add objects at destination location
        for (int i = 0; i < v.size(); i++)
        {
            if (i != 0)
            {
                task_spec += ",";
            }
            task_spec += v.at(i);
        }
        task_spec += ")>";
    }
    task_spec += ")>";
    
    std::cout << "TASK SPEC: " << task_spec << std::endl;
    
    return task_spec;
}

void TaskSpecificationTransformer::remap(std::string &location)
{
    boost::replace_all(location, "SHELF-", "S");
    boost::replace_all(location, "WORKSTATION-", "W");
}

std::string TaskSpecificationTransformer::getSourceLocation(at_work_robot_example_ros::Inventory &inventory, std::string object_name)
{
    for (int i = 0; i < inventory.items.size(); i++)
    {
        at_work_robot_example_ros::Item item = inventory.items[i];
        if (item.object.description.data == object_name)
        {
            if (!item.location.description.data.empty())
            {
                std::string loc = item.location.description.data;
                inventory.items.erase(inventory.items.begin()+i);
                return loc;
            }
            else if (!item.container.description.data.empty())
            {
                return getSourceLocation(inventory, item.container.description.data);                
            }
            else
            {
                return "";
            }
        }
    }

    return "";
}

std::string TaskSpecificationTransformer::getDestinationLocation(at_work_robot_example_ros::OrderInfo order_info, std::string object_name)
{
    for (int i = 0; i < order_info.orders.size(); i++)
    {
        at_work_robot_example_ros::Order order = order_info.orders[i];
        if (order.object.description.data == object_name)
        {
            if (!order.destination.description.data.empty())
            {
                return order.destination.description.data;
            }
            else if (!order.container.description.data.empty())
            {
                return getDestinationLocation(order_info, order.container.description.data);                
            }
            else
            {
                return "";
            }
        }
    }

    return "";
}


void TaskSpecificationTransformer::executeCycle()
{
    switch (state_) {
        case INIT:
            initState();
            break;
        case IDLE:
            idleState();
            break;
        case RUNNING:
            runningState();
            break;
        default:
            initState();
    }
}

void TaskSpecificationTransformer::cbEventIn(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG("Event message recieved.");
    event_in_ = msg->data;
}
void TaskSpecificationTransformer::cbInventory(at_work_robot_example_ros::Inventory msg)
{
    ROS_DEBUG("Inventory message recieved.");
    inventory_msg_ = msg;
    is_inventory_recieved_ = true;

}
void TaskSpecificationTransformer::cbOrder(at_work_robot_example_ros::OrderInfo msg)
{
    ROS_DEBUG("Order message recieved.");
    order_info_msg_ = msg;
    is_order_recieved_ = true;
}
