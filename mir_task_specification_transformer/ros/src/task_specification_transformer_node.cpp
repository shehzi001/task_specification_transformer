#include <mir_task_specification_transformer/task_specification_transformer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_specification_transformer_node");
    ros::NodeHandle nh("~");

    TaskSpecificationTransformer task_spec_parser = TaskSpecificationTransformer(nh);

    while (ros::ok()) {
        task_spec_parser.executeCycle();
        ros::Rate(10).sleep();
        ros::spinOnce();
    }

    return 0;
}
