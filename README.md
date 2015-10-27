# Usage
### Clone repository
    
    git clone git@github.com:shehzi001/task_specification_transformer.git
    
### Launch the component

    roslaunch mir_task_specification_transformer task_specification_transformer.launch
    
### Component start/stop topic

    rostopic pub /task_specification_transformer_node/event_in std_msgs/String "data: 'e_start'"
    or
    rostopic pub /task_specification_transformer_node/event_in std_msgs/String "data: 'e_stop'"
    
### Component event out topic

    rostopic echo /task_specification_transformer_node/event_out

    outputs: e_success or e_failed

### Component data input topic(needs to remapped to actual data topics)
    
    /task_specification_transformer_node/inventory
    /task_specification_transformer_node/order_info
    
### Component data output topic

    rostopic echo /task_specification_transformer_node/task_specification
