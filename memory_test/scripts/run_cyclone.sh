echo 
echo "CycloneDDS"
echo 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir cyclone_memory

ros2 run memory_test default_nodes > cyclone_default_nodes.csv
ros2 run memory_test nodes_params_off > cyclone_nodes_params_off.csv
ros2 run memory_test default_subs_params_off > cyclone_default_subs_params_off.csv
ros2 run memory_test default_pubs_params_off > cyclone_default_pubs_params_off.csv
ros2 run memory_test default_clients_params_off > cyclone_default_clients_params_off.csv
ros2 run memory_test default_services_params_off > cyclone_default_services_params_off.csv
ros2 run memory_test nodes_params_off_logging_on > cyclone_nodes_params_off_logging_on.csv
ros2 run memory_test pub_sub_diff_topic > cyclone_pub_sub_diff_topic.csv
ros2 run memory_test pub_sub_same_topic > cyclone_pub_sub_same_topic.csv
ros2 run memory_test pub_sub_diff_msg_type > cyclone_pub_sub_diff_msg_type.csv
ros2 run memory_test cli_serv_diff_topics > cyclone_cli_serv_diff_topics.csv
ros2 run memory_test cli_serv_same_topics > cyclone_cli_serv_same_topics.csv
ros2 run memory_test pub_sub_big_history_size > cyclone_pub_sub_big_history_size.csv
ros2 run memory_test pub_sub_big_message_size > cyclone_pub_sub_big_message_size.csv

mv *.csv cyclone_memory
