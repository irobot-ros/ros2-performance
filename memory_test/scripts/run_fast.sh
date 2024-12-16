echo 
echo "Fastrtps"
echo 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir fastdds_memory

ros2 run memory_test default_nodes > fast_default_nodes.csv
ros2 run memory_test nodes_params_off > fast_nodes_params_off.csv
ros2 run memory_test default_subs_params_off > fast_default_subs_params_off.csv
ros2 run memory_test default_pubs_params_off > fast_default_pubs_params_off.csv
ros2 run memory_test default_clients_params_off > fast_default_clients_params_off.csv
ros2 run memory_test default_services_params_off > fast_default_services_params_off.csv
ros2 run memory_test nodes_params_off_logging_on > fast_nodes_params_off_logging_on.csv
ros2 run memory_test pub_sub_diff_topic > fast_pub_sub_diff_topic.csv
ros2 run memory_test pub_sub_same_topic > fast_pub_sub_same_topic.csv
ros2 run memory_test pub_sub_diff_msg_type > fast_pub_sub_diff_msg_type.csv
ros2 run memory_test cli_serv_diff_topics > fast_cli_serv_diff_topics.csv
ros2 run memory_test cli_serv_same_topics > fast_cli_serv_same_topics.csv
ros2 run memory_test pub_sub_big_history_size > fast_pub_sub_big_history_size.csv
ros2 run memory_test pub_sub_big_message_size > fast_pub_sub_big_message_size.csv

mv *.csv fastdds_memory
