#!/usr/bin/env bash

current_date=$(date +"%d_%m_%y_%Hh%M")
results_dir="cyclone_memory_${current_date}"
mkdir -p "$results_dir"
echo "Storing results in $results_dir"

ros2 run memory_test default_nodes > $results_dir/cyclone_default_nodes.csv
ros2 run memory_test nodes_params_off > $results_dir/cyclone_nodes_params_off.csv
ros2 run memory_test default_subs_params_off > $results_dir/cyclone_default_subs_params_off.csv
ros2 run memory_test default_pubs_params_off > $results_dir/cyclone_default_pubs_params_off.csv
ros2 run memory_test default_clients_params_off > $results_dir/cyclone_default_clients_params_off.csv
ros2 run memory_test default_services_params_off > $results_dir/cyclone_default_services_params_off.csv
ros2 run memory_test nodes_params_off_logging_on > $results_dir/cyclone_nodes_params_off_logging_on.csv
ros2 run memory_test pub_sub_diff_topic > $results_dir/cyclone_pub_sub_diff_topic.csv
ros2 run memory_test pub_sub_same_topic > $results_dir/cyclone_pub_sub_same_topic.csv
ros2 run memory_test pub_sub_diff_msg_type > $results_dir/cyclone_pub_sub_diff_msg_type.csv
ros2 run memory_test cli_serv_diff_topics > $results_dir/cyclone_cli_serv_diff_topics.csv
ros2 run memory_test cli_serv_same_topics > $results_dir/cyclone_cli_serv_same_topics.csv
ros2 run memory_test pub_sub_big_history_size > $results_dir/cyclone_pub_sub_big_history_size.csv
ros2 run memory_test pub_sub_big_message_size > $results_dir/cyclone_pub_sub_big_message_size.csv
