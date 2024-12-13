echo 
echo "Fastrtps"
echo 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir fastdds_memory

./default_nodes > fast_default_nodes.csv
./nodes_params_off > fast_nodes_params_off.csv
./default_subs_params_off > fast_default_subs_params_off.csv
./default_pubs_params_off > fast_default_pubs_params_off.csv
./default_clients_params_off > fast_default_clients_params_off.csv
./default_services_params_off > fast_default_services_params_off.csv
./nodes_params_off_logging_on > fast_nodes_params_off_logging_on.csv
./pub_sub_diff_topic > fast_pub_sub_diff_topic.csv
./pub_sub_same_topic > fast_pub_sub_same_topic.csv
./pub_sub_diff_msg_type > fast_pub_sub_diff_msg_type.csv
./cli_serv_diff_topics > fast_cli_serv_diff_topics.csv
./cli_serv_same_topics > fast_cli_serv_same_topics.csv
./pub_sub_big_history_size > fast_pub_sub_big_history_size.csv
./pub_sub_big_message_size > fast_pub_sub_big_message_size.csv

mv *.csv fastdds_memory
