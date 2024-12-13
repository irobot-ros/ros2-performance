echo 
echo "CycloneDDS"
echo 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir cyclone_memory

./default_nodes > cyclone_default_nodes.csv
./nodes_params_off > cyclone_nodes_params_off.csv
./default_subs_params_off > cyclone_default_subs_params_off.csv
./default_pubs_params_off > cyclone_default_pubs_params_off.csv
./default_clients_params_off > cyclone_default_clients_params_off.csv
./default_services_params_off > cyclone_default_services_params_off.csv
./nodes_params_off_logging_on > cyclone_nodes_params_off_logging_on.csv
./pub_sub_diff_topic > cyclone_pub_sub_diff_topic.csv
./pub_sub_same_topic > cyclone_pub_sub_same_topic.csv
./pub_sub_diff_msg_type > cyclone_pub_sub_diff_msg_type.csv
./cli_serv_diff_topics > cyclone_cli_serv_diff_topics.csv
./cli_serv_same_topics > cyclone_cli_serv_same_topics.csv
./pub_sub_big_history_size > cyclone_pub_sub_big_history_size.csv
./pub_sub_big_message_size > cyclone_pub_sub_big_message_size.csv

mv *.csv cyclone_memory
