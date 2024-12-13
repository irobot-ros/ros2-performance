echo 
echo "StubDDS"
echo 
export RMW_IMPLEMENTATION=rmw_stub_cpp

mkdir stubdds_memory

./default_nodes > stub_default_nodes.csv
./nodes_params_off > stub_nodes_params_off.csv
./default_subs_params_off > stub_default_subs_params_off.csv
./default_pubs_params_off > stub_default_pubs_params_off.csv
./default_clients_params_off > stub_default_clients_params_off.csv
./default_services_params_off > stub_default_services_params_off.csv
./nodes_params_off_logging_on > stub_nodes_params_off_logging_on.csv
./pub_sub_diff_topic > stub_pub_sub_diff_topic.csv
./pub_sub_same_topic > stub_pub_sub_same_topic.csv
./pub_sub_diff_msg_type > stub_pub_sub_diff_msg_type.csv
./cli_serv_diff_topics > stub_cli_serv_diff_topics.csv
./cli_serv_same_topics > stub_cli_serv_same_topics.csv
./pub_sub_big_history_size > stub_pub_sub_big_history_size.csv
./pub_sub_big_message_size > stub_pub_sub_big_message_size.csv

mv *.csv stubdds_memory
