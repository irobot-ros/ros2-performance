#!/usr/bin/env bash

if [ -z "$1" ]; then
  results_dir=/tmp/$(date +"%d_%m_%y_%Hh%M")
else
  results_dir=$1
fi

mkdir -p "$results_dir"

echo -n "Running FastDDS memory tests..."

ros2 run memory_test default_nodes > $results_dir/fast_default_nodes.csv
ros2 run memory_test pub_sub_same_topic > $results_dir/fast_pub_sub_same_topic.csv
ros2 run memory_test cli_serv_same_topics > $results_dir/fast_cli_serv_same_topics.csv

echo " complete!"
echo "Results can be found in: $results_dir"

