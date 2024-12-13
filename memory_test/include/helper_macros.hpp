// Nodes with single entities
#define NODE(NUM) \
  auto node_##NUM = rclcpp::Node::make_shared("node_" #NUM, node_options); \
  executor->add_node(node_##NUM); \
  print_rss("emtpy_node", NUM);

#define NODE_P(NUM, TYPE) \
  auto node_##NUM = rclcpp::Node::make_shared("node_p_"#TYPE"_"#NUM, node_options); \
  auto publisher_##NUM = node_##NUM ->create_publisher<memory_msgs::msg::TYPE>("topic_"#NUM, 1); \
  executor->add_node(node_##NUM); \
  print_rss("node_p_"#TYPE, NUM);

#define NODE_PS_DIFF_TOPIC(NUM, TYPE) \
  auto node_##NUM = rclcpp::Node::make_shared("node_ps_"#TYPE"_"#NUM, node_options); \
  auto publisher_##NUM = node_##NUM ->create_publisher<memory_msgs::msg::TYPE>("topic_"#NUM, 1); \
  auto subscription_##NUM = node_##NUM ->create_subscription<memory_msgs::msg::TYPE>( \
    "topic_"#NUM, 10,[](const memory_msgs::msg::TYPE & msg) {(void)msg;}); \
  executor->add_node(node_##NUM); \
  print_rss("node_ps_dif_T_"#TYPE, NUM);

#define NODE_PS_DIFF_TOPIC_BIG_HISTORY_SIZE(NUM, TYPE) \
  auto node_##NUM = rclcpp::Node::make_shared("node_ps_"#TYPE"_"#NUM, node_options); \
  auto publisher_##NUM = node_##NUM ->create_publisher<memory_msgs::msg::TYPE>("topic_"#NUM, 100); \
  auto subscription_##NUM = node_##NUM ->create_subscription<memory_msgs::msg::TYPE>( \
    "topic_"#NUM, 100,[](const memory_msgs::msg::TYPE & msg) {(void)msg;}); \
  executor->add_node(node_##NUM); \
  print_rss("node_ps_dif_T_"#TYPE, NUM);

#define NODE_PS_SAME_TOPIC(NUM, TYPE) \
  auto node_##NUM = rclcpp::Node::make_shared("node_ps_"#TYPE"_"#NUM, node_options); \
  auto publisher_##NUM = node_##NUM ->create_publisher<memory_msgs::msg::TYPE>("topic", 1); \
  auto subscription_##NUM = node_##NUM ->create_subscription<memory_msgs::msg::TYPE>( \
    "topic", 10,[](const memory_msgs::msg::TYPE & msg) {(void)msg;}); \
  executor->add_node(node_##NUM); \
  print_rss("node_ps_sameT_"#TYPE, NUM);

#define NODE_S(NUM, TYPE) \
  auto node_##NUM = rclcpp::Node::make_shared("node_s_"#TYPE"_"#NUM, node_options); \
  auto subscription_##NUM = node_##NUM ->create_subscription<memory_msgs::msg::TYPE>( \
    "topic_"#NUM, 10,[](const memory_msgs::msg::TYPE & msg) {(void)msg;}); \
  executor->add_node(node_##NUM); \
  print_rss("node_s_"#TYPE, NUM);

#define NODE_SRV(NUM) \
  auto node_##NUM = rclcpp::Node::make_shared("node_srv_"#NUM, node_options); \
  auto service_##NUM = node_##NUM ->create_service<std_srvs::srv::SetBool>("topic_"#NUM, handle_service); \
  executor->add_node(node_##NUM); \
  print_rss("node_srv", NUM);

#define NODE_CLI(NUM) \
  auto node_##NUM = rclcpp::Node::make_shared("node_cli_"#NUM, node_options); \
  auto client_##NUM = node_##NUM ->create_client<std_srvs::srv::SetBool>("topic_"#NUM); \
  executor->add_node(node_##NUM); \
  print_rss("node_cli", NUM);

#define NODE_CLI_SRV(NUM) \
  auto node_##NUM = rclcpp::Node::make_shared("node_srv_"#NUM, node_options); \
  auto client_##NUM = node_##NUM ->create_client<std_srvs::srv::SetBool>("topic_"#NUM); \
  auto service_##NUM = node_##NUM ->create_service<std_srvs::srv::SetBool>("topic_"#NUM, handle_service); \
  executor->add_node(node_##NUM); \
  print_rss("node_srv", NUM);

#define NODE_CLI_SRV_SAME(NUM) \
  auto node_##NUM = rclcpp::Node::make_shared("node_srv_"#NUM, node_options); \
  auto client_##NUM = node_##NUM ->create_client<std_srvs::srv::SetBool>("topic"); \
  auto service_##NUM = node_##NUM ->create_service<std_srvs::srv::SetBool>("topic", handle_service); \
  executor->add_node(node_##NUM); \
  print_rss("node_srv", NUM);

// Single nodes with many entities
#define PUB(NUM,TYPE) \
  auto publisher_##NUM = node_1->create_publisher<memory_msgs::msg::TYPE>("topic_"#NUM, 1); \
  print_rss("pub_"#TYPE, NUM);

#define SUB(NUM,TYPE) \
  auto subscription_##NUM = node_1->create_subscription<memory_msgs::msg::TYPE>( \
    "topic_"#NUM, 10,[](const memory_msgs::msg::TYPE & msg) {(void)msg;}); \
  print_rss("sub_"#TYPE, NUM);

#define CLI(NUM) \
  auto client_##NUM = node_1->create_client<std_srvs::srv::SetBool>("topic_"#NUM); \
  print_rss("client", NUM);

#define SRV(NUM) \
  auto service_##NUM = node_1->create_service<std_srvs::srv::SetBool>("topic_"#NUM, handle_service); \
  print_rss("service", NUM);

#define PUBLISH(NUM, TYPE) \
  auto message_##NUM = memory_msgs::msg::TYPE(); \
  publisher_##NUM->publish(message_##NUM);

#define CREATE_50(X) \
  X(1)  X(2)  X(3)  X(4)  X(5)  X(6)  X(7)  X(8)  X(9)  X(10) \
  X(11) X(12) X(13) X(14) X(15) X(16) X(17) X(18) X(19) X(20) \
  X(21) X(22) X(23) X(24) X(25) X(26) X(27) X(28) X(29) X(30) \
  X(31) X(32) X(33) X(34) X(35) X(36) X(37) X(38) X(39) X(40) \
  X(41) X(42) X(43) X(44) X(45) X(46) X(47) X(48) X(49) X(50) \

#define CREATE_50_TYPED(X,TYPE) \
  X(1,TYPE)  X(2,TYPE)  X(3,TYPE)  X(4,TYPE)  X(5,TYPE)  X(6,TYPE)  X(7,TYPE)  X(8,TYPE)  X(9,TYPE)  X(10,TYPE) \
  X(11,TYPE) X(12,TYPE) X(13,TYPE) X(14,TYPE) X(15,TYPE) X(16,TYPE) X(17,TYPE) X(18,TYPE) X(19,TYPE) X(20,TYPE) \
  X(21,TYPE) X(22,TYPE) X(23,TYPE) X(24,TYPE) X(25,TYPE) X(26,TYPE) X(27,TYPE) X(28,TYPE) X(29,TYPE) X(30,TYPE) \
  X(31,TYPE) X(32,TYPE) X(33,TYPE) X(34,TYPE) X(35,TYPE) X(36,TYPE) X(37,TYPE) X(38,TYPE) X(39,TYPE) X(40,TYPE) \
  X(41,TYPE) X(42,TYPE) X(43,TYPE) X(44,TYPE) X(45,TYPE) X(46,TYPE) X(47,TYPE) X(48,TYPE) X(49,TYPE) X(50,TYPE) \

#define CREATE_50_DIFF_MSG(X) \
  X(1,M1)   X(2,M2)   X(3,M3)   X(4,M4)   X(5,M5)   X(6,M6)   X(7,M7)   X(8,M8)   X(9,M9)   X(10,M10) \
  X(11,M11) X(12,M12) X(13,M13) X(14,M14) X(15,M15) X(16,M16) X(17,M17) X(18,M18) X(19,M19) X(20,M20) \
  X(21,M21) X(22,M22) X(23,M23) X(24,M24) X(25,M25) X(26,M26) X(27,M27) X(28,M28) X(29,M29) X(30,M30) \
  X(31,M31) X(32,M32) X(33,M33) X(34,M34) X(35,M35) X(36,M36) X(37,M37) X(38,M38) X(39,M39) X(40,M40) \
  X(41,M41) X(42,M42) X(43,M43) X(44,M44) X(45,M45) X(46,M46) X(47,M47) X(48,M48) X(49,M49) X(50,M50) \

#define RUN_ALL \
  {CREATE_50(NODE)} \
  {CREATE_50(NODE_CLI)} \
  {CREATE_50(NODE_SRV)} \
  {CREATE_50(NODE_CLI_SRV)} \
  {CREATE_50(NODE_CLI_SRV_SAME)} \
  {CREATE_50_TYPED(NODE_P, M1)} \
  {CREATE_50_TYPED(NODE_S, M1)} \
  {CREATE_50_TYPED(NODE_PS_DIFF_TOPIC, M1)} \
  {CREATE_50_TYPED(NODE_PS_SAME_TOPIC, M1)} \
  {CREATE_50_DIFF_MSG(NODE_PS_DIFF_TOPIC)} \
  {NODE(1) CREATE_50(CLI)} \
  {NODE(1) CREATE_50(SRV)} \
  {NODE(1) CREATE_50_TYPED(SUB,M1)} \
  {NODE(1) CREATE_50_TYPED(PUB,M1)} \
  {NODE(1) CREATE_50_DIFF_MSG(PUB)} \
  {NODE(1) CREATE_50_DIFF_MSG(SUB)}
