#include <vector>
#include <chrono>
#include <iostream>
#include <string>

#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/communication.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"

#include "performance_test_msgs/msg/stamped_vector.hpp"

#include "benchmark/msg/amazon.hpp"
#include "benchmark/msg/danube.hpp"
#include "benchmark/msg/congo.hpp"
#include "benchmark/msg/parana.hpp"
#include "benchmark/msg/volga.hpp"
#include "benchmark/msg/ganges.hpp"
#include "benchmark/msg/tigris.hpp"
#include "benchmark/msg/salween.hpp"
#include "benchmark/msg/arkansas.hpp"
#include "benchmark/msg/nile.hpp"

#include "memory_stats.hpp"
#include "options.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    auto options = benchmark::Options(argc, argv);

    std::cout << "Intra-process-communication: " << (options.ipc ? "on" : "off") << std::endl;
    std::cout << "Run test for: " << options.duration_sec << " seconds" << std::endl;
    std::cout << "Sampling resources every " << options.resources_sampling_per_ms << "ms" << std::endl;
    std::cout << "Start test" << std::endl;

    system("mkdir -p log");
    std::string resources_output_path = "log/resources.txt";
    std::string events_output_path = "log/events.txt";
    std::string latency_all_output_path = "log/latency_all.txt";
    std::string latency_total_output_path = "log/latency_total.txt";

    // Start resources logger
    performance_test::ResourceUsageLogger ru_logger(resources_output_path);
    ru_logger.start(std::chrono::milliseconds(options.resources_sampling_per_ms));

    rclcpp::init(argc, argv);

    // Architecture
    int executors = 0; // set to 1 if you want to add all nodes to the same executor
    performance_test::System low_end_sys(options.duration_sec, executors);
    low_end_sys.enable_events_logger(events_output_path);

    // Create topics
    auto amazon     = performance_test::Topic<benchmark::msg::Amazon>(                              "amazon"    );
    auto nile       = performance_test::Topic<benchmark::msg::Nile>(                                "nile"      );
    auto ganges     = performance_test::Topic<benchmark::msg::Ganges>(                              "ganges"    );
    auto danube     = performance_test::Topic<benchmark::msg::Danube>(                              "danube"    );
    auto tigris     = performance_test::Topic<benchmark::msg::Tigris>(                              "tigris"    );
    auto parana     = performance_test::Topic<benchmark::msg::Parana>(                              "parana"    );
    auto salween    = performance_test::Topic<benchmark::msg::Salween>(                             "salween"   );
    auto missouri   = performance_test::Topic<performance_test_msgs::msg::StampedVector>(       "missouri"  );
    auto mekong     = performance_test::Topic<performance_test_msgs::msg::StampedVector>(       "mekong"    );
    auto congo      = performance_test::Topic<benchmark::msg::Congo>(                               "congo"     );
    auto lena       = performance_test::Topic<performance_test_msgs::msg::StampedVector>(       "lena"      );
    auto volga      = performance_test::Topic<benchmark::msg::Volga>(                               "volga"     );
    auto arkansas   = performance_test::Topic<benchmark::msg::Arkansas>(                            "arkansas"  );

    // Create nodes with pulishers and subscribers
    auto montreal = std::make_shared<performance_test::Node>("montreal", "",options.ipc);
    montreal->add_periodic_publisher(amazon, 10ms, rmw_qos_profile_default);
    montreal->add_periodic_publisher(nile,   10ms, rmw_qos_profile_default);
    montreal->add_periodic_publisher(ganges, 10ms, rmw_qos_profile_default);
    montreal->add_periodic_publisher(danube, 10ms, rmw_qos_profile_default);
    low_end_sys.add_node(montreal);

    auto lyon = std::make_shared<performance_test::Node>("lyon", "",options.ipc);
    lyon->add_subscriber(amazon, options.tracking_options, rmw_qos_profile_default);
    lyon->add_periodic_publisher(tigris, 10ms, rmw_qos_profile_default);
    low_end_sys.add_node(lyon);

    auto hamburg = std::make_shared<performance_test::Node>("hamburg", "",options.ipc);
    hamburg->add_subscriber(nile, options.tracking_options, rmw_qos_profile_default);
    hamburg->add_subscriber(tigris, options.tracking_options, rmw_qos_profile_default);
    hamburg->add_subscriber(ganges, options.tracking_options, rmw_qos_profile_default);
    hamburg->add_subscriber(danube, options.tracking_options, rmw_qos_profile_default);
    hamburg->add_periodic_publisher(parana, 10ms, rmw_qos_profile_default);
    low_end_sys.add_node(hamburg);

    auto osaka = std::make_shared<performance_test::Node>("osaka", "",options.ipc);
    osaka->add_subscriber(parana, options.tracking_options, rmw_qos_profile_default);
    osaka->add_periodic_publisher(salween, 100ms, rmw_qos_profile_default);
    low_end_sys.add_node(osaka);

    auto mandalay = std::make_shared<performance_test::Node>("mandalay", "",options.ipc);
    mandalay->add_subscriber(salween, options.tracking_options, rmw_qos_profile_default);
    mandalay->add_subscriber(danube, options.tracking_options, rmw_qos_profile_default);
    // For dynamic messages, we can specify the size with a 3rd argument
    mandalay->add_periodic_publisher(missouri, 100ms, rmw_qos_profile_default, 10000);
    low_end_sys.add_node(mandalay);

    auto ponce = std::make_shared<performance_test::Node>("ponce", "",options.ipc);
    ponce->add_subscriber(missouri, options.tracking_options, rmw_qos_profile_default);
    ponce->add_subscriber(danube, options.tracking_options, rmw_qos_profile_default);
    ponce->add_subscriber(volga, options.tracking_options, rmw_qos_profile_default);
    ponce->add_periodic_publisher(mekong, 500ms, rmw_qos_profile_default, 100);
    ponce->add_periodic_publisher(congo,  100ms, rmw_qos_profile_default);
    low_end_sys.add_node(ponce);

    auto barcelona = std::make_shared<performance_test::Node>("barcelona", "",options.ipc);
    barcelona->add_subscriber(mekong, options.tracking_options, rmw_qos_profile_default);
    barcelona->add_periodic_publisher(lena, 100ms, rmw_qos_profile_default, 50);
    low_end_sys.add_node(barcelona);

    auto georgetown = std::make_shared<performance_test::Node>("georgetown", "",options.ipc);
    georgetown->add_subscriber(lena, options.tracking_options, rmw_qos_profile_default);
    georgetown->add_periodic_publisher(volga, 500ms, rmw_qos_profile_default);
    low_end_sys.add_node(georgetown);

    auto geneva = std::make_shared<performance_test::Node>("geneva", "",options.ipc);
    geneva->add_subscriber(congo, options.tracking_options, rmw_qos_profile_default);
    geneva->add_subscriber(danube, options.tracking_options, rmw_qos_profile_default);
    geneva->add_subscriber(parana, options.tracking_options, rmw_qos_profile_default);
    geneva->add_periodic_publisher(arkansas, 100ms, rmw_qos_profile_default);
    low_end_sys.add_node(geneva);

    auto arequipa = std::make_shared<performance_test::Node>("arequipa", "",options.ipc);
    arequipa->add_subscriber(arkansas, options.tracking_options, rmw_qos_profile_default);
    low_end_sys.add_node(arequipa);

    low_end_sys.spin();

    ru_logger.stop();

    rclcpp::shutdown();

    std::this_thread::sleep_for(500ms);

    low_end_sys.print_latency_all_stats();
    std::cout << std::endl;
    std::cout << "System total:" << std::endl;
    low_end_sys.print_latency_total_stats();
    low_end_sys.save_latency_all_stats(latency_all_output_path);
    low_end_sys.save_latency_total_stats(latency_total_output_path);

    std::cout << std::endl;


}
