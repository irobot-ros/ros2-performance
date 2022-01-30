#include <composition_benchmark/base_node.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>

int main(int argc, char ** argv)
{
  run_test(
    argc,
    argv,
    create_simple_nodes<BaseNode>,
    [](const NodesVector&) {sleep_task(MAX_HOURS);});
}
