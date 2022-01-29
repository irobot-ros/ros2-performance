#include <composition_benchmark/composable_node.hpp>
#include <composition_benchmark/helpers/helper_factory.hpp>
#include <composition_benchmark/helpers/helper_spin.hpp>
#include <composition_benchmark/helpers/run_test.hpp>

int main(int argc, char** argv)
{
  run_test(
    argc,
    argv,
    global_factory_create_generic_nodes<ComposableNode>,
    [](const NodesVector&) {sleep_task(MAX_HOURS);});
}
