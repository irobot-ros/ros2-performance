#include <composition_benchmark/composable_node.hpp>
#include "factory_setup.hpp"
#include "run_test.hpp"

int main(int argc, char** argv)
{
  generic_factory_setup(argc, argv);
  run_test<ComposableNode>(argc, argv);
}
