#include <composition_benchmark/base_node.hpp>
#include "factory_setup.hpp"
#include "run_test.hpp"

int main(int argc, char** argv)
{
  generic_factory_setup(argc, argv);
  run_test<BaseNode>(argc, argv);
}
