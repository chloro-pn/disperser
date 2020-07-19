#include <iostream>
#include <string> 
#include <thread>
#include <ctime>
#include "stl_entity.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"

int main(int argc, const char* argv[])
{
  if(argc != 3) {
    std::cerr << "args : input_file out_file_name." << std::endl;
    exit(-1);
  }
  auto console = spdlog::stdout_logger_st("console");
  console->set_pattern("[%@] <%!> {%c} |%l| %v");
  console->set_level(spdlog::level::trace);
  set_logger(console);

  StlEntity stl;
  stl.load(argv[1]);
  double grid_size = 2;
  auto try_result = stl.try_disperse(grid_size);
  if (try_result[0] * try_result[1] * try_result[2] > 10e8) {
    //log and exit
    SPDLOG_LOGGER_CRITICAL(logger(), "grid size too little {}", grid_size);
    spdlog::shutdown();
    exit(-1);
  }
  else {
    stl.disperse(grid_size);
    stl.out_to_tecplot(argv[2]);
  }
  return 0;
}
