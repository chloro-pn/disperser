#include <iostream>
#include <string> 
#include <thread>
#include <ctime>
#include "../include/pnlog.h"
#include "../include/stl_entity.h"

using pnlog::backend;
using pnlog::capture;

int main()
{
    capture->setLevel(pnlog::CapTure::Level::PN_TRACE);
    backend->open(2, new pnlog::FileOutStream("log.txt"));
    backend->open(3, new pnlog::FileOutStream("boundary.txt"));
    StlEntity stl;
    stl.load("e://yuan.stl");
    double grid_size = 2;
    auto try_result = stl.try_disperse(grid_size);
    if (try_result[0] * try_result[1] * try_result[2] > 10e8) {
      //log and exit
      capture->log_fatal(1, piece("网格尺寸过小 : ", grid_size));
    }
    else {
      stl.disperse(grid_size);
      stl.out_to_tecplot("f://have_fun.txt");
    }
    system("pause");
    return 0;
}
