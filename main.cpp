// log.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include <string>
//前端： 
#include <thread>
#include <ctime>
#include "pnlog.h"
#include "stl_entity.h"

int main()
{
    backend.open(2, new FileOutStream("e://log.txt"));
    StlEntity stl;
    stl.load("e://xl.stl");
    double grid_size = 0.005;
    auto try_result = stl.try_disperse(grid_size);
    if (try_result[0] * try_result[1] * try_result[2] > 10e8) {
      //log and exit
      capture.log_fatal(1, piece("网格尺寸过小 : ", grid_size));
    }
    else {
      stl.disperse(0.025);
      stl.out_to_tecplot("f://have_fun.txt");
    }
    //log end
    backend.stop();
    system("pause");
    return 0;
}