#pragma once
#include "spdlog/spdlog.h"
#include <memory>

std::shared_ptr<spdlog::logger>& logger();

void set_logger(std::shared_ptr<spdlog::logger> logger);
