#include "logger.h"

static std::shared_ptr<spdlog::logger> logger_;

std::shared_ptr<spdlog::logger>& logger() {
    return logger_;
}

void set_logger(std::shared_ptr<spdlog::logger> logger) {
    logger_ = logger;
}
