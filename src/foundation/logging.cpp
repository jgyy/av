#include "av/foundation/logging.hpp"

namespace av {

// Instantiate static member
std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;

} // namespace av
