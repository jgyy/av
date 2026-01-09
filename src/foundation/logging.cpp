#include "av/foundation/logging.hpp"

namespace av {

// Instantiate static members
bool Logger::initialized_ = true;  // Enabled by default
Logger::Level Logger::minLevel_ = Logger::DEBUG;

} // namespace av
