#pragma once

#include <functional>
#include <string>

namespace Exynos {

/// Searches for video devices named driverName. Found ones are checked with checker.
/// Returns file descriptor or -1 if nothing found
int OpenDevice(const std::string& driverName, std::function(bool(int)) checker);

} // namespace Exynos
