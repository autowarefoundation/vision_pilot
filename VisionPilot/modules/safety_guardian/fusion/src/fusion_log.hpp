#ifndef VISIONPILOT_FUSION_LOG_HPP
#define VISIONPILOT_FUSION_LOG_HPP

// Debug logging for the fusion module without <cstdio> (MISRA C++:2023
// RULE-30-0-1). Output matches VP_INFO: "[INFO]  <message>" on stdout.

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace visionpilot::fusion::detail {

// Same text as printf("%.<precision>f", value).
inline std::string fmt_fixed(float value, int precision)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << value;
    return os.str();
}

inline void log_info(const std::string& message)
{
    std::cout << "[INFO]  " << message << '\n';
}

}  // namespace visionpilot::fusion::detail

#endif // VISIONPILOT_FUSION_LOG_HPP
