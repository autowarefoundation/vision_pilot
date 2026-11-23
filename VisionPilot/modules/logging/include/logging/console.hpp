#ifndef VISIONPILOT_LOGGING_CONSOLE_HPP
#define VISIONPILOT_LOGGING_CONSOLE_HPP

// Console logging for VisionPilot without <cstdio> or function-like macros
// (MISRA C++:2023 RULE-30-0-1, RULE-19-0-2).
//
//   log_info : "[INFO]  <message>" on stdout
//   log_warn : "[WARN]  <message>" on stderr
//   log_error: "[ERROR] <message>" on stderr
//
// fmt_fixed / fmt_signed produce the same text as printf "%.Nf" / "%+.Nf",
// so messages can be built with std::string concatenation or an ostringstream.

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace logging {

// Same text as printf("%.<precision>f", value).
inline std::string fmt_fixed(double value, std::int32_t precision)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(static_cast<int>(precision)) << value;
    return os.str();
}

inline std::string fmt_fixed(float value, std::int32_t precision)
{
    return fmt_fixed(static_cast<double>(value), precision);
}

// Same text as printf("%+.<precision>f", value).
inline std::string fmt_signed(double value, std::int32_t precision)
{
    std::ostringstream os;
    os << std::showpos << std::fixed << std::setprecision(static_cast<int>(precision)) << value;
    return os.str();
}

inline std::string fmt_signed(float value, std::int32_t precision)
{
    return fmt_signed(static_cast<double>(value), precision);
}

inline void log_info(const std::string& message)
{
    std::cout << "[INFO]  " << message << '\n';
}

inline void log_warn(const std::string& message)
{
    std::cerr << "[WARN]  " << message << '\n';
}

inline void log_error(const std::string& message)
{
    std::cerr << "[ERROR] " << message << '\n';
}

}  // namespace logging

#endif // VISIONPILOT_LOGGING_CONSOLE_HPP
