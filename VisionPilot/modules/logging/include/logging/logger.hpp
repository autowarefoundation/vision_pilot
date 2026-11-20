#ifndef VISIONPILOT_LOGGER_HPP
#define VISIONPILOT_LOGGER_HPP

// =============================================================================
// Logger.hpp
//
// Process-wide, allocation-free logger for automotive HPCs (Linux / QNX).
// Written to the MISRA C++:2023 guidelines (C++17).
//
// Usage
// -----
//    using logging::Logger;
//    using logging::Mode;
//
//    Logger& log{Logger::instance()};          // created on the first call
//    log.info("Pipeline up, {} cameras", count);           // level's mode
//    log.error("Camera {} lost", camId);                  // ERROR: Sync by default
//    log.warn(Mode::Sync, "Thermal limit {} C", temp);    // forced sync
//    log.error(Mode::Async, "Sensor {} noisy", id);       // forced async
//    log.error("Config load failed: {}", e.what());       // string: always sync
//    log.info("cte={:.2} m epsi={:+.3} rad", cte, epsi);  // double, formatted
//
//    No start() or shutdown(): the logger starts its background thread on
//    first use and flushes everything when the program exits normally.
//
// Arguments and placeholders
// --------------------------
//  Arguments (max 12): integers (not bool), float/double (not long double),
//  strings (char const*, char array, std::string_view, std::string).
//  Each placeholder takes the next argument, left to right:
//    {}       default: integers in decimal, floating point with 6 decimals
//    {:.N}    N decimals (N = 0..9) for float/double; ignored otherwise
//    {:+}     always print the sign of a number ("+" for zero and positive)
//    {:+.N}   both; a trailing 'f' is accepted: {:.4f} == {:.4}
//    {{ }}    literal '{' and '}'
//  A placeholder that cannot be parsed, or one without a remaining argument,
//  is printed as written. NaN prints "nan", infinities "inf"/"-inf"; values
//  too large for fixed-point output use d.ddde+XX.
//
// Modes
// -----
//  * Async: the call only stores the record in a lock-free ring buffer
//    (~20 ns). A background thread formats it and writes it out in batches,
//    normally within about 1 ms. Never blocks; drops when the buffer is full.
//  * Sync: the line has been written to the sink when the call returns.
//    Use it for lines that must survive a crash or precede a safe-state
//    transition.
//
//  Which mode a call uses:
//    1. a Mode passed in the call (log.info(Mode::Sync, ...)), otherwise
//    2. the level's setting from setMode(), otherwise
//    3. the defaults: INFO and WARN Async, ERROR Sync.
//  Exception: a call with a string argument is ALWAYS sync, whatever mode
//  is passed or configured. The text is formatted before the call returns,
//  so temporaries such as e.what() or a std::string are safe to pass.
//
// Ordering
// --------
//  A sync call first writes every async record queued before it (from any
//  thread), then its own line. So within one thread, lines appear in call
//  order whatever mix of modes is used. Across threads, order is the order
//  in which records reached the logger.
//  Edge case: if another thread was descheduled half-way through queuing a
//  record for longer than the bounded wait, the sync line is written without
//  it and syncOrderTimeouts() is incremented. The logger never hangs.
//
// Lifetime rules
// --------------
//  * The logger is destroyed during normal program exit (return from main,
//    std::exit), after flushing. It is NOT flushed on a crash, abort(),
//    _exit() or an unhandled signal such as SIGTERM/SIGKILL: use Mode::Sync
//    for lines that must survive those, and handle SIGTERM by returning from
//    main.
//  * Do not log from destructors of static or global objects, and stop other
//    threads from logging before main returns.
//  * Do not log from signal handlers.
//  * A sink passed to setSink() must stay alive until resetSink() is called
//    or the program ends.
// =============================================================================
#include <pthread.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>

namespace logging
{
    enum class LogLevel : std::uint8_t
    {
        Info = 0U,
        Warn = 1U,
        Error = 2U
    };

    enum class Mode : std::uint8_t
    {
        Async = 0U,
        Sync = 1U
    };

    /// Output destination. write() is always called with the logger's lock held,
    /// so it is never called concurrently. It receives one or more complete
    /// lines, each ending in '\n'.
    class LogSink
    {
    public:
        virtual ~LogSink() = default;
        virtual void write(char const* const data, std::size_t const length) noexcept = 0;

    protected:
        LogSink() noexcept = default;
        LogSink(LogSink const&) noexcept = default;
        LogSink(LogSink&&) noexcept = default;
        LogSink& operator=(LogSink const&) & noexcept = default;
        LogSink& operator=(LogSink&&) & noexcept = default;
    };

    /// Default sink: writes to standard output with the OS write() call, without
    /// stdio buffering (no reordering against other output, nothing lost in a
    /// stdio buffer). Implemented in the platform section of Logger.cpp.
    class ConsoleSink final : public LogSink
    {
    public:
        ConsoleSink() noexcept = default;
        ~ConsoleSink() override = default;
        ConsoleSink(ConsoleSink const&) = delete;
        ConsoleSink(ConsoleSink&&) = delete;
        ConsoleSink& operator=(ConsoleSink const&) = delete;
        ConsoleSink& operator=(ConsoleSink&&) = delete;

        void write(char const* const data, std::size_t const length) noexcept override;
    };

    namespace detail
    {
        /// 12 is the most arguments for which a queue Slot still fits in two
        /// cache lines (128 bytes); see the static_assert after Logger::Slot.
        constexpr std::size_t kMaxArgs{12U};
        constexpr std::size_t kLineCapacity{256U};
        constexpr std::size_t kCacheLineSize{64U};
        constexpr std::size_t kLevelCount{3U};

        /// Queued argument kinds are packed into LogRecord::kindBits.
        constexpr std::size_t kKindBitsPerArg{2U};
        constexpr std::uint32_t kKindMask{3U};
        static_assert((kMaxArgs * kKindBitsPerArg) <= 32U, "LogRecord::kindBits too narrow for kMaxArgs");

        /// Decimals printed for a floating-point argument by a plain {}.
        constexpr std::size_t kDefaultFloatPrecision{6U};

        constexpr std::string_view kDecimalDigits{"0123456789"};

        struct LogRecord
        {
            std::uint64_t timestamp{0U}; // nanoseconds, steady clock
            char const* message{nullptr};
            std::array<std::uint64_t, kMaxArgs> args{}; // integer value or IEEE 754 double bits
            // Ordered largest-first so the small members share one 8-byte word.
            std::uint32_t kindBits{0U}; // ArgKind per argument, kKindBitsPerArg bits each
            std::uint16_t messageLength{0U};
            std::uint8_t argCount{0U};
            LogLevel level{LogLevel::Info};
        };

        /// Monotonic time in nanoseconds (CLOCK_MONOTONIC on Linux and QNX).
        inline std::uint64_t monotonicNanoseconds() noexcept
        {
            auto const sinceEpoch{std::chrono::steady_clock::now().time_since_epoch()};
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(sinceEpoch).count());
        }

        template <typename T>
        constexpr bool isLoggableInteger() noexcept
        {
            return std::is_integral<T>::value && (!std::is_same<T, bool>::value);
        }

        template <typename T>
        constexpr bool isLoggableText() noexcept
        {
            using Element = std::remove_cv_t<std::remove_extent_t<T>>;
            return std::is_same<T, char const*>::value || std::is_same<T, char*>::value ||
                std::is_same<T, std::string_view>::value || std::is_same<T, std::string>::value ||
                (std::is_array<T>::value && std::is_same<Element, char>::value);
        }

        template <typename T>
        constexpr bool isLoggableFloat() noexcept
        {
            return std::is_same<T, float>::value || std::is_same<T, double>::value;
        }

        template <typename T>
        constexpr bool isLoggableArg() noexcept
        {
            return isLoggableInteger<T>() || isLoggableFloat<T>() || isLoggableText<T>();
        }

        /// Arguments that can be stored in a queued (async) record.
        template <typename T>
        constexpr bool isQueueableArg() noexcept
        {
            return isLoggableInteger<T>() || isLoggableFloat<T>();
        }

        template <typename... Ts>
        constexpr bool allQueueableArgs() noexcept
        {
            constexpr std::array<bool, sizeof...(Ts) + 1U> kChecks{{isQueueableArg<Ts>()..., true}};
            bool result{true};
            for (bool const ok : kChecks)
            {
                result = result && ok;
            }
            return result;
        }

        template <typename... Ts>
        constexpr bool allLoggableArgs() noexcept
        {
            constexpr std::array<bool, sizeof...(Ts) + 1U> kChecks{{isLoggableArg<Ts>()..., true}};
            bool result{true};
            for (bool const ok : kChecks)
            {
                result = result && ok;
            }
            return result;
        }

        template <typename... Ts>
        constexpr bool anyTextArg() noexcept
        {
            constexpr std::array<bool, sizeof...(Ts) + 1U> kChecks{{isLoggableText<Ts>()..., false}};
            bool result{false};
            for (bool const isText : kChecks)
            {
                result = result || isText;
            }
            return result;
        }

        /// The values of Unsigned, Signed and Float are stored in kKindBitsPerArg
        /// bits of LogRecord::kindBits; Text is never queued.
        enum class ArgKind : std::uint8_t
        {
            Unsigned = 0U,
            Signed = 1U,
            Float = 2U,
            Text = 3U
        };

        /// Decodes one queued argument kind (see LogRecord::kindBits).
        inline ArgKind decodeQueuedKind(std::uint32_t const code) noexcept
        {
            ArgKind kind{ArgKind::Unsigned};
            if (code == static_cast<std::uint32_t>(ArgKind::Signed))
            {
                kind = ArgKind::Signed;
            }
            else if (code == static_cast<std::uint32_t>(ArgKind::Float))
            {
                kind = ArgKind::Float;
            }
            else
            {
                // Unsigned (Text is never queued).
            }
            return kind;
        }

        /// IEEE 754 double <-> its 64-bit representation, so a double fits the
        /// integer argument slot. std::memcpy is the C++17 way to copy an object's
        /// representation (std::bit_cast is C++20); no cast is needed.
        static_assert(sizeof(double) == sizeof(std::uint64_t), "double must be 64 bits");
        static_assert(std::numeric_limits<double>::is_iec559, "double must be IEEE 754");

        inline std::uint64_t doubleToBits(double const value) noexcept
        {
            std::uint64_t bits{0U};
            static_cast<void>(std::memcpy(&bits, &value, sizeof(bits)));
            return bits;
        }

        inline double bitsToDouble(std::uint64_t const bits) noexcept
        {
            double value{0.0};
            static_cast<void>(std::memcpy(&value, &bits, sizeof(value)));
            return value;
        }

        template <typename T>
        constexpr ArgKind queuedKind() noexcept
        {
            ArgKind kind{ArgKind::Unsigned};
            if constexpr (isLoggableFloat<T>())
            {
                kind = ArgKind::Float;
            }
            else if constexpr (std::is_signed<T>::value)
            {
                kind = ArgKind::Signed;
            }
            else
            {
                // Unsigned integer.
            }
            return kind;
        }

        template <typename T>
        std::uint64_t queuedBits(T const value) noexcept
        {
            std::uint64_t bits{0U};
            if constexpr (isLoggableFloat<T>())
            {
                bits = doubleToBits(static_cast<double>(value)); // float -> double is exact
            }
            else
            {
                bits = static_cast<std::uint64_t>(value); // two's complement for signed
            }
            return bits;
        }

        /// One argument, ready to format. Text is referenced, not copied, so a
        /// FormatArg holding text is only valid until the logging call returns.
        struct FormatArg
        {
            std::uint64_t bits{0U};
            char const* text{nullptr};
            std::size_t textLength{0U};
            ArgKind kind{ArgKind::Unsigned};
        };

        /// Everything needed to format one line.
        struct LineSpec
        {
            std::uint64_t timestamp{0U};
            char const* message{nullptr};
            std::size_t messageLength{0U};
            std::array<FormatArg, kMaxArgs> args;
            std::size_t argCount{0U};
            LogLevel level{LogLevel::Info};
        };

        /// Length of a C string, scanning at most maxLength characters (anything
        /// longer is truncated in the line anyway).
        inline std::size_t boundedLength(char const* const text, std::size_t const maxLength) noexcept
        {
            std::size_t length{0U};
            while ((length < maxLength) && (text[length] != '\0'))
            {
                ++length;
            }
            return length;
        }

        template <typename T>
        FormatArg toFormatArg(T const& value) noexcept
        {
            static_assert(isLoggableArg<T>(),
                          "Log arguments must be integers (not bool), float/double or strings "
                          "(char const*, char array, std::string_view, std::string)");
            FormatArg arg{};
            if constexpr (isQueueableArg<T>())
            {
                arg.bits = queuedBits(value);
                arg.kind = queuedKind<T>();
            }
            else if constexpr (std::is_same<T, std::string_view>::value || std::is_same<T, std::string>::value)
            {
                arg.text = value.data();
                arg.textLength = value.size();
                arg.kind = ArgKind::Text;
            }
            else if constexpr (std::is_array<T>::value)
            {
                arg.text = &value[0];
                arg.textLength = boundedLength(&value[0], std::extent<T>::value);
                arg.kind = ArgKind::Text;
            }
            else
            {
                arg.text = (value != nullptr) ? value : "(null)"; // literal: static storage
                arg.textLength = boundedLength(arg.text, kLineCapacity);
                arg.kind = ArgKind::Text;
            }
            return arg;
        }

        template <std::size_t N, typename... Args>
        LineSpec makeLineSpec(LogLevel const level, std::uint64_t const timestamp, char const (&message)[N],
                              Args const&... args) noexcept
        {
            static_assert((N >= 1U) && (N <= 65536U), "Message length out of range");
            static_assert(sizeof...(Args) <= kMaxArgs, "Too many log arguments (max 12)");
            LineSpec spec{};
            spec.timestamp = timestamp;
            spec.message = &message[0];
            spec.messageLength = N - 1U;
            spec.level = level;
            spec.argCount = sizeof...(Args);
            std::array<FormatArg, sizeof...(Args)> const converted{{toFormatArg(args)...}};
            for (std::size_t i{0U}; i < converted.size(); ++i)
            {
                spec.args[i] = converted[i];
            }
            return spec;
        }

        template <std::size_t N, typename... Args>
        void fillRecord(LogRecord& record, LogLevel const level, std::uint64_t const timestamp,
                        char const (&message)[N], Args const&... args) noexcept
        {
            static_assert((N >= 1U) && (N <= 65536U), "Message length out of range");
            static_assert(sizeof...(Args) <= kMaxArgs, "Too many log arguments (max 12)");
            static_assert(allQueueableArgs<Args...>(),
                          "Queued log arguments must be integers (not bool) or float/double");

            record.timestamp = timestamp;
            record.message = &message[0];
            record.messageLength = static_cast<std::uint16_t>(N - 1U);
            record.level = level;
            record.argCount = static_cast<std::uint8_t>(sizeof...(Args));

            constexpr std::size_t kCount{sizeof...(Args)};
            std::array<std::uint64_t, kCount> const values{{queuedBits(args)...}};
            constexpr std::array<ArgKind, kCount> kKinds{{queuedKind<Args>()...}};
            std::uint32_t kinds{0U};
            for (std::size_t i{0U}; i < kCount; ++i)
            {
                record.args[i] = values[i];
                kinds |= (static_cast<std::uint32_t>(kKinds[i]) << (kKindBitsPerArg * i));
            }
            record.kindBits = kinds;
        }

        /// Fixed-capacity line buffer. Content is capped at kLineCapacity - 1 so the
        /// terminating '\n' always fits.
        class LineBuilder final
        {
        public:
            void reset() noexcept
            {
                length_ = 0U;
            }

            void appendChar(char const c) noexcept
            {
                std::size_t const current{length_};
                if (current < (kLineCapacity - 1U))
                {
                    buffer_[current] = c;
                    length_ = current + 1U;
                }
            }

            template <std::size_t N>
            void appendLiteral(char const (&text)[N]) noexcept
            {
                for (char const c : std::string_view{&text[0], N - 1U})
                {
                    appendChar(c);
                }
            }

            /// Appends text; line breaks become spaces (one record = one line).
            void appendText(char const* const text, std::size_t const length) noexcept
            {
                for (char const c : std::string_view{text, length})
                {
                    appendChar(((c == '\n') || (c == '\r')) ? ' ' : c);
                }
            }

            /// Appends 'value' in decimal, left-padded with '0' to at least minDigits.
            void appendUnsigned(std::uint64_t value, std::size_t const minDigits = 1U) noexcept
            {
                std::array<char, 20U> digits{}; // 2^64 - 1 has 20 decimal digits
                std::size_t count{0U};
                do
                {
                    digits[count] = kDecimalDigits[static_cast<std::size_t>(value % 10U)];
                    ++count;
                    value /= 10U;
                }
                while ((value != 0U) || ((count < minDigits) && (count < digits.size())));

                while (count > 0U)
                {
                    --count;
                    appendChar(digits[count]);
                }
            }

            /// 'bits' holds a two's-complement value; negation is done in unsigned
            /// arithmetic so INT64_MIN is handled without undefined behaviour.
            void appendSigned(std::uint64_t const bits, bool const forceSign) noexcept
            {
                constexpr std::uint64_t kSignBit{static_cast<std::uint64_t>(1U) << 63U};
                if ((bits & kSignBit) != 0U)
                {
                    appendChar('-');
                    appendUnsigned((~bits) + 1U);
                }
                else
                {
                    if (forceSign)
                    {
                        appendChar('+');
                    }
                    appendUnsigned(bits);
                }
            }

            /// Appends 'value' with 'precision' decimals (clamped to 0..9), rounded
            /// half away from zero. No <cstdio>: fixed-point conversion to integers.
            /// Every double -> integer conversion is range-checked first.
            void appendFloat(double const value, std::size_t const precision, bool const forceSign) noexcept
            {
                constexpr std::array<std::uint64_t, 10U> kPowersOfTen{
                    {1U, 10U, 100U, 1000U, 10000U, 100000U, 1000000U, 10000000U, 100000000U, 1000000000U}};
                constexpr double kTwoPow64{18446744073709551616.0};
                std::size_t const decimals{(precision < kPowersOfTen.size()) ? precision
                                                                              : (kPowersOfTen.size() - 1U)};
                if (std::isnan(value))
                {
                    appendLiteral("nan");
                }
                else
                {
                    bool const negative{std::signbit(value)};
                    double const magnitude{std::fabs(value)};
                    if (negative)
                    {
                        appendChar('-');
                    }
                    else if (forceSign)
                    {
                        appendChar('+');
                    }
                    else
                    {
                        // No sign for a positive value.
                    }

                    std::uint64_t const scale{kPowersOfTen[decimals]};
                    if (std::isinf(magnitude))
                    {
                        appendLiteral("inf");
                    }
                    else
                    {
                        double const scaled{std::floor((magnitude * static_cast<double>(scale)) + 0.5)};
                        if (scaled < kTwoPow64)
                        {
                            appendFixedPoint(static_cast<std::uint64_t>(scaled), scale, decimals);
                        }
                        else
                        {
                            // Too large for fixed point: d.ddd...e+XX. magnitude is
                            // finite, so the loop ends (at most ~308 iterations).
                            double mantissa{magnitude};
                            std::uint64_t exponent{0U};
                            while (mantissa >= 10.0)
                            {
                                mantissa /= 10.0;
                                ++exponent;
                            }
                            // mantissa < 10 and scale <= 10^9: fits in 64 bits.
                            std::uint64_t fixed{
                                static_cast<std::uint64_t>(std::floor((mantissa * static_cast<double>(scale)) + 0.5))};
                            if (fixed >= (10U * scale))
                            {
                                fixed /= 10U; // rounding carried into a new digit (9.99.. -> 10.0)
                                ++exponent;
                            }
                            appendFixedPoint(fixed, scale, decimals);
                            appendLiteral("e+");
                            appendUnsigned(exponent, static_cast<std::size_t>(2U));
                        }
                    }
                }
            }

            /// "[<seconds>.<microseconds>] ", like dmesg; seconds since boot on Linux.
            void appendTimestamp(std::uint64_t const nanoseconds) noexcept
            {
                constexpr std::uint64_t kNanosPerSecond{1000000000U};
                constexpr std::uint64_t kNanosPerMicro{1000U};
                appendChar('[');
                appendUnsigned(nanoseconds / kNanosPerSecond);
                appendChar('.');
                appendUnsigned((nanoseconds % kNanosPerSecond) / kNanosPerMicro, static_cast<std::size_t>(6U));
                appendLiteral("] ");
            }

            void finish() noexcept
            {
                buffer_[length_] = '\n';
                ++length_;
            }

            char const* data() const noexcept
            {
                return buffer_.data();
            }

            std::size_t length() const noexcept
            {
                return length_;
            }

        private:
            /// 'fixed' is the value times 'scale' (= 10^decimals): integer part,
            /// then '.', then the fraction zero-padded to 'decimals' digits.
            void appendFixedPoint(std::uint64_t const fixed, std::uint64_t const scale,
                                  std::size_t const decimals) noexcept
            {
                appendUnsigned(fixed / scale);
                if (decimals > 0U)
                {
                    appendChar('.');
                    appendUnsigned(fixed % scale, decimals);
                }
            }

            std::array<char, kLineCapacity> buffer_{};
            std::size_t length_{0U};
        };

        inline void appendLevel(LineBuilder& line, LogLevel const level) noexcept
        {
            switch (level)
            {
            case LogLevel::Info:
                line.appendLiteral("INFO ");
                break;
            case LogLevel::Warn:
                line.appendLiteral("WARN ");
                break;
            case LogLevel::Error:
                line.appendLiteral("ERROR");
                break;
            default:
                line.appendLiteral("?????");
                break;
            }
        }

        /// Options of one placeholder: {} {:.N} {:+} {:+.N} (optional trailing 'f').
        struct FormatSpec
        {
            std::size_t precision{kDefaultFloatPrecision};
            bool forceSign{false};
        };

        /// Value of a decimal digit character, without relying on the numerical
        /// values of characters (Rule 7.0.3).
        inline bool decimalDigitValue(char const c, std::size_t& value) noexcept
        {
            std::size_t const position{kDecimalDigits.find(c)};
            bool const found{position != std::string_view::npos};
            if (found)
            {
                value = position;
            }
            return found;
        }

        /// Parses the placeholder that starts at text[start] (a '{').
        /// Returns the number of characters it occupies and sets 'spec', or
        /// returns 0 (and leaves 'spec' unchanged) if it is not a valid placeholder.
        /// Never reads at or beyond text[length].
        inline std::size_t parsePlaceholder(char const* const text, std::size_t const length,
                                            std::size_t const start, FormatSpec& spec) noexcept
        {
            FormatSpec parsed{};
            bool valid{true};
            std::size_t position{start + 1U};
            if ((position < length) && (text[position] == ':'))
            {
                ++position;
                if ((position < length) && (text[position] == '+'))
                {
                    parsed.forceSign = true;
                    ++position;
                }
                if ((position < length) && (text[position] == '.'))
                {
                    ++position;
                    std::size_t digit{0U};
                    bool isDigit{false};
                    if (position < length)
                    {
                        isDigit = decimalDigitValue(text[position], digit);
                    }
                    if (isDigit)
                    {
                        parsed.precision = digit;
                        ++position;
                    }
                    else
                    {
                        valid = false;
                    }
                }
                if (valid && (position < length) && (text[position] == 'f'))
                {
                    ++position; // printf-style type letter, accepted and ignored
                }
            }

            std::size_t consumed{0U};
            if (valid && (position < length) && (text[position] == '}'))
            {
                spec = parsed;
                consumed = (position - start) + 1U;
            }
            return consumed;
        }

        inline void appendArg(LineBuilder& line, FormatArg const& arg, FormatSpec const& format) noexcept
        {
            switch (arg.kind)
            {
            case ArgKind::Signed:
                line.appendSigned(arg.bits, format.forceSign);
                break;
            case ArgKind::Float:
                line.appendFloat(bitsToDouble(arg.bits), format.precision, format.forceSign);
                break;
            case ArgKind::Text:
                line.appendText(arg.text, arg.textLength);
                break;
            case ArgKind::Unsigned:
            default:
                if (format.forceSign)
                {
                    line.appendChar('+');
                }
                line.appendUnsigned(arg.bits);
                break;
            }
        }

        /// Output format:  [<sec>.<usec>] <LEVEL> <message with placeholders substituted>\n
        inline void formatLine(LineSpec const& spec, LineBuilder& line) noexcept
        {
            line.reset();
            line.appendTimestamp(spec.timestamp);
            appendLevel(line, spec.level);
            line.appendChar(' ');

            std::size_t argIndex{0U};
            std::size_t i{0U};
            while (i < spec.messageLength)
            {
                char const c{spec.message[i]};
                bool const hasNext{(i + 1U) < spec.messageLength};
                FormatSpec format{};
                std::size_t consumed{0U};
                if ((c == '{') && (argIndex < spec.argCount))
                {
                    consumed = parsePlaceholder(spec.message, spec.messageLength, i, format);
                }

                if (consumed > 0U)
                {
                    appendArg(line, spec.args[argIndex], format);
                    ++argIndex;
                    i += consumed;
                }
                else if (hasNext && (((c == '{') && (spec.message[i + 1U] == '{')) ||
                                     ((c == '}') && (spec.message[i + 1U] == '}'))))
                {
                    line.appendChar(c); // "{{" -> "{", "}}" -> "}"
                    i += 2U;
                }
                else
                {
                    line.appendChar(c);
                    ++i;
                }
            }
            line.finish();
        }

        /// Formats a queued record (integers and floating point only).
        inline void formatRecord(LogRecord const& record, LineBuilder& line) noexcept
        {
            LineSpec spec;
            spec.timestamp = record.timestamp;
            spec.message = record.message;
            spec.messageLength = static_cast<std::size_t>(record.messageLength);
            spec.level = record.level;
            spec.argCount = static_cast<std::size_t>(record.argCount);
            std::size_t i{0U};
            while (i < spec.argCount)
            {
                std::uint32_t const code{
                    (static_cast<std::uint32_t>(record.kindBits) >> (kKindBitsPerArg * i)) & kKindMask};
                spec.args[i].bits = record.args[i];
                spec.args[i].kind = decodeQueuedKind(code);
                ++i;
            }
            formatLine(spec, line);
        }

        /// Mutex with priority inheritance where the OS supports it, so a
        /// low-priority thread holding it cannot block a real-time thread
        /// indefinitely. Implemented in the platform section of Logger.cpp.
        class PiMutex final
        {
        public:
            PiMutex() noexcept;
            ~PiMutex();
            PiMutex(PiMutex const&) = delete;
            PiMutex(PiMutex&&) = delete;
            PiMutex& operator=(PiMutex const&) = delete;
            PiMutex& operator=(PiMutex&&) = delete;

            void lock() noexcept;
            void unlock() noexcept;

        private:
            pthread_mutex_t mutex_;
        };

        class LockGuard final
        {
        public:
            explicit LockGuard(PiMutex& mutex) noexcept : mutex_{mutex}
            {
                mutex_.lock();
            }

            ~LockGuard()
            {
                mutex_.unlock();
            }

            LockGuard(LockGuard const&) = delete;
            LockGuard(LockGuard&&) = delete;
            LockGuard& operator=(LockGuard const&) = delete;
            LockGuard& operator=(LockGuard&&) = delete;

        private:
            PiMutex& mutex_;
        };
    } // namespace detail

    /// Process-wide logger. Obtain it with Logger::instance().
    class Logger final
    {
    public:
        /// Async records buffered. Must be a power of two. 128 bytes each (8 MiB).
        static constexpr std::size_t kQueueCapacity{65536U};
        static_assert((kQueueCapacity >= 2U) && ((kQueueCapacity & (kQueueCapacity - 1U)) == 0U),
                      "kQueueCapacity must be a power of two");
        static_assert(std::atomic<std::size_t>::is_always_lock_free,
                      "std::atomic<std::size_t> must be lock-free on this target");

        /// The one logger. Created, with its background thread, on the first call.
        static Logger& instance() noexcept;

        ~Logger();
        Logger(Logger const&) = delete;
        Logger(Logger&&) = delete;
        Logger& operator=(Logger const&) = delete;
        Logger& operator=(Logger&&) = delete;

        // ---- Logging: mode from the level's setting ----------------------------

        template <std::size_t N, typename... Args>
        void info(char const (&message)[N], Args const&... args) noexcept
        {
            logDefault(LogLevel::Info, message, args...);
        }

        template <std::size_t N, typename... Args>
        void warn(char const (&message)[N], Args const&... args) noexcept
        {
            logDefault(LogLevel::Warn, message, args...);
        }

        template <std::size_t N, typename... Args>
        void error(char const (&message)[N], Args const&... args) noexcept
        {
            logDefault(LogLevel::Error, message, args...);
        }

        template <std::size_t N, typename... Args>
        void log(LogLevel const level, char const (&message)[N], Args const&... args) noexcept
        {
            logDefault(level, message, args...);
        }

        // ---- Logging: explicit mode, overrides the level's setting -------------

        template <std::size_t N, typename... Args>
        void info(Mode const mode, char const (&message)[N], Args const&... args) noexcept
        {
            logWithMode(LogLevel::Info, mode, message, args...);
        }

        template <std::size_t N, typename... Args>
        void warn(Mode const mode, char const (&message)[N], Args const&... args) noexcept
        {
            logWithMode(LogLevel::Warn, mode, message, args...);
        }

        template <std::size_t N, typename... Args>
        void error(Mode const mode, char const (&message)[N], Args const&... args) noexcept
        {
            logWithMode(LogLevel::Error, mode, message, args...);
        }

        template <std::size_t N, typename... Args>
        void log(LogLevel const level, Mode const mode, char const (&message)[N],
                 Args const&... args) noexcept
        {
            logWithMode(level, mode, message, args...);
        }

        // ---- Configuration (thread-safe, any time) -----------------------------

        /// Records below this level are discarded (default: Info, i.e. none).
        void setMinimumLevel(LogLevel const level) noexcept
        {
            minimumLevel_.store(static_cast<std::uint8_t>(level), std::memory_order_relaxed);
        }

        LogLevel minimumLevel() const noexcept
        {
            return static_cast<LogLevel>(minimumLevel_.load(std::memory_order_relaxed));
        }

        /// Mode used by calls that do not pass one (default: ERROR Sync, rest Async).
        void setMode(LogLevel const level, Mode const newMode) noexcept
        {
            std::size_t const index{static_cast<std::size_t>(level)};
            if (index < detail::kLevelCount)
            {
                modes_[index].store(static_cast<std::uint8_t>(newMode), std::memory_order_relaxed);
            }
        }

        Mode mode(LogLevel const level) const noexcept
        {
            Mode result{Mode::Async};
            std::size_t const index{static_cast<std::size_t>(level)};
            if (index < detail::kLevelCount)
            {
                result = static_cast<Mode>(modes_[index].load(std::memory_order_relaxed));
            }
            return result;
        }

        /// Redirects output (e.g. to a file sink, or a capture sink in tests).
        /// Pending output is written to the old sink first. 'sink' must stay
        /// alive until resetSink() is called or the program ends.
        void setSink(LogSink& sink) noexcept;

        /// Returns output to the built-in console sink.
        void resetSink() noexcept;

        /// Writes everything queued so far before returning. Not needed in normal
        /// use; useful before an intentional abort() or in tests.
        void flush() noexcept;

        // ---- Diagnostics -------------------------------------------------------

        /// Async records discarded because the queue was full.
        std::size_t droppedCount() const noexcept
        {
            return dropped_.load(std::memory_order_relaxed);
        }

        /// Sync calls that stopped waiting for an unfinished record from another
        /// thread and may therefore appear ahead of it.
        std::size_t syncOrderTimeouts() const noexcept
        {
            return orderTimeouts_.load(std::memory_order_relaxed);
        }

    private:
        struct alignas(detail::kCacheLineSize) Slot
        {
            std::atomic<std::size_t> sequence{0U};
            detail::LogRecord record;
        };
        static_assert(sizeof(Slot) == (2U * detail::kCacheLineSize),
                      "Slot must fill exactly two cache lines; check kMaxArgs and LogRecord layout");

        Logger() noexcept;

        bool isEnabled(LogLevel const level) const noexcept
        {
            return static_cast<std::uint32_t>(level) >=
                static_cast<std::uint32_t>(minimumLevel_.load(std::memory_order_relaxed));
        }

        template <std::size_t N, typename... Args>
        void logDefault(LogLevel const level, char const (&message)[N], Args const&... args) noexcept
        {
            if (isEnabled(level))
            {
                emit(level, mode(level), message, args...);
            }
        }

        template <std::size_t N, typename... Args>
        void logWithMode(LogLevel const level, Mode const callMode, char const (&message)[N],
                         Args const&... args) noexcept
        {
            if (isEnabled(level))
            {
                emit(level, callMode, message, args...);
            }
        }

        template <std::size_t N, typename... Args>
        void emit(LogLevel const level, Mode const callMode, char const (&message)[N],
                  Args const&... args) noexcept
        {
            static_assert(sizeof...(Args) <= detail::kMaxArgs, "Too many log arguments (max 12)");
            static_assert(detail::allLoggableArgs<Args...>(),
                          "Log arguments must be integers (not bool), float/double or strings "
                          "(char const*, char array, std::string_view, std::string)");
            std::uint64_t const timestamp{detail::monotonicNanoseconds()};
            if constexpr (detail::anyTextArg<Args...>())
            {
                // Strings are only valid during this call: always sync.
                static_cast<void>(callMode);
                writeSync(detail::makeLineSpec(level, timestamp, message, args...));
            }
            else if (callMode == Mode::Sync)
            {
                writeSync(detail::makeLineSpec(level, timestamp, message, args...));
            }
            else
            {
                Slot* const slot{claimSlot()};
                if (slot != nullptr)
                {
                    detail::fillRecord(slot->record, level, timestamp, message, args...);
                    // The slot was claimed at 'sequence'; mark it readable.
                    std::size_t const sequence{slot->sequence.load(std::memory_order_relaxed)};
                    slot->sequence.store(sequence + 1U, std::memory_order_release);
                }
                else
                {
                    static_cast<void>(dropped_.fetch_add(1U, std::memory_order_relaxed));
                }
            }
        }

        /// Vyukov bounded queue, producer side (lock-free, multi-producer).
        /// Sequence comparisons use unsigned wrap-around, so counters may overflow.
        Slot* claimSlot() noexcept
        {
            constexpr std::size_t kHalfRange{std::numeric_limits<std::size_t>::max() / 2U};
            Slot* claimed{nullptr};
            bool done{false};
            std::size_t position{enqueuePosition_.load(std::memory_order_relaxed)};
            while (!done)
            {
                Slot& slot{slots_[position & (kQueueCapacity - 1U)]};
                std::size_t const sequence{slot.sequence.load(std::memory_order_acquire)};
                std::size_t const diff{sequence - position};
                if (diff == 0U)
                {
                    // compare_exchange_weak reloads 'position' on failure.
                    if (enqueuePosition_.compare_exchange_weak(position, position + 1U,
                                                               std::memory_order_relaxed,
                                                               std::memory_order_relaxed))
                    {
                        claimed = &slot;
                        done = true;
                    }
                }
                else if (diff > kHalfRange)
                {
                    done = true; // slot not yet consumed -> queue full
                }
                else
                {
                    position = enqueuePosition_.load(std::memory_order_relaxed);
                }
            }
            return claimed;
        }

        // Implemented in Logger.cpp. All "Locked" functions require lock_ held.
        void writeSync(detail::LineSpec const& spec) noexcept;
        void backgroundLoop() noexcept;
        std::size_t drainChunk() noexcept; // takes lock_ itself
        std::size_t drainLocked(std::size_t const maxRecords) noexcept;
        bool writeNextLocked() noexcept;
        bool tryDequeueLocked(detail::LogRecord& out) noexcept;
        void reportDropsLocked() noexcept;
        void emitLineLocked() noexcept;
        void flushBatchLocked() noexcept;

        static constexpr std::size_t kBatchCapacity{65536U};

        std::array<Slot, kQueueCapacity> slots_;
        alignas(detail::kCacheLineSize) std::atomic<std::size_t> enqueuePosition_;
        alignas(detail::kCacheLineSize) std::atomic<std::size_t> dropped_;
        std::atomic<std::size_t> orderTimeouts_;
        std::atomic<std::uint8_t> minimumLevel_;
        std::array<std::atomic<std::uint8_t>, detail::kLevelCount> modes_;
        std::atomic<bool> stopRequested_;

        // Guarded by lock_ (consumer side).
        alignas(detail::kCacheLineSize) detail::PiMutex lock_;
        std::size_t dequeuePosition_;
        std::size_t reportedDrops_;
        detail::LineBuilder line_;
        std::array<char, kBatchCapacity> batch_;
        std::size_t batchLength_;
        ConsoleSink console_;
        LogSink* sink_;

        std::thread worker_; // last: started after everything else is ready
    };

    // ---- Convenience API ------------------------------------------------------
    // logging::info("cte={:.2} m", cte);  No global object: each call forwards to
    // Logger::instance(), which creates the logger on first use. Configuration
    // (setSink, setMode, flush, ...) is still done through Logger::instance().

    template <std::size_t N, typename... Args>
    void info(char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().info(message, args...);
    }

    template <std::size_t N, typename... Args>
    void warn(char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().warn(message, args...);
    }

    template <std::size_t N, typename... Args>
    void error(char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().error(message, args...);
    }

    template <std::size_t N, typename... Args>
    void info(Mode const mode, char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().info(mode, message, args...);
    }

    template <std::size_t N, typename... Args>
    void warn(Mode const mode, char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().warn(mode, message, args...);
    }

    template <std::size_t N, typename... Args>
    void error(Mode const mode, char const (&message)[N], Args const&... args) noexcept
    {
        Logger::instance().error(mode, message, args...);
    }
} // namespace logging

#endif //VISIONPILOT_LOGGER_HPP
