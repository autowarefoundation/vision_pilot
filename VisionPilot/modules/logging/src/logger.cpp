// =============================================================================
// Logger.cpp - logger implementation: singleton, background thread, sync
// writes, batching, plus the platform section (Linux / QNX).
//
// All OS calls (write, pthread) are confined to the section marked
// "PLATFORM SECTION" at the end of this file, so the MISRA analysis can
// treat it as platform glue; everything above it is portable ISO C++17.
// =============================================================================
#include <logging/logger.hpp>

#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <thread>

namespace logging
{
    namespace
    {
        /// Records written per lock hold by the background thread, so a sync caller
        /// never waits behind more than one chunk.
        constexpr std::size_t kDrainChunk{1024U};

        /// How often a sync call yields while waiting for a record that another
        /// thread has claimed but not yet finished writing (normally nanoseconds).
        constexpr std::size_t kSyncWaitYields{1000U};

        /// Background thread sleep when the queue is empty: upper bound on how long
        /// an async line waits before it is written.
        constexpr std::chrono::milliseconds kIdleSleep{1};
    } // namespace

    // MISRA C++:2023 deviation DEV-LOG-001 - Rule 6.7.1 (Required)
    // Local variable with static storage duration: Logger::instance()::s_instance.
    // Justification: the process-wide logger is a function-local static so that
    //  * it is created on first use, also when used during static initialisation
    //    of other translation units (no initialisation-order dependency);
    //  * its initialisation is thread-safe ([stmt.dcl]/4);
    //  * it is destroyed, after flushing, at normal program exit.
    // Alternatives rejected: a namespace-scope or static-member object violates
    // Rule 6.7.2 and has unspecified cross-TU initialisation order; allocating it
    // with new violates Rule 21.6.1/21.6.2 and is never flushed at exit.
    // Scope: this object only.
    Logger& Logger::instance() noexcept
    {
        static Logger s_instance{};
        return s_instance;
    }

    Logger::Logger() noexcept
        : enqueuePosition_{0U}, dropped_{0U}, orderTimeouts_{0U},
          minimumLevel_{static_cast<std::uint8_t>(LogLevel::Info)}, modes_{}, stopRequested_{false},
          lock_{}, dequeuePosition_{0U}, reportedDrops_{0U}, line_{}, batch_{}, batchLength_{0U},
          console_{}, sink_{&console_}, worker_{}
    {
        std::size_t index{0U};
        for (Slot& slot : slots_)
        {
            slot.sequence.store(index, std::memory_order_relaxed);
            ++index;
        }
        modes_[static_cast<std::size_t>(LogLevel::Info)].store(static_cast<std::uint8_t>(Mode::Async),
                                                               std::memory_order_relaxed);
        modes_[static_cast<std::size_t>(LogLevel::Warn)].store(static_cast<std::uint8_t>(Mode::Async),
                                                               std::memory_order_relaxed);
        modes_[static_cast<std::size_t>(LogLevel::Error)].store(static_cast<std::uint8_t>(Mode::Sync),
                                                                std::memory_order_relaxed);
        // Starting a thread synchronises with it, so everything above is visible.
        worker_ = std::thread{[this]() noexcept { backgroundLoop(); }};
    }

    Logger::~Logger()
    {
        stopRequested_.store(true, std::memory_order_release);
        if (worker_.joinable())
        {
            worker_.join();
        }
        detail::LockGuard const guard{lock_};
        static_cast<void>(drainLocked(kQueueCapacity));
        flushBatchLocked();
    }

    void Logger::setSink(LogSink& sink) noexcept
    {
        detail::LockGuard const guard{lock_};
        static_cast<void>(drainLocked(kQueueCapacity)); // queued records go to the old sink
        flushBatchLocked();
        sink_ = &sink;
    }

    void Logger::resetSink() noexcept
    {
        setSink(console_);
    }

    void Logger::flush() noexcept
    {
        detail::LockGuard const guard{lock_};
        static_cast<void>(drainLocked(kQueueCapacity));
        flushBatchLocked();
    }

    void Logger::writeSync(detail::LineSpec const& spec) noexcept
    {
        detail::LockGuard const guard{lock_};

        // Every async record claimed before this point must be written first, so
        // output keeps call order. Records claimed later may follow.
        std::size_t const target{enqueuePosition_.load(std::memory_order_acquire)};
        std::size_t waits{0U};
        while ((dequeuePosition_ != target) && (waits < kSyncWaitYields))
        {
            if (!writeNextLocked())
            {
                // Head slot claimed by another thread that has not finished
                // writing it. Bounded wait, so a descheduled thread cannot hang us.
                ++waits;
                std::this_thread::yield();
            }
        }
        if (dequeuePosition_ != target)
        {
            static_cast<void>(orderTimeouts_.fetch_add(1U, std::memory_order_relaxed));
        }
        reportDropsLocked();

        detail::formatLine(spec, line_);
        emitLineLocked();
        flushBatchLocked();
    }

    void Logger::backgroundLoop() noexcept
    {
        while (!stopRequested_.load(std::memory_order_acquire))
        {
            std::size_t const written{drainChunk()};
            if (written == 0U)
            {
                std::this_thread::sleep_for(kIdleSleep);
            }
        }
    }

    /// Writes up to kDrainChunk queued records and flushes them, holding lock_
    /// only for this chunk so a sync caller never waits behind more than one.
    std::size_t Logger::drainChunk() noexcept
    {
        detail::LockGuard const guard{lock_};
        std::size_t const written{drainLocked(kDrainChunk)};
        flushBatchLocked();
        return written;
    }

    std::size_t Logger::drainLocked(std::size_t const maxRecords) noexcept
    {
        std::size_t processed{0U};
        bool more{true};
        while (more && (processed < maxRecords))
        {
            more = writeNextLocked();
            if (more)
            {
                ++processed;
            }
        }
        reportDropsLocked();
        return processed;
    }

    bool Logger::writeNextLocked() noexcept
    {
        detail::LogRecord record;
        bool const ready{tryDequeueLocked(record)};
        if (ready)
        {
            detail::formatRecord(record, line_);
            emitLineLocked();
        }
        return ready;
    }

    bool Logger::tryDequeueLocked(detail::LogRecord& out) noexcept
    {
        bool success{false};
        Slot& slot{slots_[dequeuePosition_ & (kQueueCapacity - 1U)]};
        std::size_t const sequence{slot.sequence.load(std::memory_order_acquire)};
        if (sequence == (dequeuePosition_ + 1U))
        {
            out = slot.record; // copy out, then free the slot immediately
            slot.sequence.store(dequeuePosition_ + kQueueCapacity, std::memory_order_release);
            ++dequeuePosition_;
            success = true;
        }
        return success;
    }

    void Logger::reportDropsLocked() noexcept
    {
        std::size_t const total{dropped_.load(std::memory_order_relaxed)};
        if (total != reportedDrops_)
        {
            line_.reset();
            detail::appendLevel(line_, LogLevel::Warn);
            line_.appendLiteral(" logger: ");
            line_.appendUnsigned(static_cast<std::uint64_t>(total - reportedDrops_));
            line_.appendLiteral(" records dropped (queue full)");
            line_.finish();
            emitLineLocked();
            reportedDrops_ = total;
        }
    }

    void Logger::emitLineLocked() noexcept
    {
        std::size_t const length{line_.length()};
        if ((batchLength_ + length) > kBatchCapacity)
        {
            flushBatchLocked();
        }
        char const* const data{line_.data()};
        static_cast<void>(std::copy_n(data, length,
                                      std::next(batch_.begin(), static_cast<std::ptrdiff_t>(batchLength_))));
        batchLength_ += length;
    }

    void Logger::flushBatchLocked() noexcept
    {
        if (batchLength_ > 0U)
        {
            sink_->write(batch_.data(), batchLength_);
            batchLength_ = 0U;
        }
    }
} // namespace logging

// =============================================================================
// PLATFORM SECTION (POSIX: Linux, QNX)
// The only code in the logger that calls the operating system.
// =============================================================================

namespace logging
{
    void ConsoleSink::write(char const* const data, std::size_t const length) noexcept
    {
        std::size_t written{0U};
        bool failed{false};
        while ((written < length) && (!failed))
        {
            ssize_t const result{::write(STDOUT_FILENO, &data[written], length - written)};
            if (result > 0)
            {
                written += static_cast<std::size_t>(result);
            }
            else if ((result < 0) && (errno == EINTR))
            {
                // Interrupted by a signal before anything was written: retry.
            }
            else
            {
                failed = true; // stdout closed or broken: drop silently
            }
        }
    }

    namespace detail
    {
        PiMutex::PiMutex() noexcept : mutex_{}
        {
            pthread_mutexattr_t attributes{};
            bool initialised{false};
            if (pthread_mutexattr_init(&attributes) == 0)
            {
                if (pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT) == 0)
                {
                    initialised = (pthread_mutex_init(&mutex_, &attributes) == 0);
                }
                static_cast<void>(pthread_mutexattr_destroy(&attributes));
            }
            if (!initialised)
            {
                // Priority inheritance unavailable: fall back to a default mutex.
                static_cast<void>(pthread_mutex_init(&mutex_, nullptr));
            }
        }

        PiMutex::~PiMutex()
        {
            static_cast<void>(pthread_mutex_destroy(&mutex_));
        }

        void PiMutex::lock() noexcept
        {
            static_cast<void>(pthread_mutex_lock(&mutex_));
        }

        void PiMutex::unlock() noexcept
        {
            static_cast<void>(pthread_mutex_unlock(&mutex_));
        }
    } // namespace detail
} // namespace logging
