#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace sl {

// Fixed storage for diagnostic evidence only; no packet is replayed to the decoder.
constexpr std::size_t kDecodeErrorSamples = 4;
constexpr std::size_t kDecodeErrorPayloadBytes = 256;
struct LidarDecodeErrorSample {
    std::uint64_t sequence = 0, monotonic_us = 0;
    std::uint64_t rx_bytes = 0, rx_reads = 0, last_rx_monotonic_us = 0;
    std::uint64_t payload_size = 0;
    std::uint32_t captured_size = 0;
    std::uint8_t answer_type = 0, checksum_expected = 0, checksum_actual = 0;
    bool checksum_available = false;
    std::array<std::uint8_t, kDecodeErrorPayloadBytes> payload{};
};

// Cumulative observations only; reading this snapshot does not touch the
// serial channel, decoder, scan buffers, or device state. Fields are sampled
// independently, not as a transaction across the SDK worker threads.
struct LidarDiagnosticSnapshot {
    std::uint64_t rx_bytes = 0;
    std::uint64_t rx_reads = 0;
    std::uint64_t rx_timeouts = 0;
    std::uint64_t rx_errors = 0;
    std::uint64_t decoder_queue_depth = 0;
    std::uint64_t decoder_queue_high_water = 0;
    std::uint64_t checksum_errors = 0;
    std::uint64_t encoder_resets = 0;
    std::uint64_t completed_scans = 0;
    std::uint64_t last_rx_monotonic_us = 0;
    std::uint64_t last_complete_monotonic_us = 0;
    std::uint64_t decode_error_sample_overwrites = 0;
    std::uint64_t decode_error_sample_drops = 0;
    bool decode_error_samples_available = false;
    std::array<LidarDecodeErrorSample, kDecodeErrorSamples> decode_error_samples{};
};

namespace internal {

class LidarDiagnosticCounters {
public:
    void received(std::uint64_t bytes, std::uint64_t now_us) {
        rx_bytes.fetch_add(bytes, std::memory_order_relaxed);
        rx_reads.fetch_add(1, std::memory_order_relaxed);
        last_rx_monotonic_us.store(now_us, std::memory_order_relaxed);
    }

    // Called while the existing RX queue lock is held, so depth cannot be
    // overwritten out of order by enqueue/dequeue operations.
    void queueDepth(std::uint64_t depth) {
        decoder_queue_depth.store(depth, std::memory_order_relaxed);
        auto peak = decoder_queue_high_water.load(std::memory_order_relaxed);
        while (peak < depth && !decoder_queue_high_water.compare_exchange_weak(
            peak, depth, std::memory_order_relaxed)) {}
    }

    void completed(std::uint64_t now_us) {
        completed_scans.fetch_add(1, std::memory_order_relaxed);
        last_complete_monotonic_us.store(now_us, std::memory_order_relaxed);
    }

    // Error path only. Both reader and producer try once: serial/decoder threads
    // never wait for diagnostics, allocate storage, or write logs/files here.
    void checksumError(std::uint8_t answer_type, const void* payload,
                       std::size_t size, std::uint64_t now_us) {
        const auto sequence = checksum_errors.fetch_add(1, std::memory_order_relaxed) + 1;
        if (sample_lock.test_and_set(std::memory_order_acquire)) {
            decode_error_sample_drops.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        auto& sample = samples[next_sample];
        if (sample.sequence) sample_overwrites.fetch_add(1, std::memory_order_relaxed);
        sample = LidarDecodeErrorSample{};
        sample.sequence = sequence;
        sample.monotonic_us = now_us;
        sample.answer_type = answer_type;
        sample.payload_size = size;
        sample.rx_bytes = rx_bytes.load(std::memory_order_relaxed);
        sample.rx_reads = rx_reads.load(std::memory_order_relaxed);
        sample.last_rx_monotonic_us = last_rx_monotonic_us.load(std::memory_order_relaxed);
        if (payload) {
            const auto* bytes = static_cast<const std::uint8_t*>(payload);
            sample.captured_size = std::min(size, kDecodeErrorPayloadBytes);
            std::copy_n(bytes, sample.captured_size, sample.payload.begin());
            // All express capsule variants encode an XOR of bytes [2, size)
            // in the low nibbles of bytes 0/1. HQ CRC and unknown formats are
            // explicitly unavailable instead of guessing a checksum algorithm.
            const bool capsule = answer_type == 0x82 || answer_type == 0x84 ||
                                 answer_type == 0x85 || answer_type == 0x86;
            if (capsule && size >= 2 && size <= kDecodeErrorPayloadBytes) {
                sample.checksum_available = true;
                sample.checksum_actual = (bytes[0] & 0xf) | ((bytes[1] & 0xf) << 4);
                for (std::size_t i = 2; i < size; ++i) sample.checksum_expected ^= bytes[i];
            }
        }
        next_sample = (next_sample + 1) % kDecodeErrorSamples;
        sample_lock.clear(std::memory_order_release);
    }

    LidarDiagnosticSnapshot snapshot() const {
        LidarDiagnosticSnapshot result;
#define SL_DIAGNOSTIC_LOAD(field) result.field = field.load(std::memory_order_relaxed)
        SL_DIAGNOSTIC_LOAD(rx_bytes);
        SL_DIAGNOSTIC_LOAD(rx_reads);
        SL_DIAGNOSTIC_LOAD(rx_timeouts);
        SL_DIAGNOSTIC_LOAD(rx_errors);
        SL_DIAGNOSTIC_LOAD(decoder_queue_depth);
        SL_DIAGNOSTIC_LOAD(decoder_queue_high_water);
        SL_DIAGNOSTIC_LOAD(checksum_errors);
        SL_DIAGNOSTIC_LOAD(encoder_resets);
        SL_DIAGNOSTIC_LOAD(completed_scans);
        SL_DIAGNOSTIC_LOAD(last_rx_monotonic_us);
        SL_DIAGNOSTIC_LOAD(last_complete_monotonic_us);
#undef SL_DIAGNOSTIC_LOAD
        result.decode_error_sample_drops = decode_error_sample_drops.load(std::memory_order_relaxed);
        result.decode_error_sample_overwrites = sample_overwrites.load(std::memory_order_relaxed);
        if (!sample_lock.test_and_set(std::memory_order_acquire)) {
            result.decode_error_samples_available = true;
            for (std::size_t i = 0; i < kDecodeErrorSamples; ++i)
                result.decode_error_samples[i] = samples[(next_sample + i) % kDecodeErrorSamples];
            sample_lock.clear(std::memory_order_release);
        }
        return result;
    }

    std::atomic<std::uint64_t> rx_bytes{0}, rx_reads{0}, rx_timeouts{0}, rx_errors{0};
    std::atomic<std::uint64_t> decoder_queue_depth{0}, decoder_queue_high_water{0};
    std::atomic<std::uint64_t> checksum_errors{0}, encoder_resets{0}, completed_scans{0};
    std::atomic<std::uint64_t> last_rx_monotonic_us{0}, last_complete_monotonic_us{0};

private:
    mutable std::atomic_flag sample_lock = ATOMIC_FLAG_INIT;
    std::array<LidarDecodeErrorSample, kDecodeErrorSamples> samples{};
    std::size_t next_sample = 0;
    std::atomic<std::uint64_t> sample_overwrites{0};
    std::atomic<std::uint64_t> decode_error_sample_drops{0};
};

}  // namespace internal
}  // namespace sl
