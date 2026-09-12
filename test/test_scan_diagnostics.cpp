#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "scan_diagnostics.h"
#include "sl_types.h"

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void timeout_then_recovery_retains_failure_evidence() {
    argos_lidar::ScanLoopDiagnostics diagnostic;
    sl::LidarDiagnosticSnapshot sdk;
    argos_lidar::ScanLoopSample healthy;
    healthy.published = true;
    healthy.grab_ms = 120.0;
    diagnostic.observe(healthy, sdk);
    require(diagnostic.shouldReport(10.0), "initial report missing");
    diagnostic.report(10.0, 10'000'000'000LL, sdk);

    // A completed two-second acquisition timeout emits no scan. Its report
    // distinguishes missing acquisition from the next successful attempt.
    auto timeout = healthy;
    timeout.grab_result = SL_RESULT_OPERATION_TIMEOUT;
    timeout.timeout = true;
    timeout.published = false;
    timeout.grab_ms = 2000.0;
    ++sdk.rx_timeouts;
    diagnostic.observe(timeout, sdk);
    require(diagnostic.consecutive_timeouts == 1, "timeout streak missing");
    require(diagnostic.shouldReport(12.0), "timeout report missing");
    const auto failed = diagnostic.report(12.0, 12'000'000'000LL, sdk);
    require(failed.find("\"max_grab_ms\":2000.000") != std::string::npos,
            "timeout duration lost");
    healthy.publish_interval_ms = 2540.0;
    diagnostic.observe(healthy, sdk);
    require(diagnostic.consecutive_timeouts == 0, "recovery did not clear streak");
    require(diagnostic.timeouts == 1 && diagnostic.failures == 1, "lost failure totals");
    require(diagnostic.published_scans == 2, "timeout counted as published data");
    require(!diagnostic.shouldReport(12.54), "unbounded recovery logging");
    require(diagnostic.shouldReport(13.0), "recovered failure was hidden");
    const auto report = diagnostic.report(13.0, 13'000'000'000LL, sdk);
    require(report.find("\"abnormal_since_last_report\":true") != std::string::npos,
            "recovery erased abnormal interval");
    require(report.find("\"max_publish_interval_ms\":2540.000") != std::string::npos,
            "timeout recovery publication gap lost");
    require(report.find("\"last_grab_result\":0") != std::string::npos,
            "latest success not distinguished from prior failure");
    require(!diagnostic.shouldReport(17.9) && diagnostic.shouldReport(18.0),
            "normal logging period not restored");
}

void slow_publisher_and_sdk_errors_are_distinct_from_grab_failure() {
    argos_lidar::ScanLoopDiagnostics diagnostic;
    sl::LidarDiagnosticSnapshot sdk;
    argos_lidar::ScanLoopSample sample;
    sample.published = true;
    sample.grab_ms = 120.0;
    diagnostic.observe(sample, sdk);
    diagnostic.report(20.0, 20'000'000'000LL, sdk);
    sample.publish_ms = 350.0;
    sample.publish_interval_ms = 480.0;
    ++sdk.checksum_errors;
    sdk.decoder_queue_high_water = 9;
    diagnostic.observe(sample, sdk);
    require(!diagnostic.shouldReport(20.1), "unbounded fault logging");
    // An intervening healthy sample must not erase the unreported fault.
    sample.publish_ms = 0.0;
    sample.publish_interval_ms = 120.0;
    diagnostic.observe(sample, sdk);
    require(diagnostic.shouldReport(21.0), "publish stall or decoder fault hidden");
    const auto report = diagnostic.report(21.0, 21'000'000'000LL, sdk);
    require(report.find("\"grab_failures\":0") != std::string::npos,
            "publish stall misclassified as grab failure");
    require(report.find("\"max_publish_ms\":350.000") != std::string::npos,
            "publish timing missing");
    require(report.find("\"checksum_errors\":1") != std::string::npos,
            "decoder fault missing");
}

void concurrent_sdk_counters_do_not_lose_events_or_mutate_on_snapshot() {
    sl::internal::LidarDiagnosticCounters counters;
    std::atomic<bool> reading{true};
    std::atomic<bool> monotonic{true};
    std::thread reader([&] {
        std::uint64_t previous = 0;
        while (reading.load()) {
            auto snapshot = counters.snapshot();
            if (snapshot.rx_bytes < previous) monotonic.store(false);
            previous = snapshot.rx_bytes;
        }
    });
    std::vector<std::thread> workers;
    for (unsigned worker = 0; worker < 4; ++worker) {
        workers.emplace_back([&, worker] {
            for (unsigned i = 0; i < 5000; ++i) {
                counters.received(84, i);
                counters.completed(i);
                counters.checksum_errors.fetch_add(1, std::memory_order_relaxed);
                counters.queueDepth(worker + 1);
            }
        });
    }
    for (auto& worker : workers) worker.join();
    reading.store(false);
    reader.join();
    counters.queueDepth(0);
    const auto first = counters.snapshot();
    const auto second = counters.snapshot();
    require(monotonic.load(), "concurrent snapshot went backwards");
    require(first.rx_bytes == 20000 * 84 && first.rx_reads == 20000,
            "concurrent receive events lost");
    require(first.completed_scans == 20000 && first.checksum_errors == 20000,
            "concurrent decoder events lost");
    require(first.decoder_queue_depth == 0 && first.decoder_queue_high_water == 4,
            "queue peak lost after drain");
    require(first.rx_bytes == second.rx_bytes && first.rx_reads == second.rx_reads,
            "snapshot reset cumulative counters");
}
void bounded_failure_samples_preserve_packet_and_checksum() {
    sl::internal::LidarDiagnosticCounters counters;
    std::array<std::uint8_t, 132> packet{};
    packet[0] = 0xa5;
    packet[1] = 0x50;
    packet[2] = 0x04;  // Expected XOR 4 differs from received checksum 5.
    counters.received(132, 200);
    for (unsigned i = 0; i < 10; ++i)
        counters.checksumError(0x84, packet.data(), packet.size(), 300 + i);
    // Caller storage is recycled immediately; the diagnostic must own its bytes.
    packet[2] = 0xff;
    auto sdk = counters.snapshot();
    require(sdk.checksum_errors == 10 && sdk.decode_error_sample_overwrites == 6,
            "bounded ring did not account for overwritten evidence");
    require(sdk.decode_error_sample_drops == 0, "uncontended capture dropped");
    for (unsigned i = 0; i < 4; ++i) {
        const auto& e = sdk.decode_error_samples[i];
        require(e.sequence == i + 7 && e.monotonic_us == i + 306,
                "failure samples not in monotonic capture order");
        require(e.answer_type == 0x84 && e.payload_size == 132 && e.captured_size == 132 &&
                e.payload[2] == 4 && e.checksum_available && e.checksum_expected == 4 &&
                e.checksum_actual == 5, "failed packet or checksum not preserved");
        require(e.rx_bytes == 132 && e.rx_reads == 1 && e.last_rx_monotonic_us == 200,
                "receive context missing");
    }
    argos_lidar::ScanLoopDiagnostics diagnostic;
    argos_lidar::ScanLoopSample sample;
    sample.published = true;
    diagnostic.observe(sample, sdk);
    const auto first = diagnostic.report(10, 100, sdk);
    require(first.find("\"checksum_expected\":4,\"checksum_actual\":5") != std::string::npos,
            "checksum evidence absent from diagnostic JSON");
    require(first.size() < 6000, "per-report diagnostic payload exceeds fixed upper bound");
    const auto repeated = diagnostic.report(11, 110, sdk);
    require(repeated.find("\"decode_error_samples\":[]") != std::string::npos,
            "already reported packet was logged repeatedly");
    require(!diagnostic.shouldReport(11.5), "new evidence bypassed log rate limit");

    std::array<std::uint8_t, 300> oversized{};
    counters.checksumError(0x99, oversized.data(), oversized.size(), 400);
    sdk = counters.snapshot();
    const auto& last = sdk.decode_error_samples.back();
    require(last.captured_size == 256 && last.payload_size == 300 && !last.checksum_available,
            "unknown/oversized packet not explicitly bounded");
    const auto truncated = diagnostic.report(12, 120, sdk);
    require(truncated.find("\"truncated\":true") != std::string::npos &&
            truncated.find("\"checksum_expected\":null") != std::string::npos,
            "unsupported checksum or truncation not visible");
}

void concurrent_failure_samples_never_block_or_tear() {
    sl::internal::LidarDiagnosticCounters counters;
    std::atomic<bool> done{false}, valid{true};
    std::thread reader([&] {
        while (!done.load()) {
            const auto sdk = counters.snapshot();
            for (const auto& e : sdk.decode_error_samples) {
                if (e.sequence && (e.payload[2] != (e.monotonic_us & 255) ||
                    e.captured_size != 132 || !e.checksum_available)) valid.store(false);
            }
        }
    });
    std::array<std::uint8_t, 132> packet{};
    packet[0] = 0xa5;
    packet[1] = 0x50;
    for (unsigned i = 0; i < 20000; ++i) {
        packet[2] = i & 255;
        counters.checksumError(0x84, packet.data(), packet.size(), i);
    }
    done.store(true);
    reader.join();
    const auto sdk = counters.snapshot();
    unsigned stored = 0;
    for (const auto& e : sdk.decode_error_samples) if (e.sequence) ++stored;
    require(valid.load(), "concurrent diagnostic snapshot tore a packet");
    require(sdk.checksum_errors == 20000 &&
            stored + sdk.decode_error_sample_overwrites + sdk.decode_error_sample_drops == 20000,
            "contention drops and ring overwrites do not account for all errors");
}
}  // namespace

int main() {
    try {
        timeout_then_recovery_retains_failure_evidence();
        slow_publisher_and_sdk_errors_are_distinct_from_grab_failure();
        concurrent_sdk_counters_do_not_lose_events_or_mutate_on_snapshot();
        bounded_failure_samples_preserve_packet_and_checksum();
        concurrent_failure_samples_never_block_or_tear();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
