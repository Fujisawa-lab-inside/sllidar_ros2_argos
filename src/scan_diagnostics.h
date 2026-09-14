#pragma once

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "sl_scan_diagnostics.h"

namespace argos_lidar {

struct ScanLoopSample {
    std::uint32_t grab_result = 0;
    bool timeout = false;
    bool published = false;
    double grab_ms = 0.0;
    double prepare_ms = 0.0;
    double publish_ms = 0.0;
    double spin_ms = 0.0;
    double loop_ms = 0.0;
    double publish_interval_ms = 0.0;
};

// Reporting policy is independent of ROS/device clocks and performs no I/O.
// Failures remain visible even if a successful scan arrives before the next
// permitted log. Sampling or emitting diagnostics never retries an operation.
class ScanLoopDiagnostics {
public:
    void observe(const ScanLoopSample& sample, const sl::LidarDiagnosticSnapshot& sdk) {
        latest = sample;
        ++attempts;
        if (sample.grab_result != 0) {
            ++failures;
            ++consecutive_failures;
            last_failure_result = sample.grab_result;
        } else {
            consecutive_failures = 0;
        }
        if (sample.timeout) {
            ++timeouts;
            ++consecutive_timeouts;
        } else {
            consecutive_timeouts = 0;
        }
        if (sample.published) ++published_scans;
        peak.grab_ms = std::max(peak.grab_ms, sample.grab_ms);
        peak.prepare_ms = std::max(peak.prepare_ms, sample.prepare_ms);
        peak.publish_ms = std::max(peak.publish_ms, sample.publish_ms);
        peak.spin_ms = std::max(peak.spin_ms, sample.spin_ms);
        peak.loop_ms = std::max(peak.loop_ms, sample.loop_ms);
        peak.publish_interval_ms = std::max(peak.publish_interval_ms, sample.publish_interval_ms);
        abnormal = abnormal || sample.grab_result != 0 || !sample.published ||
            sample.grab_ms > 250.0 || sample.prepare_ms > 20.0 ||
            sample.publish_ms > 20.0 || sample.spin_ms > 20.0 ||
            sample.publish_interval_ms > 250.0 || sdk.rx_errors > previous_sdk.rx_errors ||
            sdk.rx_timeouts > previous_sdk.rx_timeouts ||
            sdk.checksum_errors > previous_sdk.checksum_errors ||
            sdk.encoder_resets > previous_sdk.encoder_resets;
        previous_sdk = sdk;
    }

    bool shouldReport(double steady_sec) const {
        return !reported || steady_sec - last_report_sec >= (abnormal ? 1.0 : 5.0);
    }

    std::string report(double steady_sec, std::int64_t ros_stamp_ns,
                       const sl::LidarDiagnosticSnapshot& sdk) {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::fixed << std::setprecision(3)
            << "{\"schema_version\":2,\"kind\":\"sllidar_diagnostics\",\"stamp_ns\":\""
            << ros_stamp_ns << "\",\"abnormal_since_last_report\":" << (abnormal ? "true" : "false")
            << ",\"attempts\":" << attempts << ",\"published_scans\":" << published_scans
            << ",\"grab_failures\":" << failures << ",\"grab_timeouts\":" << timeouts
            << ",\"consecutive_grab_failures\":" << consecutive_failures
            << ",\"consecutive_timeouts\":" << consecutive_timeouts
            << ",\"last_grab_result\":" << latest.grab_result
            << ",\"last_failure_result\":" << last_failure_result;
#define ARGOS_LIDAR_TIMING(field) \
        out << ",\"" #field "\":" << latest.field << ",\"max_" #field "\":" << peak.field
        ARGOS_LIDAR_TIMING(grab_ms);
        ARGOS_LIDAR_TIMING(prepare_ms);
        ARGOS_LIDAR_TIMING(publish_ms);
        ARGOS_LIDAR_TIMING(spin_ms);
        ARGOS_LIDAR_TIMING(loop_ms);
        ARGOS_LIDAR_TIMING(publish_interval_ms);
#undef ARGOS_LIDAR_TIMING
#define ARGOS_LIDAR_COUNTER(field) out << ",\"" #field "\":" << sdk.field
        ARGOS_LIDAR_COUNTER(rx_bytes);
        ARGOS_LIDAR_COUNTER(rx_reads);
        ARGOS_LIDAR_COUNTER(rx_timeouts);
        ARGOS_LIDAR_COUNTER(rx_errors);
        ARGOS_LIDAR_COUNTER(decoder_queue_depth);
        ARGOS_LIDAR_COUNTER(decoder_queue_high_water);
        ARGOS_LIDAR_COUNTER(checksum_errors);
        ARGOS_LIDAR_COUNTER(encoder_resets);
        ARGOS_LIDAR_COUNTER(completed_scans);
        ARGOS_LIDAR_COUNTER(last_rx_monotonic_us);
        ARGOS_LIDAR_COUNTER(last_complete_monotonic_us);
        ARGOS_LIDAR_COUNTER(decode_error_sample_drops);
        ARGOS_LIDAR_COUNTER(decode_error_sample_overwrites);
#undef ARGOS_LIDAR_COUNTER
        out << ",\"decode_error_samples_available\":"
            << (sdk.decode_error_samples_available ? "true" : "false")
            << ",\"decode_error_samples\":[";
        bool first = true;
        for (const auto& sample : sdk.decode_error_samples) {
            if (!sdk.decode_error_samples_available || sample.sequence <= last_error_sequence) continue;
            if (!first) out << ',';
            first = false;
            out << "{\"sequence\":" << sample.sequence
                << ",\"monotonic_us\":" << sample.monotonic_us
                << ",\"answer_type\":" << static_cast<unsigned>(sample.answer_type)
                << ",\"payload_size\":" << sample.payload_size
                << ",\"captured_size\":" << sample.captured_size
                << ",\"truncated\":" << (sample.payload_size > sample.captured_size ? "true" : "false")
                << ",\"rx_bytes\":" << sample.rx_bytes << ",\"rx_reads\":" << sample.rx_reads
                << ",\"last_rx_monotonic_us\":" << sample.last_rx_monotonic_us
                << ",\"checksum_algorithm\":\"" << (sample.checksum_available ? "capsule_xor8" : "unavailable")
                << "\",\"checksum_expected\":";
            if (sample.checksum_available) out << static_cast<unsigned>(sample.checksum_expected);
            else out << "null";
            out << ",\"checksum_actual\":";
            if (sample.checksum_available) out << static_cast<unsigned>(sample.checksum_actual);
            else out << "null";
            out << ",\"payload_hex\":\"";
            constexpr char hex[] = "0123456789abcdef";
            for (std::size_t i = 0; i < sample.captured_size; ++i)
                out << hex[sample.payload[i] >> 4] << hex[sample.payload[i] & 0xf];
            out << "\"}";
            last_error_sequence = sample.sequence;
        }
        out << "]}";
        reported = true;
        last_report_sec = steady_sec;
        abnormal = false;
        peak = ScanLoopSample{};
        return out.str();
    }

    std::uint64_t attempts = 0, failures = 0, timeouts = 0, published_scans = 0;
    std::uint64_t consecutive_failures = 0, consecutive_timeouts = 0;
    std::uint32_t last_failure_result = 0;

private:
    ScanLoopSample latest, peak;
    sl::LidarDiagnosticSnapshot previous_sdk;
    bool abnormal = false, reported = false;
    double last_report_sec = 0.0;
    std::uint64_t last_error_sequence = 0;
};

}  // namespace argos_lidar
