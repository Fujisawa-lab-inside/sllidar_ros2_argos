// Device-free integration: real SDK unpacker callbacks, synthetic Ultra capsules.
#include "dataunpacker/dataunnpacker_commondef.h"
#include "dataunpacker/dataunpacker.h"
#include "sl_scan_diagnostics.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace sl::internal;
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
struct Sink : LIDARSampleDataListener {
    std::size_t nodes = 0, syncs = 0, byte_position = 0;
    sl::internal::LidarDiagnosticCounters diagnostics;
    void onHQNodeScanResetReq() override {}
    void onHQNodeDecoded(_u64, const rplidar_response_measurement_node_hq_t* n) override {
        ++nodes;
        if (n->flag & 1) ++syncs;
    }
    void onDecodingError(int error, _u8 format, const void* payload, size_t size) override {
        if (error == LIDARSampleDataUnpacker::ERR_EVENT_ON_EXP_CHECKSUM_ERR)
            diagnostics.checksumError(format, payload, size, byte_position);
    }
};
std::vector<_u8> stream() {
    std::vector<_u8> bytes;
    for (int k = 0; k < 160; ++k) {
        sl_lidar_response_ultra_capsule_measurement_nodes_t frame{};
        frame.start_angle_sync_q6 = ((k * 30) % 360) * 64;
        for (auto& c : frame.ultra_cabins) c.combined_x3 = 1000;
        const auto* p = reinterpret_cast<const _u8*>(&frame);
        _u8 checksum = 0;
        for (std::size_t i = 2; i < sizeof(frame); ++i) checksum ^= p[i];
        frame.s_checksum_1 = 0xa0 | (checksum & 15);
        frame.s_checksum_2 = 0x50 | (checksum >> 4);
        bytes.insert(bytes.end(), p, p + sizeof(frame));
    }
    return bytes;
}
void fragmented_faults_preserve_evidence_and_recover() {
    const std::array<const char*, 6> names{{
        "clean", "bit_flip", "delete_byte", "insert_byte", "sync_prefix", "repeated_failure"}};
    const std::array<unsigned, 6> errors{{0, 1, 1, 1, 0, 60}};
    for (std::size_t kind = 0; kind < names.size(); ++kind) {
        std::size_t first_nodes = 0, first_syncs = 0;
        for (std::size_t chunk : {1, 7, 132, 1000}) {
            auto bytes = stream();
            const std::size_t where = 12 * 132 + 20;
            if (kind == 1) bytes[where] ^= 1;
            if (kind == 2) bytes.erase(bytes.begin() + where);
            if (kind == 3) bytes.insert(bytes.begin() + where, 0x11);
            if (kind == 4) bytes.insert(bytes.begin() + 12 * 132, 0xa0);
            if (kind == 5) for (std::size_t k = 12; k < 72; ++k) bytes[k * 132 + 20] ^= 1;
            Sink sink;
            auto* unpacker = LIDARSampleDataUnpacker::CreateInstance(sink);
            sl::SlamtecLidarTimingDesc timing{};
            timing.sample_duration_uS = 125;
            timing.native_baudrate = 115200;
            timing.native_interface_type = sl::LIDAR_INTERFACE_UART;
            unpacker->updateUnpackerContext(LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING,
                                            &timing, sizeof(timing));
            unpacker->enable();
            std::size_t nodes_at_fault_end = 0;
            const auto fault_end = kind == 5 ? 72 * 132 : where + 1;
            for (std::size_t pos = 0; pos < bytes.size(); pos += chunk) {
                const auto size = std::min(chunk, bytes.size() - pos);
                sink.byte_position = pos + size;
                sink.diagnostics.received(size, sink.byte_position);
                unpacker->onSampleData(0x84, bytes.data() + pos, size);
                if (pos <= fault_end) nodes_at_fault_end = sink.nodes;
            }
            LIDARSampleDataUnpacker::ReleaseInstance(unpacker);
            const auto snapshot = sink.diagnostics.snapshot();
            require(snapshot.checksum_errors == errors[kind], "unexpected decoder error count");
            require(sink.nodes > nodes_at_fault_end && sink.syncs > 0, "decoder did not resume scans");
            if (chunk == 1) { first_nodes = sink.nodes; first_syncs = sink.syncs; }
            require(sink.nodes == first_nodes && sink.syncs == first_syncs,
                    "read fragmentation changed decoder recovery");
            for (const auto& e : snapshot.decode_error_samples) {
                if (!e.sequence) continue;
                require(e.answer_type == 0x84 && e.captured_size == 132 &&
                        e.checksum_available && e.checksum_expected != e.checksum_actual,
                        "actual decoder callback did not preserve bad capsule evidence");
                require(e.monotonic_us == e.last_rx_monotonic_us && e.rx_bytes == e.monotonic_us,
                        "failed packet not associated with receive context");
                if (kind == 1)
                    require(std::equal(e.payload.begin(), e.payload.begin() + 132, bytes.begin() + 12 * 132),
                            "recorded failure sample differs from input bytes");
            }
            if (kind == 5) require(snapshot.decode_error_sample_overwrites == 56,
                                   "sustained corruption did not bound diagnostic storage");
        }
    }
}
}  // namespace
int main() {
    try { fragmented_faults_preserve_evidence_and_recover(); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    return 0;
}
