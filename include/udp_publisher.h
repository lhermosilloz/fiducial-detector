#pragma once

// ---------------------------------------------------------------------------
// Serializes FiducialFrame protobuf messages and sends them as UDP datagrams.
//
// File name and role mirror jetson-vision-service's udp_publisher so the two
// services read the same way.
//
// One socket per process serves every stream; datagrams carry camera_id and the
// consumer demultiplexes on it.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

#include <netinet/in.h>

#include "detector.h"

namespace fd
{

class UdpPublisher
{
public:
    struct Config
    {
        std::string dest_ip   = "127.0.0.1";
        uint16_t    dest_port = 5602;
        std::string camera_id;
    };

    explicit UdpPublisher(Config cfg);
    ~UdpPublisher();

    UdpPublisher(const UdpPublisher&)            = delete;
    UdpPublisher& operator=(const UdpPublisher&) = delete;

    bool init(std::string& err);

    /// Publish a frame carrying detections.
    /// `capture_mono_us` is the sensor exposure time, not the current time.
    void send(uint64_t capture_mono_us,
              int32_t  frame_id,
              uint32_t frame_width,
              uint32_t frame_height,
              const std::vector<Detection>& detections,
              float    detect_ms);

    /// Publish an empty frame with heartbeat = true.
    ///
    /// Without heartbeats, a crashed detector and an empty field of view are
    /// indistinguishable. Liveness is tracked per camera, so each stream
    /// heartbeats independently.
    void sendHeartbeat(uint64_t capture_mono_us,
                       int32_t  frame_id,
                       uint32_t frame_width,
                       uint32_t frame_height);

    /// Datagrams that sendto() refused, for the stats line.
    uint64_t sendErrors() const { return send_errors_; }
    /// Largest datagram emitted so far, in bytes. Tracked because datagrams
    /// past roughly 1472 bytes fragment on a normal Ethernet path.
    size_t   maxDatagramBytes() const { return max_bytes_; }

private:
    void doSend(const std::string& serialized);

    Config             cfg_;
    int                sock_fd_ = -1;
    struct sockaddr_in dest_{};
    uint64_t           send_errors_ = 0;
    size_t             max_bytes_   = 0;
    bool               warned_mtu_  = false;
};

}  // namespace fd
