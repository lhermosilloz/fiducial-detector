#include "udp_publisher.h"

#include "clock_domain.h"
#include "fiducial.pb.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace fd
{
namespace
{

/// Datagrams larger than this fragment on a normal Ethernet path, where losing
/// one fragment drops the whole datagram. A frame with a handful of detections
/// is roughly 100-300 bytes, so exceeding this indicates a structural problem.
constexpr size_t kMtuWarnBytes = 1400;

void fillFrameHeader(nexus::FiducialFrame& msg,
                     const std::string& camera_id,
                     uint64_t capture_mono_us,
                     int32_t frame_id,
                     uint32_t w,
                     uint32_t h)
{
    msg.set_capture_timestamp_us(capture_mono_us);
    msg.set_publish_timestamp_us(monotonicUs());
    msg.set_clock_domain(clockDomain());
    msg.set_frame_id(frame_id);
    msg.set_frame_width(w);
    msg.set_frame_height(h);
    msg.set_camera_id(camera_id);

    // Same instant as capture_timestamp_us, expressed on CLOCK_REALTIME, so a
    // consumer on another machine can estimate the offset between the two
    // monotonic timebases without a separate time-sync protocol.
    const int64_t rt = static_cast<int64_t>(capture_mono_us) + realtimeMinusMonotonicUs();
    msg.set_capture_realtime_us(rt > 0 ? static_cast<uint64_t>(rt) : 0);
}

}  // namespace

// ---------------------------------------------------------------------------

UdpPublisher::UdpPublisher(Config cfg) : cfg_(std::move(cfg)) {}

UdpPublisher::~UdpPublisher()
{
    if (sock_fd_ >= 0) ::close(sock_fd_);
}

bool UdpPublisher::init(std::string& err)
{
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
    {
        err = std::string("socket() failed: ") + std::strerror(errno);
        return false;
    }

    dest_.sin_family = AF_INET;
    dest_.sin_port   = htons(cfg_.dest_port);
    if (::inet_pton(AF_INET, cfg_.dest_ip.c_str(), &dest_.sin_addr) <= 0)
    {
        err = "invalid udp.dest_ip: " + cfg_.dest_ip;
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------

void UdpPublisher::doSend(const std::string& serialized)
{
    if (sock_fd_ < 0) return;

    if (serialized.size() > max_bytes_) max_bytes_ = serialized.size();
    if (serialized.size() > kMtuWarnBytes && !warned_mtu_)
    {
        std::cerr << "[udp/" << cfg_.camera_id << "] datagram is " << serialized.size()
                  << " bytes, past the ~1472-byte non-fragmenting Ethernet MTU. "
                     "One lost fragment will now drop the whole frame.\n";
        warned_mtu_ = true;
    }

    const ssize_t sent = ::sendto(sock_fd_, serialized.data(), serialized.size(), 0,
                                  reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
    if (sent < 0)
    {
        // UDP is connectionless: with no listener on loopback the kernel
        // reports ECONNREFUSED from a previous datagram's ICMP. This is normal
        // before the consumer starts, so count it rather than logging each one.
        ++send_errors_;
    }
}

// ---------------------------------------------------------------------------

void UdpPublisher::send(uint64_t capture_mono_us,
                        int32_t  frame_id,
                        uint32_t frame_width,
                        uint32_t frame_height,
                        const std::vector<Detection>& detections,
                        float    detect_ms)
{
    nexus::FiducialFrame msg;
    fillFrameHeader(msg, cfg_.camera_id, capture_mono_us, frame_id, frame_width, frame_height);
    msg.set_heartbeat(false);
    msg.set_detect_ms(detect_ms);

    for (const auto& d : detections)
    {
        auto* pd = msg.add_detections();
        pd->set_tag_id(d.tag_id);
        pd->set_family(d.family);
        pd->set_size_m(d.size_m);

        for (int i = 0; i < 3; ++i)
            pd->add_t_tag_wrt_cam(static_cast<float>(d.t_tag_wrt_cam[i]));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                pd->add_r_tag_to_cam(static_cast<float>(d.r_tag_to_cam(r, c)));

        pd->set_reproj_error_px(d.reproj_error_px);
        pd->set_camera_id(cfg_.camera_id);

        // loc_type 0: camera-frame pose only. Neither platform has a world
        // frame to offer, so t_tag_wrt_fixed / r_tag_to_fixed are left empty
        // rather than carrying a fabricated identity pose.
        pd->set_loc_type(0);

        for (const auto& c : d.corners)
        {
            pd->add_corners_px(c.x);
            pd->add_corners_px(c.y);
        }
    }

    std::string buf;
    if (!msg.SerializeToString(&buf))
    {
        std::cerr << "[udp/" << cfg_.camera_id << "] SerializeToString failed\n";
        return;
    }
    doSend(buf);
}

// ---------------------------------------------------------------------------

void UdpPublisher::sendHeartbeat(uint64_t capture_mono_us,
                                 int32_t  frame_id,
                                 uint32_t frame_width,
                                 uint32_t frame_height)
{
    nexus::FiducialFrame msg;
    fillFrameHeader(msg, cfg_.camera_id, capture_mono_us, frame_id, frame_width, frame_height);
    msg.set_heartbeat(true);
    msg.set_detect_ms(0.0f);

    std::string buf;
    if (!msg.SerializeToString(&buf))
    {
        std::cerr << "[udp/" << cfg_.camera_id << "] heartbeat SerializeToString failed\n";
        return;
    }
    doSend(buf);
}

}  // namespace fd
