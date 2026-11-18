#include "kinectv2_sdk.h"

#include <cstdio>

int main() {
    Kv2Config cfg{};
    kv2_default_config(&cfg);
    cfg.stream_flags = KV2_STREAM_DEFAULT;

    kv2::Device dev(&cfg);
    dev.start();

    std::printf("Connected to Kinect v2 (serial=%s, pipeline=%s)\n",
                kv2_get_serial(dev.raw()), kv2_get_pipeline_name(dev.raw()));

    uint64_t last_seq = 0;
    for (int i = 0; i < 90; ++i) {
        kv2::ScopedFrame frame = dev.wait_frame(last_seq, 1000);
        if (!frame) continue;
        last_seq = frame->frame_seq;

        if (frame->body_count > 0) {
            const Kv2Body& b0 = frame->bodies[0];
            const Kv2Joint& head = b0.joints[KV2_JOINT_HEAD];
            std::printf("[#%05llu] %.1f FPS | infer=%.2f ms | Bodies=%u | Head=(%+.2f, %+.2f, %.2f m)\n",
                        static_cast<unsigned long long>(frame->frame_seq),
                        frame->sensor_fps, frame->skeleton_infer_ms, frame->body_count,
                        head.pos[0], head.pos[1], head.pos[2]);
        } else if (i % 30 == 0) {
            std::printf("[#%05llu] %.1f FPS | infer=%.2f ms | Bodies=0\n",
                        static_cast<unsigned long long>(frame->frame_seq),
                        frame->sensor_fps, frame->skeleton_infer_ms);
        }
    }
    return 0;
}
