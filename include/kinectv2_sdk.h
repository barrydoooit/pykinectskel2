#ifndef KINECTV2_SDK_H_
#define KINECTV2_SDK_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KV2_COLOR_WIDTH   1920
#define KV2_COLOR_HEIGHT  1080
#define KV2_DEPTH_WIDTH   512
#define KV2_DEPTH_HEIGHT  424
#define KV2_MAX_BODIES    6
#define KV2_JOINT_COUNT   25

typedef enum Kv2PipelineType {
    KV2_PIPELINE_AUTO   = 0,
    KV2_PIPELINE_CUDA   = 1,
    KV2_PIPELINE_OPENCL = 2,
    KV2_PIPELINE_CPU    = 3
} Kv2PipelineType;

typedef enum Kv2StreamFlags {
    KV2_STREAM_DEPTH            = 0x01,
    KV2_STREAM_IR               = 0x02,
    KV2_STREAM_COLOR            = 0x04,
    KV2_STREAM_BODY             = 0x08,
    KV2_STREAM_BODY_INDEX       = 0x10,
    KV2_STREAM_REGISTERED_COLOR = 0x20,
    KV2_STREAM_DEFAULT          = (0x01 | 0x02 | 0x04 | 0x08 | 0x10),
    KV2_STREAM_ALL              = (0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x20)
} Kv2StreamFlags;

typedef enum Kv2TrackingState {
    KV2_TRACKING_NOT_TRACKED = 0,
    KV2_TRACKING_INFERRED    = 1,
    KV2_TRACKING_TRACKED     = 2
} Kv2TrackingState;

typedef enum Kv2JointType {
    KV2_JOINT_SPINE_BASE     = 0,
    KV2_JOINT_SPINE_MID      = 1,
    KV2_JOINT_NECK           = 2,
    KV2_JOINT_HEAD           = 3,
    KV2_JOINT_SHOULDER_LEFT  = 4,
    KV2_JOINT_ELBOW_LEFT     = 5,
    KV2_JOINT_WRIST_LEFT     = 6,
    KV2_JOINT_HAND_LEFT      = 7,
    KV2_JOINT_SHOULDER_RIGHT = 8,
    KV2_JOINT_ELBOW_RIGHT    = 9,
    KV2_JOINT_WRIST_RIGHT    = 10,
    KV2_JOINT_HAND_RIGHT     = 11,
    KV2_JOINT_HIP_LEFT       = 12,
    KV2_JOINT_KNEE_LEFT      = 13,
    KV2_JOINT_ANKLE_LEFT     = 14,
    KV2_JOINT_FOOT_LEFT      = 15,
    KV2_JOINT_HIP_RIGHT      = 16,
    KV2_JOINT_KNEE_RIGHT     = 17,
    KV2_JOINT_ANKLE_RIGHT    = 18,
    KV2_JOINT_FOOT_RIGHT     = 19,
    KV2_JOINT_SPINE_SHOULDER = 20,
    KV2_JOINT_HAND_TIP_LEFT  = 21,
    KV2_JOINT_THUMB_LEFT     = 22,
    KV2_JOINT_HAND_TIP_RIGHT = 23,
    KV2_JOINT_THUMB_RIGHT    = 24
} Kv2JointType;

typedef struct Kv2IrIntrinsics {
    float fx, fy;
    float cx, cy;
    float k1, k2, k3;
    float p1, p2;
} Kv2IrIntrinsics;

typedef struct Kv2ColorIntrinsics {
    float fx, fy;
    float cx, cy;
} Kv2ColorIntrinsics;

typedef struct Kv2Joint {
    float    pos[3];          /* CameraSpace meters: +X right (selfie), +Y up, +Z forward */
    float    quat[4];         /* Joint orientation quaternion (x, y, z, w) */
    float    depth_uv[2];     /* Projected pixel (u, v) on 512x424 Depth/IR image */
    float    color_uv[2];     /* Projected pixel (u, v) on 1920x1080 Color image */
    uint32_t tracking_state;  /* Kv2TrackingState: 0=NotTracked, 1=Inferred, 2=Tracked */
} Kv2Joint;

typedef struct Kv2Body {
    uint64_t tracking_id;
    uint8_t  is_tracked;
    uint8_t  slot_index;
    uint16_t reserved;
    uint32_t clipped_edges;
    Kv2Joint joints[KV2_JOINT_COUNT];
} Kv2Body;

typedef struct Kv2FrameView {
    uint64_t        frame_seq;
    double          timestamp_sec;
    float           sensor_fps;
    float           skeleton_fps;
    float           skeleton_infer_ms;
    uint32_t        stream_flags;

    /* Zero-copy pointers valid until kv2_release_frame() */
    const uint8_t*  color_bgrx;          /* 1920 * 1080 * 4 bytes (BGRX), or NULL if disabled */
    const uint16_t* depth_mm;            /* 512 * 424 uint16_t millimeters, or NULL */
    const uint16_t* ir;                  /* 512 * 424 uint16_t infrared amplitude, or NULL */
    const uint8_t*  registered_bgrx;     /* 512 * 424 * 4 bytes (RGB aligned to Depth), or NULL */
    const float*    undistorted_depth_mm;/* 512 * 424 float32 undistorted depth (mm), or NULL */
    const uint8_t*  body_index;          /* 512 * 424 uint8_t (0..5 body slot, 255 background), or NULL */

    float           floor_clip_plane[4];
    float           up_vector[4];
    uint32_t        body_count;
    Kv2Body         bodies[KV2_MAX_BODIES];

    /* Internal pin token — do not modify */
    int32_t         _slot_id;
    int32_t         _reserved;
} Kv2FrameView;

typedef struct Kv2Config {
    const char* serial;         /* NULL = open first available Kinect v2 */
    int32_t     pipeline;       /* Kv2PipelineType (default: KV2_PIPELINE_AUTO) */
    uint32_t    stream_flags;   /* Bitmask of Kv2StreamFlags (default: KV2_STREAM_DEFAULT) */
    const char* worker_exe;     /* NULL = auto-locate dist/kinectv2_linux_bundle/vvtechs_worker.exe */
    const char* dll_dir;        /* NULL = same directory as worker_exe */
    const char* wine_binary;    /* NULL = auto-locate wine64 */
    const char* wineprefix;     /* NULL = default $HOME/.wine-kinectv2 */
    int32_t     mirror_skeleton;/* 0 = native selfie alignment (recommended), 1 = horizontal flip */
} Kv2Config;

typedef struct Kv2Device Kv2Device;

void       kv2_default_config(Kv2Config* out_cfg);
Kv2Device* kv2_open(const Kv2Config* cfg, char* err_buf, size_t err_cap);
void       kv2_close(Kv2Device* dev);

int        kv2_start(Kv2Device* dev, char* err_buf, size_t err_cap);
void       kv2_stop(Kv2Device* dev);

/* Blocks up to timeout_ms for a newer frame than last_seq; returns 1 on success, 0 on timeout, <0 on error. */
int        kv2_wait_frame(Kv2Device* dev, uint64_t last_seq, int32_t timeout_ms, Kv2FrameView* out_frame);
/* Non-blocking grab of the most recent completed frame; returns 1 if available, 0 if none yet. */
int        kv2_get_latest_frame(Kv2Device* dev, Kv2FrameView* out_frame);
void       kv2_release_frame(Kv2Device* dev, Kv2FrameView* frame);

/* Reset vvtechs.dll temporal skeleton & 15-frame background state. */
int        kv2_reset_skeleton(Kv2Device* dev);
void       kv2_set_registered_color_enabled(Kv2Device* dev, int enabled);
void       kv2_get_memory_rss_mb(const Kv2Device* dev, float* out_host_rss_mb, float* out_worker_rss_mb);

/* Camera intrinsics and coordinate projection utilities (thread-safe after kv2_open). */
const char* kv2_get_serial(const Kv2Device* dev);
const char* kv2_get_pipeline_name(const Kv2Device* dev);
void        kv2_get_intrinsics(const Kv2Device* dev, Kv2IrIntrinsics* out_ir, Kv2ColorIntrinsics* out_color);

/* Project 3D CameraSpace point (meters, +X right in selfie, +Y up, +Z forward) onto 512x424 Depth & 1920x1080 Color. */
void        kv2_project_camera_to_depth(const Kv2Device* dev, const float pos_xyz[3], float out_uv[2]);
void        kv2_project_camera_to_color(const Kv2Device* dev, const float pos_xyz[3], float out_uv[2]);
/* Map a 512x424 Depth pixel (dx, dy) with depth_mm > 0 onto 1920x1080 Color pixel (cx, cy). */
int         kv2_map_depth_to_color(const Kv2Device* dev, int dx, int dy, float depth_mm, float out_color_uv[2]);
/* Back-project a 512x424 Depth pixel (dx, dy, depth_mm) into 3D CameraSpace meters (X, Y, Z). */
int         kv2_deproject_depth_to_camera(const Kv2Device* dev, int dx, int dy, float depth_mm, float out_xyz[3]);

const char* kv2_joint_name(int joint_index);

#ifdef __cplusplus
} /* extern "C" */

#include <stdexcept>
#include <string>

namespace kv2 {

class ScopedFrame {
public:
    ScopedFrame() : dev_(nullptr), valid_(false) { view_._slot_id = -1; }
    ScopedFrame(Kv2Device* dev, const Kv2FrameView& v) : dev_(dev), view_(v), valid_(true) {}
    ~ScopedFrame() { release(); }

    ScopedFrame(const ScopedFrame&) = delete;
    ScopedFrame& operator=(const ScopedFrame&) = delete;

    ScopedFrame(ScopedFrame&& other) noexcept : dev_(other.dev_), view_(other.view_), valid_(other.valid_) {
        other.dev_ = nullptr;
        other.valid_ = false;
        other.view_._slot_id = -1;
    }
    ScopedFrame& operator=(ScopedFrame&& other) noexcept {
        if (this != &other) {
            release();
            dev_ = other.dev_;
            view_ = other.view_;
            valid_ = other.valid_;
            other.dev_ = nullptr;
            other.valid_ = false;
            other.view_._slot_id = -1;
        }
        return *this;
    }

    void release() {
        if (valid_ && dev_ != nullptr) {
            kv2_release_frame(dev_, &view_);
        }
        dev_ = nullptr;
        valid_ = false;
        view_._slot_id = -1;
    }

    explicit operator bool() const { return valid_; }
    const Kv2FrameView& get() const { return view_; }
    const Kv2FrameView* operator->() const { return &view_; }

private:
    Kv2Device*   dev_;
    Kv2FrameView view_{};
    bool         valid_;
};

class Device {
public:
    explicit Device(const Kv2Config* cfg = nullptr) {
        char err[512] = {0};
        dev_ = kv2_open(cfg, err, sizeof(err));
        if (!dev_) {
            throw std::runtime_error(std::string("kv2_open failed: ") + err);
        }
    }
    ~Device() {
        if (dev_) {
            kv2_close(dev_);
            dev_ = nullptr;
        }
    }
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    void start() {
        char err[512] = {0};
        if (kv2_start(dev_, err, sizeof(err)) != 0) {
            throw std::runtime_error(std::string("kv2_start failed: ") + err);
        }
    }
    void stop() { kv2_stop(dev_); }
    void reset_skeleton() { kv2_reset_skeleton(dev_); }

    ScopedFrame wait_frame(uint64_t last_seq = 0, int32_t timeout_ms = 1000) {
        Kv2FrameView v{};
        if (kv2_wait_frame(dev_, last_seq, timeout_ms, &v) == 1) {
            return ScopedFrame(dev_, v);
        }
        return ScopedFrame();
    }

    Kv2Device* raw() const { return dev_; }

private:
    Kv2Device* dev_{nullptr};
};

} /* namespace kv2 */
#endif /* __cplusplus */

#endif /* KINECTV2_SDK_H_ */
