#include "kinectv2_sdk.h"

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/led_settings.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t KV2_REQ_MAGIC  = 0x4B563242u;
constexpr uint32_t KV2_RESP_MAGIC = 0x4B563252u;

constexpr uint32_t KV2_CMD_RESET          = 1u;
constexpr uint32_t KV2_CMD_SET_INTRINSICS = 2u;
constexpr uint32_t KV2_CMD_PROCESS_FRAME  = 3u;
constexpr uint32_t KV2_CMD_QUIT           = 9u;

constexpr uint32_t KV2_FLAG_WANT_BODY_INDEX = 0x1u;
constexpr uint32_t KV2_FLAG_HAS_IR_FRAME    = 0x2u;

constexpr size_t kColorBytes = (size_t)KV2_COLOR_WIDTH * KV2_COLOR_HEIGHT * 4u;
constexpr size_t kDepthCount = (size_t)KV2_DEPTH_WIDTH * KV2_DEPTH_HEIGHT;
constexpr size_t kRegBytes   = kDepthCount * 4u;
constexpr int    kNumSlots   = 4;

float read_pid_rss_mb(pid_t pid) {
    char path[64];
    if (pid <= 0) {
        snprintf(path, sizeof(path), "/proc/self/statm");
    } else {
        snprintf(path, sizeof(path), "/proc/%d/statm", (int)pid);
    }
    FILE* fp = fopen(path, "r");
    if (!fp) return 0.0f;
    unsigned long vmsize = 0, rss_pages = 0;
    int got = fscanf(fp, "%lu %lu", &vmsize, &rss_pages);
    fclose(fp);
    if (got != 2) return 0.0f;
    long page_sz = sysconf(_SC_PAGESIZE);
    return (float)((double)rss_pages * (double)(page_sz > 0 ? page_sz : 4096) / (1024.0 * 1024.0));
}

#pragma pack(push, 1)
struct WireHeader {
    uint32_t magic;
    uint32_t code;
    uint32_t payload_len;
    uint32_t flags;
};

struct WireIntrinsics {
    float fx, fy, cx, cy, k1, k2, k3;
};

struct WireFramePrefix {
    uint32_t frame_index;
    float    gravity[4];
};
#pragma pack(pop)

const char* kJointNames[KV2_JOINT_COUNT] = {
    "SpineBase", "SpineMid", "Neck", "Head",
    "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft",
    "ShoulderRight", "ElbowRight", "WristRight", "HandRight",
    "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
    "HipRight", "KneeRight", "AnkleRight", "FootRight",
    "SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight"
};

std::once_flag g_sigpipe_once;

void ignore_sigpipe_once() {
    std::call_once(g_sigpipe_once, []() {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = SIG_IGN;
        sigaction(SIGPIPE, &sa, nullptr);
    });
}

void set_err(char* buf, size_t cap, const std::string& msg) {
    if (buf && cap > 0) {
        snprintf(buf, cap, "%s", msg.c_str());
    }
}

bool file_exists(const std::string& p) {
    struct stat st;
    return !p.empty() && stat(p.c_str(), &st) == 0;
}

std::string dirname_of(const std::string& p) {
    size_t pos = p.find_last_of('/');
    return (pos == std::string::npos) ? "." : p.substr(0, pos);
}

std::string shared_lib_dir() {
    Dl_info info{};
    if (dladdr((void*)&kv2_default_config, &info) != 0 && info.dli_fname) {
        char real_buf[4096];
        if (realpath(info.dli_fname, real_buf)) {
            return dirname_of(real_buf);
        }
        return dirname_of(info.dli_fname);
    }
    return ".";
}

std::string find_user_x11_display() {
    uid_t uid = getuid();
    glob_t g{};
    std::string chosen;
    if (glob("/tmp/.X11-unix/X*", 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) {
            struct stat st{};
            if (stat(g.gl_pathv[i], &st) == 0 && st.st_uid == uid) {
                const char* base = strrchr(g.gl_pathv[i], 'X');
                if (base && base[1] >= '0' && base[1] <= '9') {
                    chosen = std::string(":") + (base + 1);
                    break;
                }
            }
        }
        globfree(&g);
    }
    return chosen.empty() ? ":0" : chosen;
}

bool write_exact_fd(int fd, const void* buf, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    size_t rem = len;
    while (rem > 0) {
        ssize_t n = ::write(fd, p, rem);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        p += (size_t)n;
        rem -= (size_t)n;
    }
    return true;
}

bool read_exact_fd(int fd, void* buf, size_t len) {
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t rem = len;
    while (rem > 0) {
        ssize_t n = ::read(fd, p, rem);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        p += (size_t)n;
        rem -= (size_t)n;
    }
    return true;
}

class FpsMeter {
public:
    float tick(double now_sec) {
        stamps_.push_back(now_sec);
        while (stamps_.size() > 31) {
            stamps_.pop_front();
        }
        if (stamps_.size() < 2) return 0.0f;
        double dt = stamps_.back() - stamps_.front();
        return (dt > 1e-6) ? (float)((stamps_.size() - 1) / dt) : 0.0f;
    }
    void clear() { stamps_.clear(); }
private:
    std::deque<double> stamps_;
};

struct FrameSlot {
    std::vector<uint8_t>  color_bgrx;
    std::vector<uint16_t> depth_mm;
    std::vector<uint16_t> ir;
    std::vector<uint8_t>  registered_bgrx;
    std::vector<float>    undistorted_mm;
    std::vector<uint8_t>  body_index;

    uint64_t frame_seq = 0;
    double   timestamp_sec = 0.0;
    float    sensor_fps = 0.0f;
    float    skeleton_fps = 0.0f;
    float    skeleton_infer_ms = 0.0f;
    uint32_t stream_flags = 0;

    float    floor_clip_plane[4]{};
    float    up_vector[4]{};
    uint32_t body_count = 0;
    Kv2Body  bodies[KV2_MAX_BODIES]{};

    int  readers = 0;
    bool busy_capture = false;
    bool busy_skel = false;
};

} // namespace

struct Kv2Device {
    Kv2Config cfg{};
    std::string serial_str;
    std::string pipeline_name;
    std::string worker_exe_path;
    std::string dll_dir_path;
    std::string wine_bin_path;
    std::string wineprefix_path;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::PacketPipeline* pipeline = nullptr; // owned by f2_dev after openDevice
    libfreenect2::Freenect2Device* f2_dev = nullptr;
    std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener;
    std::unique_ptr<libfreenect2::Registration> registration;

    libfreenect2::Freenect2Device::IrCameraParams ir_params{};
    libfreenect2::Freenect2Device::ColorCameraParams color_params{};
    bool has_calib = false;

    /* Child Wine64 vvtechs_worker process */
    pid_t worker_pid = -1;
    int   worker_stdin_fd = -1;
    int   worker_stdout_fd = -1;
    std::mutex worker_io_mtx;
    uint32_t worker_frame_counter = 0;

    /* Pre-allocated zero-copy frame pool */
    FrameSlot slots[kNumSlots];
    std::mutex pool_mtx;
    std::condition_variable skel_cv;
    std::condition_variable consumer_cv;
    int pending_skel_slot = -1;
    int latest_published_slot = -1;

    std::atomic<bool> running{false};
    std::atomic<bool> reset_requested{false};
    std::atomic<bool> reg_enabled_atomic{true};
    std::thread capture_thread;
    std::thread skel_thread;

    void set_device_leds(uint16_t level_0_to_1000) {
        if (!f2_dev) return;
        uint16_t lv = (level_0_to_1000 > 1000u) ? 1000u : level_0_to_1000;
        libfreenect2::LedSettings led0{0u, 0u, 0u, 0u, 0u, 0u};
        libfreenect2::LedSettings led1{1u, 0u, lv, lv, 0u, 0u};
        f2_dev->setLedStatus(led0);
        f2_dev->setLedStatus(led1);
    }

    void project_to_depth(const float xyz[3], float out_uv[2]) const {
        if (!has_calib || xyz[2] <= 1e-4f) {
            out_uv[0] = -1e6f;
            out_uv[1] = -1e6f;
            return;
        }
        float sign_x = cfg.mirror_skeleton ? -1.0f : 1.0f;
        float nx = (sign_x * xyz[0]) / xyz[2];
        float ny = (-xyz[1]) / xyz[2];
        float r2 = nx * nx + ny * ny;
        float dxdy2 = 2.0f * nx * ny;
        float kr = 1.0f + ((ir_params.k3 * r2 + ir_params.k2) * r2 + ir_params.k1) * r2;
        out_uv[0] = ir_params.fx * (nx * kr + ir_params.p2 * (r2 + 2.0f * nx * nx) + ir_params.p1 * dxdy2) + ir_params.cx;
        out_uv[1] = ir_params.fy * (ny * kr + ir_params.p1 * (r2 + 2.0f * ny * ny) + ir_params.p2 * dxdy2) + ir_params.cy;
    }

    void project_to_color(const float xyz[3], float out_uv[2]) const {
        if (!has_calib || xyz[2] <= 1e-4f) {
            out_uv[0] = -1e6f;
            out_uv[1] = -1e6f;
            return;
        }
        constexpr float depth_q = 0.01f;
        constexpr float color_q = 0.002199f;
        float sign_x = cfg.mirror_skeleton ? -1.0f : 1.0f;
        float nx = (sign_x * xyz[0]) / xyz[2];
        float ny = (-xyz[1]) / xyz[2];
        float ux = nx * ir_params.fx + ir_params.cx;
        float uy = ny * ir_params.fy + ir_params.cy;

        float mx = (ux - ir_params.cx) * depth_q;
        float my = (uy - ir_params.cy) * depth_q;

        float wx =
            (mx * mx * mx * color_params.mx_x3y0) + (my * my * my * color_params.mx_x0y3) +
            (mx * mx * my * color_params.mx_x2y1) + (my * my * mx * color_params.mx_x1y2) +
            (mx * mx * color_params.mx_x2y0) + (my * my * color_params.mx_x0y2) + (mx * my * color_params.mx_x1y1) +
            (mx * color_params.mx_x1y0) + (my * color_params.mx_x0y1) + (color_params.mx_x0y0);

        float wy =
            (mx * mx * mx * color_params.my_x3y0) + (my * my * my * color_params.my_x0y3) +
            (mx * mx * my * color_params.my_x2y1) + (my * my * mx * color_params.my_x1y2) +
            (mx * mx * color_params.my_x2y0) + (my * my * color_params.my_x0y2) + (mx * my * color_params.my_x1y1) +
            (mx * color_params.my_x1y0) + (my * color_params.my_x0y1) + (color_params.my_x0y0);

        float dz_mm = xyz[2] * 1000.0f;
        float rx = (wx / (color_params.fx * color_q)) - (color_params.shift_m / color_params.shift_d) + (color_params.shift_m / dz_mm);
        out_uv[0] = rx * color_params.fx + color_params.cx;
        out_uv[1] = (wy / color_q) + color_params.cy;
    }

    bool start_worker(std::string& err) {
        int in_pipe[2] = {-1, -1};
        int out_pipe[2] = {-1, -1};
        if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0) {
            err = "pipe() failed for vvtechs_worker";
            return false;
        }
        // Increase pipe buffer size to 1 MB on Linux to avoid context-switch stalls on 868 KB writes
        fcntl(in_pipe[1], F_SETPIPE_SZ, 1048576);
        fcntl(out_pipe[0], F_SETPIPE_SZ, 1048576);

        std::string child_display;
        if (!getenv("DISPLAY") && !getenv("WAYLAND_DISPLAY")) {
            child_display = find_user_x11_display();
        }

        pid_t pid = fork();
        if (pid < 0) {
            close(in_pipe[0]); close(in_pipe[1]);
            close(out_pipe[0]); close(out_pipe[1]);
            err = "fork() failed for vvtechs_worker";
            return false;
        }
        if (pid == 0) {
            prctl(PR_SET_PDEATHSIG, SIGTERM);
            dup2(in_pipe[0], STDIN_FILENO);
            dup2(out_pipe[1], STDOUT_FILENO);
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            close(in_pipe[0]); close(in_pipe[1]);
            close(out_pipe[0]); close(out_pipe[1]);
            for (int fd = 3; fd < 1024; ++fd) {
                close(fd);
            }

            if (!child_display.empty()) {
                setenv("DISPLAY", child_display.c_str(), 1);
            }
            setenv("WINEDEBUG", "-all", 0);
            setenv("DXVK_LOG_LEVEL", "none", 0);
            setenv("DXVK_FILTER_DEVICE_NAME", "NVIDIA", 0);
            setenv("WINEDLLOVERRIDES", "wineusb.sys,winebus.sys=d;d3d11,dxgi,msvcr110,msvcp110,k4wcll=n,b", 1);
            if (!wineprefix_path.empty()) {
                setenv("WINEPREFIX", wineprefix_path.c_str(), 1);
            }

            std::vector<const char*> argv;
            argv.push_back(wine_bin_path.c_str());
            argv.push_back(worker_exe_path.c_str());
            argv.push_back("--stdio");
            if (!dll_dir_path.empty()) {
                argv.push_back("--dll-dir");
                argv.push_back(dll_dir_path.c_str());
            }
            argv.push_back(nullptr);
            execvp(wine_bin_path.c_str(), const_cast<char* const*>(argv.data()));
            _exit(127);
        }

        close(in_pipe[0]);
        close(out_pipe[1]);
        worker_pid = pid;
        worker_stdin_fd = in_pipe[1];
        worker_stdout_fd = out_pipe[0];
        worker_frame_counter = 0;

        // Synchronous handshake: RESET + SET_INTRINSICS
        if (!send_simple_cmd(KV2_CMD_RESET, nullptr, 0)) {
            err = "vvtechs_worker failed initial RESET handshake (check Wine64 + DXVK setup)";
            stop_worker();
            return false;
        }
        if (has_calib) {
            WireIntrinsics wi{};
            wi.fx = ir_params.fx;
            wi.fy = ir_params.fy;
            wi.cx = cfg.mirror_skeleton ? (511.0f - ir_params.cx) : ir_params.cx;
            wi.cy = ir_params.cy;
            wi.k1 = ir_params.k1;
            wi.k2 = ir_params.k2;
            wi.k3 = ir_params.k3;
            if (!send_simple_cmd(KV2_CMD_SET_INTRINSICS, &wi, sizeof(wi))) {
                err = "vvtechs_worker failed SET_INTRINSICS";
                stop_worker();
                return false;
            }
        }
        return true;
    }

    bool send_simple_cmd(uint32_t code, const void* payload, uint32_t len) {
        std::lock_guard<std::mutex> lk(worker_io_mtx);
        if (worker_stdin_fd < 0 || worker_stdout_fd < 0) return false;
        WireHeader req{KV2_REQ_MAGIC, code, len, 0u};
        if (!write_exact_fd(worker_stdin_fd, &req, sizeof(req))) return false;
        if (len > 0 && !write_exact_fd(worker_stdin_fd, payload, len)) return false;
        WireHeader resp{};
        if (!read_exact_fd(worker_stdout_fd, &resp, sizeof(resp))) return false;
        if (resp.magic != KV2_RESP_MAGIC) return false;
        if (resp.payload_len > 0) {
            std::vector<uint8_t> tmp(resp.payload_len);
            if (!read_exact_fd(worker_stdout_fd, tmp.data(), resp.payload_len)) return false;
        }
        return true;
    }

    void stop_worker() {
        if (worker_pid > 0) {
            send_simple_cmd(KV2_CMD_QUIT, nullptr, 0);
        }
        if (worker_stdin_fd >= 0) { close(worker_stdin_fd); worker_stdin_fd = -1; }
        if (worker_stdout_fd >= 0) { close(worker_stdout_fd); worker_stdout_fd = -1; }
        if (worker_pid > 0) {
            int status = 0;
            for (int i = 0; i < 20; ++i) {
                if (waitpid(worker_pid, &status, WNOHANG) == worker_pid) {
                    worker_pid = -1;
                    break;
                }
                usleep(50000);
            }
            if (worker_pid > 0) {
                kill(worker_pid, SIGKILL);
                waitpid(worker_pid, &status, 0);
                worker_pid = -1;
            }
        }
    }

    int acquire_free_slot_locked() {
        for (int i = 0; i < kNumSlots; ++i) {
            if (i == latest_published_slot || i == pending_skel_slot) continue;
            if (slots[i].readers == 0 && !slots[i].busy_capture && !slots[i].busy_skel) {
                slots[i].busy_capture = true;
                return i;
            }
        }
        return -1;
    }

    void capture_loop() {
        libfreenect2::FrameMap frames;
        libfreenect2::Frame undistorted(512, 424, 4);
        libfreenect2::Frame registered(512, 424, 4);
        FpsMeter fps_meter;
        uint64_t seq = 0;

        const bool want_color = (cfg.stream_flags & KV2_STREAM_COLOR) != 0;
        const bool want_reg   = (cfg.stream_flags & KV2_STREAM_REGISTERED_COLOR) != 0;
        const bool want_body  = (cfg.stream_flags & KV2_STREAM_BODY) != 0;
        const bool mirror     = cfg.mirror_skeleton != 0;

        while (running.load(std::memory_order_relaxed)) {
            if (!listener->waitForNewFrame(frames, 1000)) {
                continue;
            }
            auto t_now = std::chrono::steady_clock::now();
            double ts_sec = std::chrono::duration<double>(t_now.time_since_epoch()).count();
            float s_fps = fps_meter.tick(ts_sec);

            int slot_idx = -1;
            {
                std::lock_guard<std::mutex> lk(pool_mtx);
                slot_idx = acquire_free_slot_locked();
            }
            if (slot_idx < 0) {
                listener->release(frames);
                continue;
            }

            FrameSlot& slot = slots[slot_idx];
            slot.frame_seq = ++seq;
            slot.timestamp_sec = ts_sec;
            slot.sensor_fps = s_fps;
            slot.stream_flags = cfg.stream_flags;

            libfreenect2::Frame* f_depth = frames[libfreenect2::Frame::Depth];
            libfreenect2::Frame* f_ir    = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame* f_color = want_color || want_reg ? frames[libfreenect2::Frame::Color] : nullptr;

            if (f_depth && f_depth->data) {
                const float* src = reinterpret_cast<const float*>(f_depth->data);
                uint16_t* dst = slot.depth_mm.data();
                if (!mirror) {
                    for (size_t i = 0; i < kDepthCount; ++i) {
                        float v = src[i];
                        dst[i] = (v > 0.0f && v < 65535.0f) ? (uint16_t)(v + 0.5f) : 0u;
                    }
                } else {
                    for (int r = 0; r < KV2_DEPTH_HEIGHT; ++r) {
                        const float* srow = src + r * KV2_DEPTH_WIDTH;
                        uint16_t* drow = dst + r * KV2_DEPTH_WIDTH;
                        for (int c = 0; c < KV2_DEPTH_WIDTH; ++c) {
                            float v = srow[KV2_DEPTH_WIDTH - 1 - c];
                            drow[c] = (v > 0.0f && v < 65535.0f) ? (uint16_t)(v + 0.5f) : 0u;
                        }
                    }
                }
            }

            if (f_ir && f_ir->data) {
                const float* src = reinterpret_cast<const float*>(f_ir->data);
                uint16_t* dst = slot.ir.data();
                if (!mirror) {
                    for (size_t i = 0; i < kDepthCount; ++i) {
                        float v = src[i];
                        dst[i] = (v > 0.0f) ? ((v >= 65535.0f) ? 65535u : (uint16_t)(v + 0.5f)) : 0u;
                    }
                } else {
                    for (int r = 0; r < KV2_DEPTH_HEIGHT; ++r) {
                        const float* srow = src + r * KV2_DEPTH_WIDTH;
                        uint16_t* drow = dst + r * KV2_DEPTH_WIDTH;
                        for (int c = 0; c < KV2_DEPTH_WIDTH; ++c) {
                            float v = srow[KV2_DEPTH_WIDTH - 1 - c];
                            drow[c] = (v > 0.0f) ? ((v >= 65535.0f) ? 65535u : (uint16_t)(v + 0.5f)) : 0u;
                        }
                    }
                }
            }

            if (want_color && f_color && f_color->data) {
                memcpy(slot.color_bgrx.data(), f_color->data, kColorBytes);
            }

            if (want_reg && reg_enabled_atomic.load(std::memory_order_relaxed) && registration && f_color && f_depth) {
                registration->apply(f_color, f_depth, &undistorted, &registered, true);
                memcpy(slot.registered_bgrx.data(), registered.data, kRegBytes);
                memcpy(slot.undistorted_mm.data(), undistorted.data, kDepthCount * sizeof(float));
            }

            listener->release(frames);

            {
                std::lock_guard<std::mutex> lk(pool_mtx);
                slot.busy_capture = false;
                if (want_body) {
                    pending_skel_slot = slot_idx;
                    skel_cv.notify_one();
                } else {
                    slot.skeleton_fps = 0.0f;
                    slot.skeleton_infer_ms = 0.0f;
                    slot.body_count = 0;
                    latest_published_slot = slot_idx;
                    consumer_cv.notify_all();
                }
            }
        }
    }

    void skel_loop() {
        FpsMeter skel_fps_meter;
        const bool want_bidx = (cfg.stream_flags & KV2_STREAM_BODY_INDEX) != 0;
        const size_t resp_body_bytes = 0x1890u + (want_bidx ? kDepthCount : 0u);
        std::vector<uint8_t> raw_resp(resp_body_bytes);

        while (running.load(std::memory_order_relaxed)) {
            int slot_idx = -1;
            {
                std::unique_lock<std::mutex> lk(pool_mtx);
                skel_cv.wait_for(lk, std::chrono::milliseconds(100), [&]() {
                    return !running.load(std::memory_order_relaxed) || pending_skel_slot >= 0;
                });
                if (!running.load(std::memory_order_relaxed)) break;
                if (pending_skel_slot < 0) continue;
                slot_idx = pending_skel_slot;
                pending_skel_slot = -1;
                slots[slot_idx].busy_skel = true;
            }

            if (reset_requested.exchange(false)) {
                send_simple_cmd(KV2_CMD_RESET, nullptr, 0);
            }

            FrameSlot& slot = slots[slot_idx];
            auto t0 = std::chrono::steady_clock::now();

            bool ok = false;
            {
                std::lock_guard<std::mutex> iolk(worker_io_mtx);
                if (worker_stdin_fd >= 0 && worker_stdout_fd >= 0) {
                    uint32_t fidx = ++worker_frame_counter;
                    uint32_t flags = KV2_FLAG_HAS_IR_FRAME | (want_bidx ? KV2_FLAG_WANT_BODY_INDEX : 0u);
                    uint32_t plen = (uint32_t)(sizeof(WireFramePrefix) + kDepthCount * 2u + kDepthCount * 2u);

                    WireHeader hdr{KV2_REQ_MAGIC, KV2_CMD_PROCESS_FRAME, plen, flags};
                    WireFramePrefix pfx{fidx, {0.0f, -1.0f, 0.0f, 0.0f}};

                    struct iovec iov[4];
                    iov[0].iov_base = &hdr;                 iov[0].iov_len = sizeof(hdr);
                    iov[1].iov_base = &pfx;                 iov[1].iov_len = sizeof(pfx);
                    iov[2].iov_base = slot.depth_mm.data(); iov[2].iov_len = kDepthCount * sizeof(uint16_t);
                    iov[3].iov_base = slot.ir.data();       iov[3].iov_len = kDepthCount * sizeof(uint16_t);

                    size_t total_to_write = sizeof(hdr) + plen;
                    ssize_t w = ::writev(worker_stdin_fd, iov, 4);
                    bool write_ok = false;
                    if (w == (ssize_t)total_to_write) {
                        write_ok = true;
                    } else if (w >= 0) {
                        std::vector<uint8_t> flat(total_to_write);
                        size_t off = 0;
                        for (int k = 0; k < 4; ++k) {
                            memcpy(flat.data() + off, iov[k].iov_base, iov[k].iov_len);
                            off += iov[k].iov_len;
                        }
                        write_ok = write_exact_fd(worker_stdin_fd, flat.data() + w, total_to_write - (size_t)w);
                    }

                    WireHeader rhdr{};
                    if (write_ok &&
                        read_exact_fd(worker_stdout_fd, &rhdr, sizeof(rhdr)) &&
                        rhdr.magic == KV2_RESP_MAGIC &&
                        rhdr.payload_len == resp_body_bytes &&
                        read_exact_fd(worker_stdout_fd, raw_resp.data(), resp_body_bytes)) {
                        ok = true;
                    }
                }
            }

            auto t1 = std::chrono::steady_clock::now();
            float infer_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            double now_sec = std::chrono::duration<double>(t1.time_since_epoch()).count();
            slot.skeleton_infer_ms = infer_ms;
            slot.skeleton_fps = ok ? skel_fps_meter.tick(now_sec) : 0.0f;

            slot.body_count = 0;
            if (ok) {
                memcpy(slot.floor_clip_plane, raw_resp.data() + 0x00, 16);
                memcpy(slot.up_vector, raw_resp.data() + 0x10, 16);
                for (uint8_t s = 0; s < KV2_MAX_BODIES; ++s) {
                    const uint8_t* bptr = raw_resp.data() + 0x20 + (size_t)s * 0x410u;
                    uint32_t is_tracked = 0;
                    memcpy(&is_tracked, bptr + 0x3F8u, 4);
                    if (is_tracked == 0) continue;

                    Kv2Body& dst_b = slot.bodies[slot.body_count++];
                    dst_b.slot_index = s;
                    dst_b.is_tracked = 1;
                    dst_b.reserved = 0;
                    memcpy(&dst_b.tracking_id, bptr + 0x3F0u, 8);
                    memcpy(&dst_b.clipped_edges, bptr + 0x40Cu, 4);

                    for (int j = 0; j < KV2_JOINT_COUNT; ++j) {
                        Kv2Joint& dj = dst_b.joints[j];
                        const float* p4 = reinterpret_cast<const float*>(bptr + 0x000u + (size_t)j * 16u);
                        const float* q4 = reinterpret_cast<const float*>(bptr + 0x190u + (size_t)j * 16u);
                        const uint32_t* st_ptr = reinterpret_cast<const uint32_t*>(bptr + 0x320u + (size_t)j * 4u);

                        dj.pos[0] = p4[0];
                        dj.pos[1] = p4[1];
                        dj.pos[2] = p4[2];
                        if (cfg.mirror_skeleton) {
                            dj.pos[0] = -dj.pos[0];
                        }
                        dj.quat[0] = q4[0];
                        dj.quat[1] = q4[1];
                        dj.quat[2] = q4[2];
                        dj.quat[3] = q4[3];
                        dj.tracking_state = *st_ptr;

                        if (dj.tracking_state != KV2_TRACKING_NOT_TRACKED && dj.pos[2] > 0.05f) {
                            project_to_depth(dj.pos, dj.depth_uv);
                            project_to_color(dj.pos, dj.color_uv);
                        } else {
                            dj.depth_uv[0] = dj.depth_uv[1] = -1e6f;
                            dj.color_uv[0] = dj.color_uv[1] = -1e6f;
                        }
                    }
                }
                if (want_bidx) {
                    memcpy(slot.body_index.data(), raw_resp.data() + 0x1890u, kDepthCount);
                }
            }

            {
                std::lock_guard<std::mutex> lk(pool_mtx);
                slot.busy_skel = false;
                latest_published_slot = slot_idx;
                consumer_cv.notify_all();
            }
        }
    }

    void fill_view_locked(int idx, Kv2FrameView* out) {
        FrameSlot& s = slots[idx];
        s.readers++;
        memset(out, 0, sizeof(*out));
        out->frame_seq = s.frame_seq;
        out->timestamp_sec = s.timestamp_sec;
        out->sensor_fps = s.sensor_fps;
        out->skeleton_fps = s.skeleton_fps;
        out->skeleton_infer_ms = s.skeleton_infer_ms;
        out->stream_flags = s.stream_flags;

        out->color_bgrx = (s.stream_flags & KV2_STREAM_COLOR) ? s.color_bgrx.data() : nullptr;
        out->depth_mm = (s.stream_flags & KV2_STREAM_DEPTH) ? s.depth_mm.data() : nullptr;
        out->ir = (s.stream_flags & KV2_STREAM_IR) ? s.ir.data() : nullptr;
        out->registered_bgrx = (s.stream_flags & KV2_STREAM_REGISTERED_COLOR) ? s.registered_bgrx.data() : nullptr;
        out->undistorted_depth_mm = (s.stream_flags & KV2_STREAM_REGISTERED_COLOR) ? s.undistorted_mm.data() : nullptr;
        out->body_index = (s.stream_flags & KV2_STREAM_BODY_INDEX) ? s.body_index.data() : nullptr;

        memcpy(out->floor_clip_plane, s.floor_clip_plane, 16);
        memcpy(out->up_vector, s.up_vector, 16);
        out->body_count = s.body_count;
        for (uint32_t i = 0; i < s.body_count && i < KV2_MAX_BODIES; ++i) {
            out->bodies[i] = s.bodies[i];
        }
        out->_slot_id = idx;
    }
};

extern "C" {

void kv2_default_config(Kv2Config* out_cfg) {
    if (!out_cfg) return;
    memset(out_cfg, 0, sizeof(*out_cfg));
    out_cfg->pipeline = KV2_PIPELINE_AUTO;
    out_cfg->stream_flags = KV2_STREAM_DEFAULT;
    out_cfg->mirror_skeleton = 0;
}

Kv2Device* kv2_open(const Kv2Config* user_cfg, char* err_buf, size_t err_cap) {
    ignore_sigpipe_once();
    // Keep kernel usbfs URB pool (~6.8 MB) safely below Ubuntu's default 16 MB usbfs_memory_mb limit
    setenv("LIBFREENECT2_IR_TRANSFERS", "24", 0);
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

    auto dev = std::make_unique<Kv2Device>();
    if (user_cfg) {
        dev->cfg = *user_cfg;
        if (dev->cfg.stream_flags == 0) dev->cfg.stream_flags = KV2_STREAM_DEFAULT;
    } else {
        kv2_default_config(&dev->cfg);
    }

    // Resolve paths for Wine64 + vvtechs_worker.exe
    const char* home = getenv("HOME");
    std::string home_dir = home ? home : "/root";
    std::string lib_dir = shared_lib_dir();
    std::string repo_guess = dirname_of(lib_dir);

    if (dev->cfg.worker_exe && dev->cfg.worker_exe[0]) {
        dev->worker_exe_path = dev->cfg.worker_exe;
    } else if (getenv("KV2_WORKER_EXE")) {
        dev->worker_exe_path = getenv("KV2_WORKER_EXE");
    } else {
        std::vector<std::string> candidates = {
            repo_guess + "/dist/kinectv2_linux_bundle/vvtechs_worker.exe",
            lib_dir + "/../dist/kinectv2_linux_bundle/vvtechs_worker.exe",
            "./dist/kinectv2_linux_bundle/vvtechs_worker.exe"
        };
        for (const auto& c : candidates) {
            if (file_exists(c)) {
                char rb[4096];
                dev->worker_exe_path = realpath(c.c_str(), rb) ? rb : c;
                break;
            }
        }
    }

    if (dev->cfg.dll_dir && dev->cfg.dll_dir[0]) {
        dev->dll_dir_path = dev->cfg.dll_dir;
    } else if (!dev->worker_exe_path.empty()) {
        dev->dll_dir_path = dirname_of(dev->worker_exe_path);
    }

    if (dev->cfg.wine_binary && dev->cfg.wine_binary[0]) {
        dev->wine_bin_path = dev->cfg.wine_binary;
    } else if (file_exists(home_dir + "/.local/bin/wine64")) {
        dev->wine_bin_path = home_dir + "/.local/bin/wine64";
    } else {
        dev->wine_bin_path = "wine64";
    }

    if (dev->cfg.wineprefix && dev->cfg.wineprefix[0]) {
        dev->wineprefix_path = dev->cfg.wineprefix;
    } else if (file_exists(home_dir + "/.wine-kinectv2")) {
        dev->wineprefix_path = home_dir + "/.wine-kinectv2";
    }

    if ((dev->cfg.stream_flags & KV2_STREAM_BODY) && !file_exists(dev->worker_exe_path)) {
        set_err(err_buf, err_cap, "vvtechs_worker.exe not found (pass Kv2Config.worker_exe or set KV2_WORKER_EXE)");
        return nullptr;
    }

    auto make_pipeline = [&]() -> libfreenect2::PacketPipeline* {
        auto try_pipe = [&](int mode) -> libfreenect2::PacketPipeline* {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            if (mode == KV2_PIPELINE_CUDA) {
                dev->pipeline_name = "CUDA";
                return new libfreenect2::CudaPacketPipeline();
            }
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if (mode == KV2_PIPELINE_OPENCL) {
                dev->pipeline_name = "OpenCL";
                return new libfreenect2::OpenCLPacketPipeline();
            }
#endif
            if (mode == KV2_PIPELINE_CPU) {
                dev->pipeline_name = "CPU";
                return new libfreenect2::CpuPacketPipeline();
            }
            return nullptr;
        };

        libfreenect2::PacketPipeline* p = nullptr;
        if (dev->cfg.pipeline == KV2_PIPELINE_AUTO) {
            int order[] = {KV2_PIPELINE_CUDA, KV2_PIPELINE_OPENCL, KV2_PIPELINE_CPU};
            for (int m : order) {
                p = try_pipe(m);
                if (p) break;
            }
        } else {
            p = try_pipe(dev->cfg.pipeline);
        }
        if (!p) {
            dev->pipeline_name = "CPU";
            p = new libfreenect2::CpuPacketPipeline();
        }
        return p;
    };

    for (int attempt = 0; attempt < 3; ++attempt) {
        int n_devs = dev->freenect2.enumerateDevices();
        if (n_devs <= 0) {
            if (attempt < 2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
                continue;
            }
            set_err(err_buf, err_cap, "No Kinect v2 USB 3.0 sensor detected by libfreenect2");
            return nullptr;
        }
        dev->serial_str = (dev->cfg.serial && dev->cfg.serial[0])
            ? std::string(dev->cfg.serial)
            : dev->freenect2.getDefaultDeviceSerialNumber();

        dev->pipeline = make_pipeline();
        dev->f2_dev = dev->freenect2.openDevice(dev->serial_str, dev->pipeline);
        if (dev->f2_dev) {
            break;
        }
        dev->pipeline = nullptr; // openDevice deletes pipeline on failure
        if (attempt < 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(900));
        }
    }
    if (!dev->f2_dev) {
        set_err(err_buf, err_cap, "libfreenect2 failed to open Kinect v2 serial " + dev->serial_str);
        return nullptr;
    }
    // Keep status LEDs OFF until streaming actually starts (kv2_start)
    dev->set_device_leds(0u);

    unsigned int ftypes = 0;
    if (dev->cfg.stream_flags & (KV2_STREAM_DEPTH | KV2_STREAM_BODY | KV2_STREAM_REGISTERED_COLOR)) {
        ftypes |= libfreenect2::Frame::Depth;
    }
    if (dev->cfg.stream_flags & (KV2_STREAM_IR | KV2_STREAM_BODY)) {
        ftypes |= libfreenect2::Frame::Ir;
    }
    if (dev->cfg.stream_flags & (KV2_STREAM_COLOR | KV2_STREAM_REGISTERED_COLOR)) {
        ftypes |= libfreenect2::Frame::Color;
    }
    dev->listener = std::make_unique<libfreenect2::SyncMultiFrameListener>(ftypes);
    if (ftypes & libfreenect2::Frame::Color) {
        dev->f2_dev->setColorFrameListener(dev->listener.get());
    }
    if (ftypes & (libfreenect2::Frame::Depth | libfreenect2::Frame::Ir)) {
        dev->f2_dev->setIrAndDepthFrameListener(dev->listener.get());
    }

    // Pre-allocate all frame slots once (zero runtime malloc)
    for (int i = 0; i < kNumSlots; ++i) {
        if (dev->cfg.stream_flags & KV2_STREAM_COLOR) {
            dev->slots[i].color_bgrx.resize(kColorBytes, 0);
        }
        dev->slots[i].depth_mm.resize(kDepthCount, 0);
        dev->slots[i].ir.resize(kDepthCount, 0);
        if (dev->cfg.stream_flags & KV2_STREAM_REGISTERED_COLOR) {
            dev->slots[i].registered_bgrx.resize(kRegBytes, 0);
            dev->slots[i].undistorted_mm.resize(kDepthCount, 0.0f);
        }
        if (dev->cfg.stream_flags & KV2_STREAM_BODY_INDEX) {
            dev->slots[i].body_index.resize(kDepthCount, 255);
        }
    }

    return dev.release();
}

int kv2_start(Kv2Device* dev, char* err_buf, size_t err_cap) {
    if (!dev) return -1;
    if (dev->running.load()) return 0;

    bool want_rgb = (dev->cfg.stream_flags & (KV2_STREAM_COLOR | KV2_STREAM_REGISTERED_COLOR)) != 0;
    bool want_depth = (dev->cfg.stream_flags & (KV2_STREAM_DEPTH | KV2_STREAM_IR | KV2_STREAM_BODY | KV2_STREAM_REGISTERED_COLOR)) != 0;
    if (!dev->f2_dev->startStreams(want_rgb, want_depth)) {
        dev->set_device_leds(0u);
        set_err(err_buf, err_cap, "Freenect2Device::startStreams failed");
        return -1;
    }

    dev->ir_params = dev->f2_dev->getIrCameraParams();
    dev->color_params = dev->f2_dev->getColorCameraParams();
    dev->has_calib = true;
    dev->registration = std::make_unique<libfreenect2::Registration>(dev->ir_params, dev->color_params);

    if (dev->cfg.stream_flags & KV2_STREAM_BODY) {
        std::string werr;
        if (!dev->start_worker(werr)) {
            dev->f2_dev->stop();
            dev->set_device_leds(0u);
            set_err(err_buf, err_cap, werr);
            return -1;
        }
    }

    {
        std::lock_guard<std::mutex> lk(dev->pool_mtx);
        dev->pending_skel_slot = -1;
        dev->latest_published_slot = -1;
    }
    dev->running.store(true);
    if (dev->cfg.stream_flags & KV2_STREAM_BODY) {
        dev->skel_thread = std::thread(&Kv2Device::skel_loop, dev);
    }
    dev->capture_thread = std::thread(&Kv2Device::capture_loop, dev);
    return 0;
}

void kv2_stop(Kv2Device* dev) {
    if (!dev) return;
    if (!dev->running.exchange(false)) return;

    dev->skel_cv.notify_all();
    dev->consumer_cv.notify_all();
    if (dev->capture_thread.joinable()) dev->capture_thread.join();
    if (dev->skel_thread.joinable()) dev->skel_thread.join();

    dev->stop_worker();
    if (dev->f2_dev) {
        dev->f2_dev->stop();
        // Clear libfreenect2's stuck white LED immediately upon stream stop
        dev->set_device_leds(0u);
    }
}

void kv2_close(Kv2Device* dev) {
    if (!dev) return;
    kv2_stop(dev);
    if (dev->f2_dev) {
        dev->set_device_leds(0u);
        dev->f2_dev->close();
    }
    // Destroy listener (and any queued CudaFrame/OpenCLFrame inside it) before deleting f2_dev/pipeline.
    dev->registration.reset();
    dev->listener.reset();
    if (dev->f2_dev) {
        delete dev->f2_dev; // Also deletes dev->pipeline owned by Freenect2DeviceImpl
        dev->f2_dev = nullptr;
        dev->pipeline = nullptr;
    }
    delete dev;
}

int kv2_wait_frame(Kv2Device* dev, uint64_t last_seq, int32_t timeout_ms, Kv2FrameView* out_frame) {
    if (!dev || !out_frame) return -1;
    std::unique_lock<std::mutex> lk(dev->pool_mtx);
    auto pred = [&]() {
        if (!dev->running.load(std::memory_order_relaxed)) return true;
        int idx = dev->latest_published_slot;
        return idx >= 0 && dev->slots[idx].frame_seq > last_seq;
    };
    if (timeout_ms < 0) {
        dev->consumer_cv.wait(lk, pred);
    } else {
        if (!dev->consumer_cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), pred)) {
            return 0;
        }
    }
    int idx = dev->latest_published_slot;
    if (idx < 0 || dev->slots[idx].frame_seq <= last_seq) {
        return dev->running.load() ? 0 : -1;
    }
    dev->fill_view_locked(idx, out_frame);
    return 1;
}

int kv2_get_latest_frame(Kv2Device* dev, Kv2FrameView* out_frame) {
    if (!dev || !out_frame) return -1;
    std::lock_guard<std::mutex> lk(dev->pool_mtx);
    int idx = dev->latest_published_slot;
    if (idx < 0) return 0;
    dev->fill_view_locked(idx, out_frame);
    return 1;
}

void kv2_release_frame(Kv2Device* dev, Kv2FrameView* frame) {
    if (!dev || !frame) return;
    int idx = frame->_slot_id;
    if (idx >= 0 && idx < kNumSlots) {
        std::lock_guard<std::mutex> lk(dev->pool_mtx);
        if (dev->slots[idx].readers > 0) {
            dev->slots[idx].readers--;
        }
    }
    frame->_slot_id = -1;
    frame->color_bgrx = nullptr;
    frame->depth_mm = nullptr;
    frame->ir = nullptr;
    frame->registered_bgrx = nullptr;
    frame->undistorted_depth_mm = nullptr;
    frame->body_index = nullptr;
}

int kv2_reset_skeleton(Kv2Device* dev) {
    if (!dev) return -1;
    dev->reset_requested.store(true);
    return 0;
}

void kv2_set_registered_color_enabled(Kv2Device* dev, int enabled) {
    if (!dev) return;
    dev->reg_enabled_atomic.store(enabled != 0, std::memory_order_relaxed);
}

void kv2_get_memory_rss_mb(const Kv2Device* dev, float* out_host_rss_mb, float* out_worker_rss_mb) {
    if (out_host_rss_mb) *out_host_rss_mb = read_pid_rss_mb(0);
    if (out_worker_rss_mb) *out_worker_rss_mb = (dev && dev->worker_pid > 0) ? read_pid_rss_mb(dev->worker_pid) : 0.0f;
}

const char* kv2_get_serial(const Kv2Device* dev) {
    return dev ? dev->serial_str.c_str() : "";
}

const char* kv2_get_pipeline_name(const Kv2Device* dev) {
    return dev ? dev->pipeline_name.c_str() : "";
}

void kv2_get_intrinsics(const Kv2Device* dev, Kv2IrIntrinsics* out_ir, Kv2ColorIntrinsics* out_color) {
    if (!dev) return;
    if (out_ir) {
        out_ir->fx = dev->ir_params.fx;
        out_ir->fy = dev->ir_params.fy;
        out_ir->cx = dev->ir_params.cx;
        out_ir->cy = dev->ir_params.cy;
        out_ir->k1 = dev->ir_params.k1;
        out_ir->k2 = dev->ir_params.k2;
        out_ir->k3 = dev->ir_params.k3;
        out_ir->p1 = dev->ir_params.p1;
        out_ir->p2 = dev->ir_params.p2;
    }
    if (out_color) {
        out_color->fx = dev->color_params.fx;
        out_color->fy = dev->color_params.fy;
        out_color->cx = dev->color_params.cx;
        out_color->cy = dev->color_params.cy;
    }
}

void kv2_project_camera_to_depth(const Kv2Device* dev, const float pos_xyz[3], float out_uv[2]) {
    if (!dev || !pos_xyz || !out_uv) return;
    dev->project_to_depth(pos_xyz, out_uv);
}

void kv2_project_camera_to_color(const Kv2Device* dev, const float pos_xyz[3], float out_uv[2]) {
    if (!dev || !pos_xyz || !out_uv) return;
    dev->project_to_color(pos_xyz, out_uv);
}

int kv2_deproject_depth_to_camera(const Kv2Device* dev, int dx, int dy, float depth_mm, float out_xyz[3]) {
    if (!dev || !dev->has_calib || !out_xyz || depth_mm <= 1.0f) return -1;
    int col = dev->cfg.mirror_skeleton ? (KV2_DEPTH_WIDTH - 1 - dx) : dx;
    float xd = ((float)col - dev->ir_params.cx) / dev->ir_params.fx;
    float yd = ((float)dy - dev->ir_params.cy) / dev->ir_params.fy;
    float nx = xd, ny = yd;
    for (int it = 0; it < 5; ++it) {
        float r2 = nx * nx + ny * ny;
        float dxdy2 = 2.0f * nx * ny;
        float kr = 1.0f + ((dev->ir_params.k3 * r2 + dev->ir_params.k2) * r2 + dev->ir_params.k1) * r2;
        float tx = dev->ir_params.p2 * (r2 + 2.0f * nx * nx) + dev->ir_params.p1 * dxdy2;
        float ty = dev->ir_params.p1 * (r2 + 2.0f * ny * ny) + dev->ir_params.p2 * dxdy2;
        if (fabsf(kr) > 1e-6f) {
            nx = (xd - tx) / kr;
            ny = (yd - ty) / kr;
        }
    }
    float z_m = depth_mm * 0.001f;
    float sign_x = dev->cfg.mirror_skeleton ? -1.0f : 1.0f;
    out_xyz[0] = sign_x * nx * z_m;
    out_xyz[1] = -ny * z_m;
    out_xyz[2] = z_m;
    return 0;
}

int kv2_map_depth_to_color(const Kv2Device* dev, int dx, int dy, float depth_mm, float out_color_uv[2]) {
    float xyz[3];
    if (kv2_deproject_depth_to_camera(dev, dx, dy, depth_mm, xyz) != 0) {
        return -1;
    }
    dev->project_to_color(xyz, out_color_uv);
    return 0;
}

const char* kv2_joint_name(int joint_index) {
    if (joint_index < 0 || joint_index >= KV2_JOINT_COUNT) return "Unknown";
    return kJointNames[joint_index];
}

} /* extern "C" */
