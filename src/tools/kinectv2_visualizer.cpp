#include "kinectv2_sdk.h"

#include <SDL2/SDL.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop_signal = 0;
void handle_stop_signal(int) { g_stop_signal = 1; }

constexpr int kWinW = 1600;
constexpr int kWinH = 900;

struct BoneDef {
    int j0;
    int j1;
    int group; // 0=torso, 1=left, 2=right
};

constexpr BoneDef kBones[] = {
    {0, 1, 0}, {1, 20, 0}, {20, 2, 0}, {2, 3, 0},
    {20, 4, 1}, {4, 5, 1}, {5, 6, 1}, {6, 7, 1}, {7, 21, 1}, {6, 22, 1},
    {20, 8, 2}, {8, 9, 2}, {9, 10, 2}, {10, 11, 2}, {11, 23, 2}, {10, 24, 2},
    {0, 12, 1}, {12, 13, 1}, {13, 14, 1}, {14, 15, 1},
    {0, 16, 2}, {16, 17, 2}, {17, 18, 2}, {18, 19, 2},
};

cv::Scalar group_color(int grp, bool tracked) {
    // BGR colors
    cv::Scalar c;
    if (grp == 0) c = cv::Scalar(120, 240, 95);       // Green
    else if (grp == 1) c = cv::Scalar(255, 200, 65);  // Cyan-blue (Left)
    else c = cv::Scalar(70, 140, 255);                // Orange-red (Right)
    return tracked ? c : c * 0.55;
}

struct DepthLut {
    uint8_t bgr[4502][3];
    DepthLut() {
        bgr[0][0] = bgr[0][1] = bgr[0][2] = 16;
        for (int mm = 1; mm <= 4501; ++mm) {
            float t = std::min(1.0f, std::max(0.0f, (mm - 450.0f) / 3600.0f));
            // Smooth Turbo-like colormap (near=warm/yellow-red, mid=green/cyan, far=deep blue)
            float r = std::min(1.0f, std::max(0.0f, 1.5f - std::fabs(4.0f * t - 1.0f)));
            float g = std::min(1.0f, std::max(0.0f, 1.5f - std::fabs(4.0f * t - 2.0f)));
            float b = std::min(1.0f, std::max(0.0f, 1.5f - std::fabs(4.0f * t - 3.0f)));
            if (t < 0.25f) { r = 1.0f; g = t * 4.0f; b = 0.15f; }
            bgr[mm][0] = (uint8_t)(b * 235.0f);
            bgr[mm][1] = (uint8_t)(g * 235.0f);
            bgr[mm][2] = (uint8_t)(r * 245.0f);
        }
    }
};

void downsample_bgrx_1080_to_540_bgr(const uint8_t* src_bgrx, cv::Mat& dst_bgr) {
    // Fast 2x2 box downsample 1920x1080 BGRX -> 960x540 BGR
    for (int y = 0; y < 540; ++y) {
        const uint8_t* r0 = src_bgrx + (size_t)(y * 2 + 0) * 1920u * 4u;
        const uint8_t* r1 = src_bgrx + (size_t)(y * 2 + 1) * 1920u * 4u;
        uint8_t* out = dst_bgr.ptr<uint8_t>(y);
        for (int x = 0; x < 960; ++x) {
            const uint8_t* p00 = r0 + (size_t)(x * 2) * 4u;
            const uint8_t* p01 = p00 + 4u;
            const uint8_t* p10 = r1 + (size_t)(x * 2) * 4u;
            const uint8_t* p11 = p10 + 4u;
            out[x * 3 + 0] = (uint8_t)((p00[0] + p01[0] + p10[0] + p11[0]) >> 2);
            out[x * 3 + 1] = (uint8_t)((p00[1] + p01[1] + p10[1] + p11[1]) >> 2);
            out[x * 3 + 2] = (uint8_t)((p00[2] + p01[2] + p10[2] + p11[2]) >> 2);
        }
    }
}

void draw_skeleton_2d(cv::Mat& roi, const Kv2Body& body, bool use_color_uv, float sx, float sy) {
    auto get_pt = [&](int j, cv::Point& pt) -> bool {
        const Kv2Joint& jt = body.joints[j];
        if (jt.tracking_state == KV2_TRACKING_NOT_TRACKED) return false;
        float u = use_color_uv ? jt.color_uv[0] : jt.depth_uv[0];
        float v = use_color_uv ? jt.color_uv[1] : jt.depth_uv[1];
        if (u < -500.0f || v < -500.0f) return false;
        pt.x = (int)std::lround(u * sx);
        pt.y = (int)std::lround(v * sy);
        return (pt.x >= -40 && pt.x < roi.cols + 40 && pt.y >= -40 && pt.y < roi.rows + 40);
    };

    for (const auto& b : kBones) {
        cv::Point p0, p1;
        if (!get_pt(b.j0, p0) || !get_pt(b.j1, p1)) continue;
        bool both_tracked = (body.joints[b.j0].tracking_state == KV2_TRACKING_TRACKED &&
                             body.joints[b.j1].tracking_state == KV2_TRACKING_TRACKED);
        cv::line(roi, p0, p1, group_color(b.group, both_tracked), both_tracked ? 3 : 2, cv::LINE_AA);
    }

    for (int j = 0; j < KV2_JOINT_COUNT; ++j) {
        cv::Point p;
        if (!get_pt(j, p)) continue;
        bool tr = (body.joints[j].tracking_state == KV2_TRACKING_TRACKED);
        if (tr) {
            cv::circle(roi, p, 5, cv::Scalar(20, 20, 20), -1, cv::LINE_AA);
            cv::circle(roi, p, 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        } else {
            cv::circle(roi, p, 4, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        }
    }

    cv::Point pL, pR, pH;
    if (get_pt(KV2_JOINT_HAND_LEFT, pL)) {
        cv::putText(roi, "L", pL + cv::Point(7, -5), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 215, 90), 2, cv::LINE_AA);
    }
    if (get_pt(KV2_JOINT_HAND_RIGHT, pR)) {
        cv::putText(roi, "R", pR + cv::Point(7, -5), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(90, 150, 255), 2, cv::LINE_AA);
    }
    if (get_pt(KV2_JOINT_HEAD, pH)) {
        char zbuf[32];
        snprintf(zbuf, sizeof(zbuf), "%.2fm", body.joints[KV2_JOINT_HEAD].pos[2]);
        cv::putText(roi, zbuf, pH + cv::Point(-22, -12), cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(130, 250, 130), 2, cv::LINE_AA);
    }
}

void draw_metric_views(cv::Mat& canvas, cv::Rect area, const Kv2FrameView& fv, float min_range_m) {
    cv::rectangle(canvas, area, cv::Scalar(34, 37, 44), -1);
    cv::rectangle(canvas, area, cv::Scalar(65, 72, 85), 1);

    int half_w = area.width / 2;
    cv::Rect front_rc(area.x + 8, area.y + 30, half_w - 14, area.height - 38);
    cv::Rect top_rc(area.x + half_w + 6, area.y + 30, half_w - 14, area.height - 38);

    cv::rectangle(canvas, front_rc, cv::Scalar(24, 26, 31), -1);
    cv::rectangle(canvas, top_rc, cv::Scalar(24, 26, 31), -1);

    cv::putText(canvas, "3D Frontal (X, Y)", cv::Point(front_rc.x + 8, area.y + 21),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(200, 210, 225), 1, cv::LINE_AA);
    cv::putText(canvas, "Top-Down Radar (X, Z: 0..4m)", cv::Point(top_rc.x + 8, area.y + 21),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(200, 210, 225), 1, cv::LINE_AA);

    // Top-down radar grid & minimum skeleton threshold line
    for (int zm = 1; zm <= 4; ++zm) {
        int gy = top_rc.y + top_rc.height - (int)std::lround((zm / 4.2f) * top_rc.height);
        cv::line(canvas, cv::Point(top_rc.x, gy), cv::Point(top_rc.x + top_rc.width, gy), cv::Scalar(45, 50, 58), 1);
        char lbl[16];
        snprintf(lbl, sizeof(lbl), "%dm", zm);
        cv::putText(canvas, lbl, cv::Point(top_rc.x + 4, gy - 3), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(120, 130, 145), 1);
    }
    int y_min_z = top_rc.y + top_rc.height - (int)std::lround((min_range_m / 4.2f) * top_rc.height);
    cv::line(canvas, cv::Point(top_rc.x, y_min_z), cv::Point(top_rc.x + top_rc.width, y_min_z), cv::Scalar(70, 195, 135), 1);
    char min_lbl[32];
    snprintf(min_lbl, sizeof(min_lbl), "%.2fm min", min_range_m);
    cv::putText(canvas, min_lbl, cv::Point(top_rc.x + top_rc.width - 78, y_min_z - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(90, 225, 145), 1, cv::LINE_AA);

    // Frontal crosshair
    cv::line(canvas, cv::Point(front_rc.x + front_rc.width / 2, front_rc.y),
             cv::Point(front_rc.x + front_rc.width / 2, front_rc.y + front_rc.height), cv::Scalar(42, 46, 54), 1);
    cv::line(canvas, cv::Point(front_rc.x, front_rc.y + front_rc.height / 2),
             cv::Point(front_rc.x + front_rc.width, front_rc.y + front_rc.height / 2), cv::Scalar(42, 46, 54), 1);

    for (uint32_t b = 0; b < fv.body_count; ++b) {
        const Kv2Body& body = fv.bodies[b];
        auto f_pt = [&](int j) -> cv::Point {
            float x = body.joints[j].pos[0];
            float y = body.joints[j].pos[1];
            int px = front_rc.x + front_rc.width / 2 + (int)std::lround((x / 1.3f) * (front_rc.width * 0.46f));
            int py = front_rc.y + front_rc.height / 2 - (int)std::lround((y / 1.3f) * (front_rc.height * 0.46f));
            return cv::Point(px, py);
        };
        auto t_pt = [&](int j) -> cv::Point {
            float x = body.joints[j].pos[0];
            float z = body.joints[j].pos[2];
            int px = top_rc.x + top_rc.width / 2 + (int)std::lround((x / 1.6f) * (top_rc.width * 0.46f));
            int py = top_rc.y + top_rc.height - (int)std::lround((z / 4.2f) * top_rc.height);
            return cv::Point(px, py);
        };

        for (const auto& bone : kBones) {
            if (body.joints[bone.j0].tracking_state == 0 || body.joints[bone.j1].tracking_state == 0) continue;
            bool tr = (body.joints[bone.j0].tracking_state == 2 && body.joints[bone.j1].tracking_state == 2);
            cv::Scalar col = group_color(bone.group, tr);
            cv::line(canvas, f_pt(bone.j0), f_pt(bone.j1), col, 2, cv::LINE_AA);
            cv::line(canvas, t_pt(bone.j0), t_pt(bone.j1), col, 2, cv::LINE_AA);
        }
        for (int j = 0; j < KV2_JOINT_COUNT; ++j) {
            if (body.joints[j].tracking_state == 0) continue;
            cv::circle(canvas, f_pt(j), 3, cv::Scalar(245, 245, 245), -1, cv::LINE_AA);
            cv::circle(canvas, t_pt(j), 3, cv::Scalar(245, 245, 245), -1, cv::LINE_AA);
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    cv::setNumThreads(1);

    int max_frames = 0;
    std::string screenshot_path;
    int screenshot_at_frame = 45;
    bool show_registered = false;
    int mirror = 0;
    int pipeline_mode = KV2_PIPELINE_OPENCL; // Default OpenCL in Visualizer: lower CPU spin & lower VRAM than CUDA

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--max-frames") == 0 && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--save-screenshot") == 0 && i + 1 < argc) {
            screenshot_path = argv[++i];
        } else if (strcmp(argv[i], "--screenshot-frame") == 0 && i + 1 < argc) {
            screenshot_at_frame = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--mirror") == 0) {
            mirror = 1;
        } else if (strcmp(argv[i], "--pipeline") == 0 && i + 1 < argc) {
            const char* p = argv[++i];
            if (strcmp(p, "cuda") == 0) pipeline_mode = KV2_PIPELINE_CUDA;
            else if (strcmp(p, "opencl") == 0) pipeline_mode = KV2_PIPELINE_OPENCL;
            else if (strcmp(p, "cpu") == 0) pipeline_mode = KV2_PIPELINE_CPU;
            else pipeline_mode = KV2_PIPELINE_AUTO;
        }
    }

    Kv2Config cfg{};
    kv2_default_config(&cfg);
    cfg.pipeline = pipeline_mode;
    cfg.stream_flags = KV2_STREAM_ALL;
    cfg.mirror_skeleton = mirror;

    char err[512] = {0};
    Kv2Device* dev = kv2_open(&cfg, err, sizeof(err));
    if (!dev) {
        fprintf(stderr, "[kinectv2_visualizer] kv2_open failed: %s\n", err);
        return 1;
    }
    kv2_set_registered_color_enabled(dev, show_registered ? 1 : 0);
    if (kv2_start(dev, err, sizeof(err)) != 0) {
        fprintf(stderr, "[kinectv2_visualizer] kv2_start failed: %s\n", err);
        kv2_close(dev);
        return 1;
    }

    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "[kinectv2_visualizer] SDL_Init failed: %s\n", SDL_GetError());
        kv2_close(dev);
        return 1;
    }

    int win_x = SDL_WINDOWPOS_CENTERED;
    int win_y = SDL_WINDOWPOS_CENTERED;
    int n_disp = SDL_GetNumVideoDisplays();
    for (int d = 0; d < n_disp; ++d) {
        SDL_Rect b{};
        if (SDL_GetDisplayBounds(d, &b) == 0 && b.w >= b.h && b.w >= kWinW) {
            win_x = b.x + (b.w - kWinW) / 2;
            win_y = b.y + std::max(28, (b.h - kWinH) / 2 - 14);
            break;
        }
    }

    std::string title = std::string("Kinect v2 Linux Live 3D SDK Visualizer (Serial: ") +
                        kv2_get_serial(dev) + ", Pipeline: " + kv2_get_pipeline_name(dev) + ")";
    SDL_Window* win = SDL_CreateWindow(
        title.c_str(),
        win_x, win_y,
        kWinW, kWinH,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    if (!win) {
        fprintf(stderr, "[kinectv2_visualizer] SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        kv2_close(dev);
        return 1;
    }
    if (win_x != SDL_WINDOWPOS_CENTERED) {
        SDL_SetWindowPosition(win, win_x, win_y);
    }
    SDL_RaiseWindow(win);

    // Use MIT-SHM software blitter to avoid creating a competing OpenGL/GLX GPU context alongside Vulkan/OpenCL
    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer) {
        renderer = SDL_CreateRenderer(win, -1, 0);
    }
    SDL_RenderSetLogicalSize(renderer, kWinW, kWinH);
    SDL_Texture* tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, kWinW, kWinH);

    DepthLut depth_lut;
    cv::Mat canvas(kWinH, kWinW, CV_8UC3, cv::Scalar(18, 20, 24));
    cv::Mat rgb_half(540, 960, CV_8UC3);
    cv::Mat depth_vis(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH, CV_8UC3);
    cv::Mat depth_scaled(540, 608, CV_8UC3);

    uint64_t last_seq = 0;
    int rendered_frames = 0;
    float host_rss_mb = 0.0f;
    float worker_rss_mb = 0.0f;
    kv2_get_memory_rss_mb(dev, &host_rss_mb, &worker_rss_mb);
    std::deque<double> render_stamps;
    bool paused = false;
    bool quit = false;

    printf("[kinectv2_visualizer] Running on DISPLAY=%s (Serial=%s, Pipeline=%s)\n",
           getenv("DISPLAY") ? getenv("DISPLAY") : "(default)",
           kv2_get_serial(dev), kv2_get_pipeline_name(dev));
    fflush(stdout);

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    while (!quit && !g_stop_signal) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                quit = true;
            } else if (ev.type == SDL_WINDOWEVENT) {
                if (ev.window.event == SDL_WINDOWEVENT_CLOSE) {
                    quit = true;
                } else if (ev.window.event == SDL_WINDOWEVENT_MINIMIZED && !paused) {
                    kv2_stop(dev);
                    paused = true;
                } else if ((ev.window.event == SDL_WINDOWEVENT_RESTORED || ev.window.event == SDL_WINDOWEVENT_SHOWN) && paused) {
                    char rerr[256] = {0};
                    if (kv2_start(dev, rerr, sizeof(rerr)) == 0) {
                        last_seq = 0;
                        paused = false;
                    }
                }
            } else if (ev.type == SDL_KEYDOWN) {
                SDL_Keycode k = ev.key.keysym.sym;
                if (k == SDLK_ESCAPE || k == SDLK_q) {
                    quit = true;
                } else if (k == SDLK_r) {
                    kv2_reset_skeleton(dev);
                    printf("[kinectv2_visualizer] Skeleton tracker reset.\n");
                    fflush(stdout);
                } else if (k == SDLK_g) {
                    show_registered = !show_registered;
                    kv2_set_registered_color_enabled(dev, show_registered ? 1 : 0);
                } else if (k == SDLK_s) {
                    cv::imwrite("kinectv2_snapshot.png", canvas);
                    printf("[kinectv2_visualizer] Saved snapshot to kinectv2_snapshot.png\n");
                }
            }
        }
        if (quit) break;
        if (paused) {
            SDL_Delay(50);
            continue;
        }

        Kv2FrameView fv{};
        int rc = kv2_wait_frame(dev, last_seq, 100, &fv);
        if (rc <= 0) continue;
        last_seq = fv.frame_seq;

        auto t_now = std::chrono::steady_clock::now();
        double now_sec = std::chrono::duration<double>(t_now.time_since_epoch()).count();
        render_stamps.push_back(now_sec);
        while (render_stamps.size() > 31) render_stamps.pop_front();
        float render_fps = (render_stamps.size() >= 2)
            ? (float)((render_stamps.size() - 1) / std::max(1e-6, render_stamps.back() - render_stamps.front()))
            : 0.0f;

        canvas.setTo(cv::Scalar(18, 20, 24));

        // 1. Left Pane: 960x540 RGB + Projected 25-Joint Skeleton
        if (fv.color_bgrx) {
            downsample_bgrx_1080_to_540_bgr(fv.color_bgrx, rgb_half);
            for (uint32_t b = 0; b < fv.body_count; ++b) {
                draw_skeleton_2d(rgb_half, fv.bodies[b], true, 0.5f, 0.5f);
            }
            cv::putText(rgb_half, "RGB 1080p + Registered 3D Skeleton", cv::Point(14, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.68, cv::Scalar(240, 245, 250), 2, cv::LINE_AA);
            rgb_half.copyTo(canvas(cv::Rect(12, 12, 960, 540)));
        }

        // 2. Right Pane: Depth Colormap (or Registered RGB-D) + BodyIndex + Skeleton (616x540)
        float center_z_m = 0.0f;
        if (fv.depth_mm) {
            center_z_m = fv.depth_mm[(KV2_DEPTH_HEIGHT / 2) * KV2_DEPTH_WIDTH + (KV2_DEPTH_WIDTH / 2)] * 0.001f;
            for (int r = 0; r < KV2_DEPTH_HEIGHT; ++r) {
                const uint16_t* drow = fv.depth_mm + r * KV2_DEPTH_WIDTH;
                const uint8_t* brow  = fv.body_index ? (fv.body_index + r * KV2_DEPTH_WIDTH) : nullptr;
                const uint8_t* reg_row = (show_registered && fv.registered_bgrx)
                    ? (fv.registered_bgrx + (size_t)r * KV2_DEPTH_WIDTH * 4u) : nullptr;
                uint8_t* out = depth_vis.ptr<uint8_t>(r);
                for (int c = 0; c < KV2_DEPTH_WIDTH; ++c) {
                    uint8_t b, g, rv;
                    if (reg_row) {
                        b = reg_row[c * 4 + 0];
                        g = reg_row[c * 4 + 1];
                        rv = reg_row[c * 4 + 2];
                    } else {
                        uint16_t mm = std::min<uint16_t>(drow[c], 4501u);
                        b = depth_lut.bgr[mm][0];
                        g = depth_lut.bgr[mm][1];
                        rv = depth_lut.bgr[mm][2];
                    }
                    if (brow && brow[c] < 6) {
                        // Highlight segmented human body mask
                        b  = (uint8_t)std::min(255, (int)(b * 0.55f + 95));
                        g  = (uint8_t)std::min(255, (int)(g * 0.65f + 105));
                        rv = (uint8_t)std::min(255, (int)(rv * 0.55f + 35));
                    }
                    out[c * 3 + 0] = b;
                    out[c * 3 + 1] = g;
                    out[c * 3 + 2] = rv;
                }
            }
            cv::resize(depth_vis, depth_scaled, depth_scaled.size(), 0, 0, cv::INTER_NEAREST);
            float sx = (float)depth_scaled.cols / (float)KV2_DEPTH_WIDTH;
            float sy = (float)depth_scaled.rows / (float)KV2_DEPTH_HEIGHT;
            for (uint32_t b = 0; b < fv.body_count; ++b) {
                draw_skeleton_2d(depth_scaled, fv.bodies[b], false, sx, sy);
            }
            cv::putText(depth_scaled,
                        show_registered ? "Registered RGB-D (512x424) + Mask" : "Depth + BodyIndex + 3D Skeleton",
                        cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(245, 245, 250), 2, cv::LINE_AA);
            depth_scaled.copyTo(canvas(cv::Rect(980, 12, 608, 540)));
        }

        // 3. Bottom-Left Telemetry & Real-Time FPS HUD (960x324)
        cv::Rect hud_rc(12, 564, 960, 324);
        cv::rectangle(canvas, hud_rc, cv::Scalar(28, 31, 38), -1);
        cv::rectangle(canvas, hud_rc, cv::Scalar(62, 68, 82), 1);

        char line1[256];
        snprintf(line1, sizeof(line1),
                 "Sensor FPS: %5.1f   |   Skeleton FPS: %5.1f   |   Display FPS: %5.1f",
                 fv.sensor_fps, fv.skeleton_fps, render_fps);
        cv::Scalar fps_col = (fv.sensor_fps >= 28.5f) ? cv::Scalar(115, 245, 130) : cv::Scalar(90, 200, 255);
        cv::putText(canvas, line1, cv::Point(hud_rc.x + 20, hud_rc.y + 46),
                    cv::FONT_HERSHEY_SIMPLEX, 0.80, fps_col, 2, cv::LINE_AA);

        int tracked_joints = 0;
        int inferred_joints = 0;
        float subj_z = center_z_m;
        if (fv.body_count > 0) {
            for (int j = 0; j < KV2_JOINT_COUNT; ++j) {
                if (fv.bodies[0].joints[j].tracking_state == 2) tracked_joints++;
                else if (fv.bodies[0].joints[j].tracking_state == 1) inferred_joints++;
            }
            subj_z = fv.bodies[0].joints[KV2_JOINT_SPINE_MID].pos[2];
        }

        if (rendered_frames % 30 == 0) {
            kv2_get_memory_rss_mb(dev, &host_rss_mb, &worker_rss_mb);
            if (host_rss_mb + worker_rss_mb > 1536.0f) {
                fprintf(stderr, "[kinectv2_visualizer] Memory safeguard triggered (RSS=%.1f MB > 1536 MB); stopping cleanly.\n",
                        host_rss_mb + worker_rss_mb);
                kv2_release_frame(dev, &fv);
                break;
            }
        }

        char line2[256];
        snprintf(line2, sizeof(line2),
                 "Infer: %4.2f ms  |  Bodies: %u  |  Joints: %d (+%d) / 25  |  RAM: %.0f MB",
                 fv.skeleton_infer_ms, fv.body_count, tracked_joints, inferred_joints,
                 host_rss_mb + worker_rss_mb);
        cv::putText(canvas, line2, cv::Point(hud_rc.x + 20, hud_rc.y + 92),
                    cv::FONT_HERSHEY_SIMPLEX, 0.64, cv::Scalar(225, 232, 242), 2, cv::LINE_AA);

        // Tracking zone guide bar (pure Kinect v2 SDK: 1.45 - 3.8 m)
        std::string zone_msg;
        cv::Scalar zone_col;
        if (fv.body_count > 0) {
            char zb[160];
            snprintf(zb, sizeof(zb), "LOCKED 3D SKELETON  (Distance Z = %.2f m, ID = %llu)",
                     subj_z, (unsigned long long)fv.bodies[0].tracking_id);
            zone_msg = zb;
            zone_col = cv::Scalar(110, 245, 140);
        } else {
            char zb[160];
            snprintf(zb, sizeof(zb), "Kinect v2 Native SDK: Stand facing sensor at 1.45 - 3.8 m [Center Z=%.2fm]", center_z_m);
            zone_msg = zb;
            zone_col = cv::Scalar(185, 205, 230);
        }
        cv::putText(canvas, zone_msg, cv::Point(hud_rc.x + 20, hud_rc.y + 142),
                    cv::FONT_HERSHEY_SIMPLEX, 0.56, zone_col, 2, cv::LINE_AA);

        if (fv.body_count > 0) {
            const auto& b0 = fv.bodies[0];
            char jline1[256], jline2[256];
            snprintf(jline1, sizeof(jline1),
                     "Head: (%+.2f, %+.2f, %.2fm)   L-Hand: (%+.2f, %+.2f, %.2fm)   R-Hand: (%+.2f, %+.2f, %.2fm)",
                     b0.joints[KV2_JOINT_HEAD].pos[0], b0.joints[KV2_JOINT_HEAD].pos[1], b0.joints[KV2_JOINT_HEAD].pos[2],
                     b0.joints[KV2_JOINT_HAND_LEFT].pos[0], b0.joints[KV2_JOINT_HAND_LEFT].pos[1], b0.joints[KV2_JOINT_HAND_LEFT].pos[2],
                     b0.joints[KV2_JOINT_HAND_RIGHT].pos[0], b0.joints[KV2_JOINT_HAND_RIGHT].pos[1], b0.joints[KV2_JOINT_HAND_RIGHT].pos[2]);
            snprintf(jline2, sizeof(jline2),
                     "SpineMid: (%+.2f, %+.2f, %.2fm)   L-Shoulder: (%+.2f, %+.2f, %.2fm)   R-Shoulder: (%+.2f, %+.2f, %.2fm)",
                     b0.joints[KV2_JOINT_SPINE_MID].pos[0], b0.joints[KV2_JOINT_SPINE_MID].pos[1], b0.joints[KV2_JOINT_SPINE_MID].pos[2],
                     b0.joints[KV2_JOINT_SHOULDER_LEFT].pos[0], b0.joints[KV2_JOINT_SHOULDER_LEFT].pos[1], b0.joints[KV2_JOINT_SHOULDER_LEFT].pos[2],
                     b0.joints[KV2_JOINT_SHOULDER_RIGHT].pos[0], b0.joints[KV2_JOINT_SHOULDER_RIGHT].pos[1], b0.joints[KV2_JOINT_SHOULDER_RIGHT].pos[2]);
            cv::putText(canvas, jline1, cv::Point(hud_rc.x + 20, hud_rc.y + 195),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(200, 225, 250), 1, cv::LINE_AA);
            cv::putText(canvas, jline2, cv::Point(hud_rc.x + 20, hud_rc.y + 228),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(200, 225, 250), 1, cv::LINE_AA);
        }

        cv::putText(canvas,
                    "Controls:  [R] Reset Tracker    [G] Toggle Registered RGB-D    [S] Save Snapshot    [Q / ESC] Quit",
                    cv::Point(hud_rc.x + 20, hud_rc.y + 295),
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(145, 158, 175), 1, cv::LINE_AA);

        // 4. Bottom-Right 3D Metric Frontal & Top-Down Radar Views (608x324)
        draw_metric_views(canvas, cv::Rect(980, 564, 608, 324), fv, 1.45f);

        kv2_release_frame(dev, &fv);

        SDL_UpdateTexture(tex, nullptr, canvas.data, (int)canvas.step);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, tex, nullptr, nullptr);
        SDL_RenderPresent(renderer);
        if (rendered_frames == 0) {
            SDL_RestoreWindow(win);
            SDL_ShowWindow(win);
            SDL_RaiseWindow(win);
        }

        rendered_frames++;
        if (!screenshot_path.empty() && rendered_frames == screenshot_at_frame) {
            cv::imwrite(screenshot_path, canvas);
            printf("[kinectv2_visualizer] Saved verification screenshot to %s\n", screenshot_path.c_str());
            fflush(stdout);
        }
        if (max_frames > 0 && rendered_frames >= max_frames) {
            kv2_get_memory_rss_mb(dev, &host_rss_mb, &worker_rss_mb);
            printf("[kinectv2_visualizer] Completed %d frames (Sensor FPS=%.2f, Skeleton FPS=%.2f, Render FPS=%.2f, Host RSS=%.1f MB, Worker RSS=%.1f MB)\n",
                   rendered_frames, fv.sensor_fps, fv.skeleton_fps, render_fps, host_rss_mb, worker_rss_mb);
            fflush(stdout);
            break;
        }
    }

    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    kv2_close(dev);
    return 0;
}
