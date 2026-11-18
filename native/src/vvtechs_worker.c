#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <d3d11.h>
#include <fcntl.h>
#include <io.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vvtechs_abi.h"

/*
 * Standalone Windows / Wine64 host worker for Microsoft Kinect v2 3D Body Tracking (vvtechs.dll).
 *
 * Designed to run:
 *   1. Natively on Windows x64 (for golden verification against official Kinect v2 SDK)
 *   2. On Linux x86_64 under `wine64` + DXVK (Vulkan Compute), fed by native Linux `libfreenect2`.
 *
 * Wire Protocol (little-endian binary over --stdio or --port <tcp_port>):
 *   Request header (16 bytes):
 *     uint32_t magic       = 0x4B563242 ("KV2B")
 *     uint32_t cmd         = 1 (RESET), 2 (SET_INTRINSICS), 3 (PROCESS_FRAME), 9 (QUIT)
 *     uint32_t payload_len = byte length of following payload
 *     uint32_t flags       = bit 0: include 512x424 body_index in response (for PROCESS_FRAME)
 *                            bit 1: payload also contains 512x424 uint16_t IR after depth
 *
 *   Cmd 1 (RESET): payload_len = 0
 *     Response: 16-byte header (magic=0x4B563252 "KV2R", status=0, payload_len=0, frame_idx=0)
 *
 *   Cmd 2 (SET_INTRINSICS):
 *     Payload (28 bytes): float fx, fy, cx, cy, k1, k2, k3
 *       Worker builds the exact 512x424 float2 undistorted ray lookup table (0x1a8000 bytes)
 *       and calls IBodyTracker::SetCameraIntrinsics.
 *     Response: 16-byte header (magic="KV2R", status=0, payload_len=0, frame_idx=0)
 *
 *   Cmd 3 (PROCESS_FRAME):
 *     Payload:
 *       uint32_t frame_index
 *       float    gravity[4]                  (16 bytes, e.g. {0.0f, -1.0f, 0.0f, 0.0f})
 *       uint16_t depth[512 * 424]            (434,176 bytes, millimeters)
 *       [optional uint16_t ir[512 * 424] if (flags & 2)]
 *     Response:
 *       16-byte header (magic="KV2R", status=hr, payload_len, frame_index)
 *       NuiBodyFrameOutput (6,288 bytes = 0x1890: floor_clip_plane + 6 * NuiBodyData)
 *       [optional uint8_t body_index[512 * 424] (217,088 bytes) if (flags & 1)]
 */

#define KV2_REQ_MAGIC  0x4B563242u
#define KV2_RESP_MAGIC 0x4B563252u

#define KV2_CMD_RESET          1u
#define KV2_CMD_SET_INTRINSICS 2u
#define KV2_CMD_PROCESS_FRAME  3u
#define KV2_CMD_QUIT           9u

#define KV2_FLAG_WANT_BODY_INDEX 0x1u
#define KV2_FLAG_HAS_IR_FRAME    0x2u

typedef struct Kv2MsgHeader {
    uint32_t magic;
    uint32_t code;
    uint32_t payload_len;
    uint32_t meta;
} Kv2MsgHeader;

typedef struct Kv2IntrinsicsPayload {
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
} Kv2IntrinsicsPayload;

typedef struct WorkerState {
    HMODULE              h_d3d11;
    HMODULE              h_vvtechs;
    ID3D11Device*        device;
    ID3D11DeviceContext* context;
    IVisionTechsFactory* factory;
    IBodyTracker*        tracker;

    /* 64-byte aligned persistent buffers required by vvtechs.dll */
    float*               ray_table;      /* 512 * 424 * 2 floats (0x1a8000 bytes) */
    uint8_t*             scratch_256;    /* 0x100 bytes */
    NuiBodyFrameOutput*  body_out;       /* 0x1890 bytes */
    uint8_t*             extra_1280;     /* 0x500 bytes */
    float*               gravity_aligned;/* 16 bytes */

    uint16_t*            depth_512;      /* 512 * 424 uint16_t */
    uint16_t*            ir_512;         /* 512 * 424 uint16_t */
    uint16_t*            min_depth_128;  /* 128 * 106 uint16_t */
    uint16_t*            max_ir_128;     /* 128 * 106 uint16_t */
    uint8_t*             body_index_512; /* 512 * 424 uint8_t */
} WorkerState;

static void* alloc_aligned64(size_t size) {
    size_t total = size + 64u + sizeof(void*);
    uint8_t* raw = (uint8_t*)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, total);
    if (raw == NULL) {
        return NULL;
    }
    uintptr_t aligned = ((uintptr_t)(raw + sizeof(void*) + 63u)) & ~(uintptr_t)63u;
    ((void**)aligned)[-1] = raw;
    return (void*)aligned;
}

static void free_aligned64(void* ptr) {
    if (ptr != NULL) {
        void* raw = ((void**)ptr)[-1];
        HeapFree(GetProcessHeap(), 0, raw);
    }
}

static void release_vv_ref(void* obj_ptr) {
    if (obj_ptr == NULL) {
        return;
    }
    IBodyTracker* obj = (IBodyTracker*)obj_ptr;
    if (InterlockedDecrement(&obj->ref_count) == 0) {
        obj->vtbl->ScalarDeletingDtor(obj, 1);
    }
}

/*
 * Exact match to KinectService.exe RVA 0xbf60:
 * 4x4 block minimum of non-zero uint16_t depth values (0 if all 16 pixels are 0).
 */
static void downsample_depth_min_4x4(const uint16_t* src_512x424, uint16_t* dst_128x106) {
    for (int by = 0; by < KINECT_DS_HEIGHT; ++by) {
        const uint16_t* row0 = src_512x424 + (by * 4 + 0) * KINECT_DEPTH_WIDTH;
        const uint16_t* row1 = src_512x424 + (by * 4 + 1) * KINECT_DEPTH_WIDTH;
        const uint16_t* row2 = src_512x424 + (by * 4 + 2) * KINECT_DEPTH_WIDTH;
        const uint16_t* row3 = src_512x424 + (by * 4 + 3) * KINECT_DEPTH_WIDTH;
        uint16_t* out_row = dst_128x106 + by * KINECT_DS_WIDTH;
        for (int bx = 0; bx < KINECT_DS_WIDTH; ++bx) {
            uint16_t m = 0xffffu;
            int x0 = bx * 4;
            for (int dx = 0; dx < 4; ++dx) {
                uint16_t v0 = row0[x0 + dx];
                uint16_t v1 = row1[x0 + dx];
                uint16_t v2 = row2[x0 + dx];
                uint16_t v3 = row3[x0 + dx];
                if (v0 != 0 && v0 < m) m = v0;
                if (v1 != 0 && v1 < m) m = v1;
                if (v2 != 0 && v2 < m) m = v2;
                if (v3 != 0 && v3 < m) m = v3;
            }
            out_row[bx] = (m == 0xffffu) ? 0u : m;
        }
    }
}

/*
 * Exact match to KinectService.exe RVA 0xbe30:
 * 4x4 block maximum of uint16_t IR values.
 */
static void downsample_ir_max_4x4(const uint16_t* src_512x424, uint16_t* dst_128x106) {
    for (int by = 0; by < KINECT_DS_HEIGHT; ++by) {
        const uint16_t* row0 = src_512x424 + (by * 4 + 0) * KINECT_DEPTH_WIDTH;
        const uint16_t* row1 = src_512x424 + (by * 4 + 1) * KINECT_DEPTH_WIDTH;
        const uint16_t* row2 = src_512x424 + (by * 4 + 2) * KINECT_DEPTH_WIDTH;
        const uint16_t* row3 = src_512x424 + (by * 4 + 3) * KINECT_DEPTH_WIDTH;
        uint16_t* out_row = dst_128x106 + by * KINECT_DS_WIDTH;
        for (int bx = 0; bx < KINECT_DS_WIDTH; ++bx) {
            uint16_t m = 0u;
            int x0 = bx * 4;
            for (int dx = 0; dx < 4; ++dx) {
                uint16_t v0 = row0[x0 + dx];
                uint16_t v1 = row1[x0 + dx];
                uint16_t v2 = row2[x0 + dx];
                uint16_t v3 = row3[x0 + dx];
                if (v0 > m) m = v0;
                if (v1 > m) m = v1;
                if (v2 > m) m = v2;
                if (v3 > m) m = v3;
            }
            out_row[bx] = m;
        }
    }
}

/*
 * Builds the 512x424 undistorted normalized ray table (ray_x, ray_y) matching
 * vvtechs.dll RVA 0x34160 (+X right in selfie depth col 0..511, +Y up at row 0..423)
 * using iterative Brown-Conrady radial undistortion (k1, k2, k3).
 */
static void build_ray_table(float* ray_table, const Kv2IntrinsicsPayload* intr) {
    float fx = (intr->fx > 1.0f) ? intr->fx : 369.15f;
    float fy = (intr->fy > 1.0f) ? intr->fy : fx;
    float cx = intr->cx;
    float cy = intr->cy;
    float k1 = intr->k1;
    float k2 = intr->k2;
    float k3 = intr->k3;

    for (int v = 0; v < KINECT_DEPTH_HEIGHT; ++v) {
        for (int u = 0; u < KINECT_DEPTH_WIDTH; ++u) {
            /* Match vvtechs.dll RVA 0x34240 pixel-center sign convention */
            float xd = ((float)u - cx) / fx;
            float yd = (cy - (float)v) / fy;
            float xu = xd;
            float yu = yd;
            if (k1 != 0.0f || k2 != 0.0f || k3 != 0.0f) {
                for (int it = 0; it < 5; ++it) {
                    float r2 = xu * xu + yu * yu;
                    float r4 = r2 * r2;
                    float r6 = r4 * r2;
                    float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
                    if (fabsf(radial) > 1e-6f) {
                        xu = xd / radial;
                        yu = yd / radial;
                    }
                }
            }
            size_t idx = ((size_t)v * KINECT_DEPTH_WIDTH + (size_t)u) * 2u;
            ray_table[idx + 0] = xu;
            ray_table[idx + 1] = yu;
        }
    }
}

static void resolve_dll_dir(const char* utf8_arg, wchar_t* out_dir, size_t cap) {
    out_dir[0] = 0;
    if (utf8_arg != NULL && utf8_arg[0] != 0) {
        if (utf8_arg[0] == '/') {
            typedef LPWSTR (__cdecl *PFN_wine_get_dos_file_name)(LPCSTR);
            HMODULE h_k32 = GetModuleHandleW(L"kernel32.dll");
            PFN_wine_get_dos_file_name pfn_dos = h_k32
                ? (PFN_wine_get_dos_file_name)(void*)GetProcAddress(h_k32, "wine_get_dos_file_name")
                : NULL;
            if (pfn_dos != NULL) {
                LPWSTR dos_path = pfn_dos(utf8_arg);
                if (dos_path != NULL) {
                    wcsncpy(out_dir, dos_path, cap - 1);
                    out_dir[cap - 1] = 0;
                    HeapFree(GetProcessHeap(), 0, dos_path);
                    return;
                }
            }
            wchar_t tmp[MAX_PATH * 2];
            MultiByteToWideChar(CP_UTF8, 0, utf8_arg, -1, tmp, MAX_PATH * 2 - 1);
            tmp[MAX_PATH * 2 - 1] = 0;
            for (wchar_t* p = tmp; *p; ++p) {
                if (*p == L'/') *p = L'\\';
            }
            _snwprintf(out_dir, cap - 1, L"Z:%ls", tmp);
            out_dir[cap - 1] = 0;
            return;
        }
        MultiByteToWideChar(CP_UTF8, 0, utf8_arg, -1, out_dir, (int)(cap - 1));
        out_dir[cap - 1] = 0;
        return;
    }

    /* Default: check whether vvtechs.dll sits beside vvtechs_worker.exe */
    wchar_t exe_path[MAX_PATH * 2];
    DWORD n = GetModuleFileNameW(NULL, exe_path, MAX_PATH * 2 - 1);
    if (n > 0 && n < MAX_PATH * 2 - 1) {
        exe_path[n] = 0;
        wchar_t* last_sep = wcsrchr(exe_path, L'\\');
        if (last_sep == NULL) {
            last_sep = wcsrchr(exe_path, L'/');
        }
        if (last_sep != NULL) {
            *last_sep = 0;
            wchar_t probe[MAX_PATH * 2];
            _snwprintf(probe, MAX_PATH * 2 - 1, L"%ls\\vvtechs.dll", exe_path);
            probe[MAX_PATH * 2 - 1] = 0;
            if (GetFileAttributesW(probe) != INVALID_FILE_ATTRIBUTES) {
                wcsncpy(out_dir, exe_path, cap - 1);
                out_dir[cap - 1] = 0;
            }
        }
    }
}

static int is_running_under_wine(void) {
    HMODULE h_ntdll = GetModuleHandleW(L"ntdll.dll");
    if (h_ntdll != NULL && GetProcAddress(h_ntdll, "wine_get_version") != NULL) {
        return 1;
    }
    HMODULE h_k32 = GetModuleHandleW(L"kernel32.dll");
    if (h_k32 != NULL && GetProcAddress(h_k32, "wine_get_dos_file_name") != NULL) {
        return 1;
    }
    return 0;
}

/*
 * Direct3D 11 specifies DXBC `f32tof16` as Round-Toward-Zero (RTZ), so Microsoft's
 * HLSL shaders (#08 and #10..#15 in vvtechs.dll) manually add a half-ULP pre-bias
 * (`and rY, rX, 4096; movc rY, rY, 1, 0; iadd rX, rX, rY; iadd rX, rX, 4095`)
 * right before `f32tof16` to emulate Round-To-Nearest-Even (RTE).
 *
 * On Linux Vulkan, both DXVK and vkd3d-shader translate `f32tof16` to SPIR-V
 * `GLSL.std.450 PackHalf2x16`, which performs hardware RTE rather than RTZ,
 * double-rounding every 3D joint vote by +0.5 float16 ULP (~3.1 mm median shift).
 * Zeroing that manual RTZ->RTE pre-bias in memory inside CreateComputeShader
 * lets Vulkan's native RTE match Windows D3D11 RTZ(+bias) without touching vvtechs.dll on disk.
 */
static void compute_dxbc_md5(const uint8_t* data, size_t msg_len, uint8_t out_digest[16]) {
    static const uint8_t s[64] = {
        7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
        5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
        4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
        6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21
    };
    static const uint32_t K[64] = {
        0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu,
        0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
        0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu,
        0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
        0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau,
        0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
        0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu,
        0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
        0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu,
        0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
        0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u,
        0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
        0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u,
        0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
        0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
        0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u
    };

    uint32_t a0 = 0x67452301u;
    uint32_t b0 = 0xefcdab89u;
    uint32_t c0 = 0x98badcfeu;
    uint32_t d0 = 0x10325476u;

    uint32_t num_bits = (uint32_t)(msg_len * 8u);
    uint32_t num_dwords_mod = (uint32_t)((msg_len * 2u) | 1u);
    size_t n_full = (msg_len / 64u) * 64u;
    size_t rem_len = msg_len - n_full;

    uint8_t tail[128];
    memset(tail, 0, sizeof(tail));
    size_t tail_blocks = 1u;
    if (rem_len >= 56u) {
        memcpy(tail, data + n_full, rem_len);
        tail[rem_len] = 0x80u;
        memcpy(tail + 64u, &num_bits, 4u);
        memcpy(tail + 124u, &num_dwords_mod, 4u);
        tail_blocks = 2u;
    } else {
        memcpy(tail, &num_bits, 4u);
        if (rem_len > 0u) {
            memcpy(tail + 4u, data + n_full, rem_len);
        }
        tail[4u + rem_len] = 0x80u;
        memcpy(tail + 60u, &num_dwords_mod, 4u);
        tail_blocks = 1u;
    }

    size_t total_blocks = (n_full / 64u) + tail_blocks;
    for (size_t bi = 0; bi < total_blocks; ++bi) {
        const uint8_t* blk = (bi < (n_full / 64u))
            ? (data + bi * 64u)
            : (tail + (bi - (n_full / 64u)) * 64u);
        uint32_t M[16];
        memcpy(M, blk, 64u);

        uint32_t A = a0, B = b0, C = c0, D = d0;
        for (uint32_t i = 0; i < 64u; ++i) {
            uint32_t F = 0, g = 0;
            if (i < 16u) {
                F = (B & C) | ((~B) & D);
                g = i;
            } else if (i < 32u) {
                F = (D & B) | ((~D) & C);
                g = (5u * i + 1u) & 15u;
            } else if (i < 48u) {
                F = B ^ C ^ D;
                g = (3u * i + 5u) & 15u;
            } else {
                F = C ^ (B | (~D));
                g = (7u * i) & 15u;
            }
            F = F + A + K[i] + M[g];
            A = D;
            D = C;
            C = B;
            uint32_t rot = (F << s[i]) | (F >> (32u - s[i]));
            B = B + rot;
        }
        a0 += A;
        b0 += B;
        c0 += C;
        d0 += D;
    }

    memcpy(out_digest +  0, &a0, 4u);
    memcpy(out_digest +  4, &b0, 4u);
    memcpy(out_digest +  8, &c0, 4u);
    memcpy(out_digest + 12, &d0, 4u);
}

static int patch_dxbc_vulkan_f16(uint8_t* buf, size_t len, uint32_t* out_iadd, uint32_t* out_and) {
    *out_iadd = 0;
    *out_and = 0;
    if (len < 32u || memcmp(buf, "DXBC", 4u) != 0) {
        return 0;
    }
    uint32_t chunk_count = 0;
    memcpy(&chunk_count, buf + 28u, 4u);
    if (chunk_count > 64u || 32u + (size_t)chunk_count * 4u > len) {
        return 0;
    }

    size_t shex_off = 0;
    uint32_t shex_len = 0;
    for (uint32_t ci = 0; ci < chunk_count; ++ci) {
        uint32_t coff = 0;
        memcpy(&coff, buf + 32u + ci * 4u, 4u);
        if ((size_t)coff + 8u <= len &&
            (memcmp(buf + coff, "SHEX", 4u) == 0 || memcmp(buf + coff, "SHDR", 4u) == 0)) {
            memcpy(&shex_len, buf + coff + 4u, 4u);
            shex_off = (size_t)coff + 8u;
            break;
        }
    }
    if (shex_off == 0 || shex_len < 8u || shex_off + (size_t)shex_len > len) {
        return 0;
    }

    uint32_t n_dwords = 0;
    memcpy(&n_dwords, buf + shex_off + 4u, 4u);
    if (n_dwords < 2u || (size_t)n_dwords * 4u > (size_t)shex_len) {
        return 0;
    }

    uint32_t* words = (uint32_t*)(void*)(buf + shex_off);
    const uint32_t OP_AND = 1u;
    const uint32_t OP_IADD = 30u;
    const uint32_t OP_CUSTOMDATA = 53u;

    /* Pass 1: check whether this shader uses the IADD 4095 f32tof16 pre-bias */
    int has_iadd_4095 = 0;
    uint32_t pc = 2u;
    while (pc < n_dwords) {
        uint32_t tok = words[pc];
        uint32_t op = tok & 0x7ffu;
        if (op == OP_CUSTOMDATA) {
            uint32_t clen = (pc + 1u < n_dwords) ? words[pc + 1u] : (n_dwords - pc);
            if (clen == 0u || clen > n_dwords - pc) break;
            pc += clen;
            continue;
        }
        uint32_t ilen = (tok >> 24) & 0x7fu;
        if (ilen == 0u || ilen > n_dwords - pc) break;
        if (op == OP_IADD) {
            for (uint32_t k = 1u; k < ilen; ++k) {
                if (words[pc + k] == 4095u) {
                    has_iadd_4095 = 1;
                    break;
                }
            }
        }
        pc += ilen;
    }
    if (!has_iadd_4095) {
        return 0;
    }

    /* Pass 2: zero immediate 4095 in IADD and immediate 4096 in AND */
    pc = 2u;
    while (pc < n_dwords) {
        uint32_t tok = words[pc];
        uint32_t op = tok & 0x7ffu;
        if (op == OP_CUSTOMDATA) {
            uint32_t clen = (pc + 1u < n_dwords) ? words[pc + 1u] : (n_dwords - pc);
            if (clen == 0u || clen > n_dwords - pc) break;
            pc += clen;
            continue;
        }
        uint32_t ilen = (tok >> 24) & 0x7fu;
        if (ilen == 0u || ilen > n_dwords - pc) break;
        if (op == OP_IADD || op == OP_AND) {
            uint32_t target_imm = (op == OP_IADD) ? 4095u : 4096u;
            uint32_t cur = tok;
            uint32_t opc = pc + 1u;
            while (((cur >> 31) & 1u) != 0u && opc < pc + ilen) {
                cur = words[opc++];
            }
            int hit = 0;
            while (opc < pc + ilen) {
                uint32_t op_tok = words[opc];
                uint32_t num_comps = op_tok & 0x3u;
                uint32_t op_type = (op_tok >> 12) & 0xffu;
                uint32_t idx_dim = (op_tok >> 20) & 0x3u;
                uint32_t ext = (op_tok >> 31) & 1u;
                opc += 1u + ext;
                if (op_type == 4u) { /* D3D10_SB_OPERAND_TYPE_IMMEDIATE32 */
                    uint32_t n_imm = (num_comps == 1u) ? 1u : ((num_comps == 2u) ? 4u : 0u);
                    for (uint32_t j = 0; j < n_imm && opc + j < pc + ilen; ++j) {
                        if (words[opc + j] == target_imm) {
                            words[opc + j] = 0u;
                            hit = 1;
                        }
                    }
                    opc += n_imm;
                } else {
                    for (uint32_t d = 0; d < idx_dim; ++d) {
                        uint32_t rep = (op_tok >> (22u + d * 3u)) & 0x7u;
                        if (rep == 0u) {
                            opc += 1u;
                        } else if (rep == 1u) {
                            opc += 2u;
                        } else if (rep >= 2u && rep <= 4u) {
                            if (rep == 3u) opc += 1u;
                            else if (rep == 4u) opc += 2u;
                            if (opc < pc + ilen) {
                                uint32_t sub_ext = (words[opc] >> 31) & 1u;
                                opc += 1u + sub_ext;
                            }
                        }
                    }
                }
            }
            if (hit) {
                if (op == OP_IADD) (*out_iadd)++;
                else (*out_and)++;
            }
        }
        pc += ilen;
    }

    compute_dxbc_md5(buf + 20u, len - 20u, buf + 4u);
    return 1;
}

typedef HRESULT (STDMETHODCALLTYPE *PFN_CreateComputeShader)(
    ID3D11Device* This,
    const void* pShaderBytecode,
    SIZE_T BytecodeLength,
    ID3D11ClassLinkage* pClassLinkage,
    ID3D11ComputeShader** ppComputeShader
);

static PFN_CreateComputeShader g_orig_create_cs = NULL;
static uint32_t g_patched_shader_count = 0;
static uint32_t g_patched_iadd_total = 0;

static HRESULT STDMETHODCALLTYPE hooked_CreateComputeShader(
    ID3D11Device* This,
    const void* pShaderBytecode,
    SIZE_T BytecodeLength,
    ID3D11ClassLinkage* pClassLinkage,
    ID3D11ComputeShader** ppComputeShader
) {
    if (pShaderBytecode != NULL && BytecodeLength >= 32u) {
        uint8_t* copy = (uint8_t*)HeapAlloc(GetProcessHeap(), 0, BytecodeLength);
        if (copy != NULL) {
            memcpy(copy, pShaderBytecode, BytecodeLength);
            uint32_t c_iadd = 0, c_and = 0;
            if (patch_dxbc_vulkan_f16(copy, BytecodeLength, &c_iadd, &c_and)) {
                g_patched_shader_count++;
                g_patched_iadd_total += c_iadd;
                HRESULT hr = g_orig_create_cs(This, copy, BytecodeLength, pClassLinkage, ppComputeShader);
                HeapFree(GetProcessHeap(), 0, copy);
                return hr;
            }
            HeapFree(GetProcessHeap(), 0, copy);
        }
    }
    return g_orig_create_cs(This, pShaderBytecode, BytecodeLength, pClassLinkage, ppComputeShader);
}

static void install_cs_f16_hook(ID3D11Device* dev) {
    if (dev == NULL || dev->lpVtbl == NULL || g_orig_create_cs != NULL) {
        return;
    }
    void** slot = (void**)&dev->lpVtbl->CreateComputeShader;
    DWORD old_prot = 0;
    if (VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &old_prot)) {
        g_orig_create_cs = dev->lpVtbl->CreateComputeShader;
        dev->lpVtbl->CreateComputeShader = hooked_CreateComputeShader;
        DWORD dummy = 0;
        VirtualProtect(slot, sizeof(void*), old_prot, &dummy);
    }
}

static void remove_cs_f16_hook(ID3D11Device* dev) {
    if (dev == NULL || dev->lpVtbl == NULL || g_orig_create_cs == NULL) {
        return;
    }
    void** slot = (void**)&dev->lpVtbl->CreateComputeShader;
    DWORD old_prot = 0;
    if (VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &old_prot)) {
        dev->lpVtbl->CreateComputeShader = g_orig_create_cs;
        DWORD dummy = 0;
        VirtualProtect(slot, sizeof(void*), old_prot, &dummy);
    }
    g_orig_create_cs = NULL;
}

static int init_worker(WorkerState* st, const wchar_t* dll_dir, int use_warp, int vulkan_f16_fix) {
    memset(st, 0, sizeof(*st));

    /* Ensure DLL search directory covers dll_dir first (for k4wcll.dll stub + vvtechs.dll) */
    if (dll_dir != NULL && dll_dir[0] != 0) {
        SetDllDirectoryW(dll_dir);
    }

    st->h_d3d11 = LoadLibraryW(L"d3d11.dll");
    if (st->h_d3d11 == NULL) {
        fprintf(stderr, "[vvtechs_worker] Failed to load d3d11.dll (err=%lu)\n", GetLastError());
        return -1;
    }

    PFN_D3D11_CREATE_DEVICE pfn_create_dev =
        (PFN_D3D11_CREATE_DEVICE)(void*)GetProcAddress(st->h_d3d11, "D3D11CreateDevice");
    if (pfn_create_dev == NULL) {
        fprintf(stderr, "[vvtechs_worker] Missing D3D11CreateDevice\n");
        return -1;
    }

    D3D_FEATURE_LEVEL fl_in = D3D_FEATURE_LEVEL_11_0;
    D3D_FEATURE_LEVEL fl_out = D3D_FEATURE_LEVEL_11_0;
    D3D_DRIVER_TYPE drv_type = use_warp ? D3D_DRIVER_TYPE_WARP : D3D_DRIVER_TYPE_HARDWARE;
    HRESULT hr = pfn_create_dev(
        NULL,
        drv_type,
        NULL,
        D3D11_CREATE_DEVICE_SINGLETHREADED,
        &fl_in,
        1,
        D3D11_SDK_VERSION,
        &st->device,
        &fl_out,
        &st->context
    );
    if (FAILED(hr) && !use_warp) {
        fprintf(stderr, "[vvtechs_worker] Hardware D3D11 failed (0x%08lx), trying WARP...\n", (unsigned long)hr);
        hr = pfn_create_dev(
            NULL,
            D3D_DRIVER_TYPE_WARP,
            NULL,
            D3D11_CREATE_DEVICE_SINGLETHREADED,
            &fl_in,
            1,
            D3D11_SDK_VERSION,
            &st->device,
            &fl_out,
            &st->context
        );
    }
    if (FAILED(hr)) {
        fprintf(stderr, "[vvtechs_worker] D3D11CreateDevice failed: 0x%08lx\n", (unsigned long)hr);
        return -1;
    }

    int apply_f16_fix = (vulkan_f16_fix > 0) || (vulkan_f16_fix < 0 && is_running_under_wine() && !use_warp);
    if (apply_f16_fix) {
        install_cs_f16_hook(st->device);
    }

    wchar_t vv_path[MAX_PATH * 2];
    if (dll_dir != NULL && dll_dir[0] != 0) {
        _snwprintf(vv_path, MAX_PATH * 2 - 1, L"%ls\\vvtechs.dll", dll_dir);
        vv_path[MAX_PATH * 2 - 1] = 0;
    } else {
        wcscpy(vv_path, L"C:\\Windows\\System32\\Kinect\\vvtechs.dll");
    }

    st->h_vvtechs = LoadLibraryW(vv_path);
    if (st->h_vvtechs == NULL) {
        fprintf(stderr, "[vvtechs_worker] Failed to load vvtechs.dll from %ls (err=%lu)\n", vv_path, GetLastError());
        return -1;
    }

    PFN_NuiVisionCreateFactory pfn_factory =
        (PFN_NuiVisionCreateFactory)(void*)GetProcAddress(st->h_vvtechs, "NuiVisionCreateFactory");
    if (pfn_factory == NULL) {
        fprintf(stderr, "[vvtechs_worker] NuiVisionCreateFactory export not found\n");
        return -1;
    }

    int32_t vhr = pfn_factory(&st->factory);
    if (vhr < 0 || st->factory == NULL) {
        fprintf(stderr, "[vvtechs_worker] NuiVisionCreateFactory failed: 0x%08x\n", (unsigned)vhr);
        return -1;
    }

    vhr = st->factory->vtbl->CreateBodyTracker(
        st->factory,
        &st->tracker,
        st->device,
        st->context,
        0ULL,
        NULL,
        NULL,
        NULL
    );
    if (vhr < 0 || st->tracker == NULL) {
        fprintf(stderr, "[vvtechs_worker] CreateBodyTracker failed: 0x%08x\n", (unsigned)vhr);
        return -1;
    }

    st->ray_table       = (float*)alloc_aligned64(KINECT_DEPTH_PIXELS * 2u * sizeof(float));
    st->scratch_256     = (uint8_t*)alloc_aligned64(0x100u);
    st->body_out        = (NuiBodyFrameOutput*)alloc_aligned64(sizeof(NuiBodyFrameOutput));
    st->extra_1280      = (uint8_t*)alloc_aligned64(0x500u);
    st->gravity_aligned = (float*)alloc_aligned64(16u);

    st->depth_512       = (uint16_t*)alloc_aligned64(KINECT_DEPTH_PIXELS * sizeof(uint16_t));
    st->ir_512          = (uint16_t*)alloc_aligned64(KINECT_DEPTH_PIXELS * sizeof(uint16_t));
    st->min_depth_128   = (uint16_t*)alloc_aligned64(KINECT_DS_PIXELS * sizeof(uint16_t));
    st->max_ir_128      = (uint16_t*)alloc_aligned64(KINECT_DS_PIXELS * sizeof(uint16_t));
    st->body_index_512  = (uint8_t*)alloc_aligned64(KINECT_DEPTH_PIXELS * sizeof(uint8_t));

    if (!st->ray_table || !st->scratch_256 || !st->body_out || !st->extra_1280 ||
        !st->gravity_aligned || !st->depth_512 || !st->ir_512 ||
        !st->min_depth_128 || !st->max_ir_128 || !st->body_index_512) {
        fprintf(stderr, "[vvtechs_worker] Buffer allocation failed\n");
        return -1;
    }

    /* Default IR fill when caller sends depth-only frames */
    for (size_t i = 0; i < KINECT_DS_PIXELS; ++i) {
        st->max_ir_128[i] = 4000u;
    }

    /* Initialize default Kinect v2 intrinsics & 512x424 Ray Lookup Table */
    st->tracker->vtbl->Reset(st->tracker);
    fprintf(
        stderr,
        "[vvtechs_worker] Ready (D3D11 FL=0x%x, BodyTracker=%p, vulkan_f16_fix=%d, patched_cs=%u/%u)\n",
        (unsigned)fl_out,
        (void*)st->tracker,
        apply_f16_fix,
        g_patched_shader_count,
        g_patched_iadd_total
    );
    fflush(stderr);
    return 0;
}

static void shutdown_worker(WorkerState* st) {
    remove_cs_f16_hook(st->device);
    release_vv_ref(st->tracker);
    st->tracker = NULL;
    release_vv_ref(st->factory);
    st->factory = NULL;
    if (st->context != NULL) {
        st->context->lpVtbl->Release(st->context);
        st->context = NULL;
    }
    if (st->device != NULL) {
        st->device->lpVtbl->Release(st->device);
        st->device = NULL;
    }
    free_aligned64(st->ray_table);
    free_aligned64(st->scratch_256);
    free_aligned64(st->body_out);
    free_aligned64(st->extra_1280);
    free_aligned64(st->gravity_aligned);
    free_aligned64(st->depth_512);
    free_aligned64(st->ir_512);
    free_aligned64(st->min_depth_128);
    free_aligned64(st->max_ir_128);
    free_aligned64(st->body_index_512);
}

typedef int (*IoReadFn)(void* ctx, void* buf, size_t len);
typedef int (*IoWriteFn)(void* ctx, const void* buf, size_t len);

typedef struct StdioHandles {
    HANDLE h_in;
    HANDLE h_out;
} StdioHandles;

static int stdio_read_exact(void* ctx, void* buf, size_t len) {
    StdioHandles* io = (StdioHandles*)ctx;
    uint8_t* p = (uint8_t*)buf;
    size_t rem = len;
    while (rem > 0) {
        DWORD chunk = (rem > 0x40000000u) ? 0x40000000u : (DWORD)rem;
        DWORD got = 0;
        if (!ReadFile(io->h_in, p, chunk, &got, NULL) || got == 0) {
            return -1;
        }
        p += got;
        rem -= (size_t)got;
    }
    return 0;
}

static int stdio_write_exact(void* ctx, const void* buf, size_t len) {
    StdioHandles* io = (StdioHandles*)ctx;
    const uint8_t* p = (const uint8_t*)buf;
    size_t rem = len;
    while (rem > 0) {
        DWORD chunk = (rem > 0x40000000u) ? 0x40000000u : (DWORD)rem;
        DWORD put = 0;
        if (!WriteFile(io->h_out, p, chunk, &put, NULL) || put == 0) {
            return -1;
        }
        p += put;
        rem -= (size_t)put;
    }
    return 0;
}

static int sock_read_exact(void* ctx, void* buf, size_t len) {
    SOCKET s = *(SOCKET*)ctx;
    char* p = (char*)buf;
    size_t rem = len;
    while (rem > 0) {
        int chunk = (rem > 0x40000000u) ? 0x40000000 : (int)rem;
        int got = recv(s, p, chunk, 0);
        if (got <= 0) {
            return -1;
        }
        p += got;
        rem -= (size_t)got;
    }
    return 0;
}

static int sock_write_exact(void* ctx, const void* buf, size_t len) {
    SOCKET s = *(SOCKET*)ctx;
    const char* p = (const char*)buf;
    size_t rem = len;
    while (rem > 0) {
        int chunk = (rem > 0x40000000u) ? 0x40000000 : (int)rem;
        int put = send(s, p, chunk, 0);
        if (put <= 0) {
            return -1;
        }
        p += put;
        rem -= (size_t)put;
    }
    return 0;
}

static int serve_stream(WorkerState* st, IoReadFn rd, IoWriteFn wr, void* io_ctx) {
    for (;;) {
        Kv2MsgHeader req;
        if (rd(io_ctx, &req, sizeof(req)) != 0) {
            return 0;
        }
        if (req.magic != KV2_REQ_MAGIC) {
            fprintf(stderr, "[vvtechs_worker] Invalid request magic: 0x%08x\n", req.magic);
            return -1;
        }

        if (req.code == KV2_CMD_QUIT) {
            Kv2MsgHeader resp = { KV2_RESP_MAGIC, 0u, 0u, 0u };
            wr(io_ctx, &resp, sizeof(resp));
            return 0;
        } else if (req.code == KV2_CMD_RESET) {
            st->tracker->vtbl->Reset(st->tracker);
            Kv2MsgHeader resp = { KV2_RESP_MAGIC, 0u, 0u, 0u };
            if (wr(io_ctx, &resp, sizeof(resp)) != 0) return -1;
        } else if (req.code == KV2_CMD_SET_INTRINSICS) {
            if (req.payload_len != sizeof(Kv2IntrinsicsPayload)) {
                fprintf(stderr, "[vvtechs_worker] Bad SET_INTRINSICS len=%u\n", req.payload_len);
                return -1;
            }
            Kv2IntrinsicsPayload intr;
            if (rd(io_ctx, &intr, sizeof(intr)) != 0) return -1;
            build_ray_table(st->ray_table, &intr);
            st->tracker->vtbl->SetCameraIntrinsics(
                st->tracker,
                intr.fx,
                intr.cx,
                intr.cy,
                intr.k1,
                intr.k2,
                intr.k3,
                st->ray_table
            );
            Kv2MsgHeader resp = { KV2_RESP_MAGIC, 0u, 0u, 0u };
            if (wr(io_ctx, &resp, sizeof(resp)) != 0) return -1;
        } else if (req.code == KV2_CMD_PROCESS_FRAME) {
            size_t depth_bytes = KINECT_DEPTH_PIXELS * sizeof(uint16_t);
            int has_ir = (req.meta & KV2_FLAG_HAS_IR_FRAME) != 0;
            int want_bidx = (req.meta & KV2_FLAG_WANT_BODY_INDEX) != 0;
            size_t expected_len = 4u + 16u + depth_bytes + (has_ir ? depth_bytes : 0u);
            if (req.payload_len != expected_len) {
                fprintf(stderr, "[vvtechs_worker] Bad PROCESS_FRAME len=%u (expected %zu)\n", req.payload_len, expected_len);
                return -1;
            }

            uint32_t frame_index = 0;
            if (rd(io_ctx, &frame_index, sizeof(frame_index)) != 0) return -1;
            if (rd(io_ctx, st->gravity_aligned, 16u) != 0) return -1;
            if (rd(io_ctx, st->depth_512, depth_bytes) != 0) return -1;
            if (has_ir) {
                if (rd(io_ctx, st->ir_512, depth_bytes) != 0) return -1;
                downsample_ir_max_4x4(st->ir_512, st->max_ir_128);
            }

            downsample_depth_min_4x4(st->depth_512, st->min_depth_128);
            memset(st->scratch_256, 0, 0x100u);
            memset(st->body_index_512, 0xff, KINECT_DEPTH_PIXELS);

            int32_t hr1 = st->tracker->vtbl->BeginProcessFrame(
                st->tracker,
                st->depth_512,
                st->min_depth_128,
                st->max_ir_128,
                st->scratch_256,
                st->body_index_512,
                frame_index
            );
            st->context->lpVtbl->Flush(st->context);
            int32_t hr2 = (hr1 >= 0)
                ? st->tracker->vtbl->EndProcessFrame(
                    st->tracker,
                    st->body_out,
                    st->extra_1280,
                    frame_index,
                    st->gravity_aligned
                )
                : hr1;

            uint32_t status = (uint32_t)((hr1 < 0) ? hr1 : hr2);
            uint32_t out_len = (uint32_t)sizeof(NuiBodyFrameOutput) + (want_bidx ? (uint32_t)KINECT_DEPTH_PIXELS : 0u);
            Kv2MsgHeader resp = { KV2_RESP_MAGIC, status, out_len, frame_index };
            if (wr(io_ctx, &resp, sizeof(resp)) != 0) return -1;
            if (wr(io_ctx, st->body_out, sizeof(NuiBodyFrameOutput)) != 0) return -1;
            if (want_bidx) {
                if (wr(io_ctx, st->body_index_512, KINECT_DEPTH_PIXELS) != 0) return -1;
            }
        } else {
            fprintf(stderr, "[vvtechs_worker] Unknown command: %u\n", req.code);
            return -1;
        }
    }
}

int main(int argc, char** argv) {
    wchar_t dll_dir_buf[MAX_PATH * 2];
    const char* dll_dir_arg = NULL;
    int use_stdio = 1;
    int tcp_port = 0;
    int use_warp = 0;
    int vulkan_f16_fix = -1; /* -1 = auto (enabled on Wine/DXVK, disabled on native Windows D3D11) */

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--dll-dir") == 0 && i + 1 < argc) {
            dll_dir_arg = argv[++i];
        } else if (strcmp(argv[i], "--stdio") == 0) {
            use_stdio = 1;
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            tcp_port = atoi(argv[++i]);
            use_stdio = 0;
        } else if (strcmp(argv[i], "--warp") == 0) {
            use_warp = 1;
        } else if (strcmp(argv[i], "--vulkan-f16-fix") == 0) {
            vulkan_f16_fix = 1;
        } else if (strcmp(argv[i], "--no-vulkan-f16-fix") == 0) {
            vulkan_f16_fix = 0;
        }
    }

    resolve_dll_dir(dll_dir_arg, dll_dir_buf, MAX_PATH * 2);
    const wchar_t* dll_dir = (dll_dir_buf[0] != 0) ? dll_dir_buf : NULL;

    WorkerState st;
    if (init_worker(&st, dll_dir, use_warp, vulkan_f16_fix) != 0) {
        return 2;
    }

    int rc = 0;
    if (use_stdio) {
        StdioHandles io;
        io.h_in = GetStdHandle(STD_INPUT_HANDLE);
        io.h_out = GetStdHandle(STD_OUTPUT_HANDLE);
        if (io.h_in == INVALID_HANDLE_VALUE || io.h_out == INVALID_HANDLE_VALUE || io.h_in == NULL || io.h_out == NULL) {
            fprintf(stderr, "[vvtechs_worker] Invalid stdio handles\n");
            shutdown_worker(&st);
            return 4;
        }
        rc = serve_stream(&st, stdio_read_exact, stdio_write_exact, &io);
    } else {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        SOCKET srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        int opt = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons((uint16_t)tcp_port);
        if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) != 0 || listen(srv, 1) != 0) {
            fprintf(stderr, "[vvtechs_worker] TCP bind/listen failed on 127.0.0.1:%d\n", tcp_port);
            closesocket(srv);
            WSACleanup();
            shutdown_worker(&st);
            return 3;
        }
        fprintf(stderr, "[vvtechs_worker] Listening on 127.0.0.1:%d\n", tcp_port);
        fflush(stderr);
        SOCKET cli = accept(srv, NULL, NULL);
        if (cli != INVALID_SOCKET) {
            setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, (const char*)&opt, sizeof(opt));
            rc = serve_stream(&st, sock_read_exact, sock_write_exact, &cli);
            closesocket(cli);
        }
        closesocket(srv);
        WSACleanup();
    }

    shutdown_worker(&st);
    return (rc == 0) ? 0 : 1;
}
