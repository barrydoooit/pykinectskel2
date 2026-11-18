#ifndef KINECTV2_VVTECHS_ABI_H
#define KINECTV2_VVTECHS_ABI_H

#include <stdint.h>
#include <stddef.h>

#ifdef _WIN32
#define VV_CALL __cdecl
#else
#define VV_CALL __attribute__((ms_abi))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define KINECT_DEPTH_WIDTH  512
#define KINECT_DEPTH_HEIGHT 424
#define KINECT_DEPTH_PIXELS (KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT)
#define KINECT_DS_WIDTH     128
#define KINECT_DS_HEIGHT    106
#define KINECT_DS_PIXELS    (KINECT_DS_WIDTH * KINECT_DS_HEIGHT)
#define KINECT_BODY_COUNT   6
#define KINECT_JOINT_COUNT  25

/*
 * Reverse-engineered binary layout of Xbox::Kinect::VisionTechs::BodyTracking
 * output record (0x410 = 1040 bytes per body, verified at vvtechs.dll RVA 0x14c0..0x17a0).
 */
typedef struct NuiBodyData {
    float    joint_positions[KINECT_JOINT_COUNT][4];      /* +0x000 (0x190 bytes): CameraSpace (x, y, z, w) in meters */
    float    joint_orientations[KINECT_JOINT_COUNT][4];   /* +0x190 (0x190 bytes): Quaternion (x, y, z, w) */
    uint32_t joint_tracking_states[KINECT_JOINT_COUNT];   /* +0x320 (0x064 bytes): 0=NotTracked, 1=Inferred, 2=Tracked */
    uint8_t  reserved_0[0x6c];                            /* +0x384 (0x06c bytes) */
    uint64_t tracking_id;                                 /* +0x3f0 (0x008 bytes) */
    uint32_t is_tracked;                                  /* +0x3f8 (0x004 bytes): 0 or 1 */
    uint8_t  reserved_1[0x10];                            /* +0x3fc (0x010 bytes) */
    uint32_t clipped_edges;                               /* +0x40c (0x004 bytes): FrameEdges bitmask */
} NuiBodyData;

/*
 * Complete 6-body output buffer written by IBodyTracker::EndProcessFrame
 * (0x1890 = 6288 bytes, 16-byte aligned, verified at vvtechs.dll RVA 0x1b50..0x1d00).
 */
typedef struct NuiBodyFrameOutput {
    float       floor_clip_plane[4];                      /* +0x000: (x, y, z, w) */
    float       up_vector[4];                             /* +0x010: gravity / up vector */
    NuiBodyData bodies[KINECT_BODY_COUNT];                /* +0x020: 6 * 0x410 = 0x1860 bytes */
    uint8_t     trailing_padding[0x10];                   /* +0x1880..0x1890 */
} NuiBodyFrameOutput;

typedef char assert_nui_body_data_size[(sizeof(NuiBodyData) == 0x410) ? 1 : -1];
typedef char assert_nui_body_frame_size[(sizeof(NuiBodyFrameOutput) == 0x1890) ? 1 : -1];

typedef struct IBodyTracker IBodyTracker;
typedef struct IVisionTechsFactory IVisionTechsFactory;

/*
 * VTable of Xbox::Kinect::VisionTechs::BodyTracking (RVA 0xbc118 in vvtechs.dll)
 * Note: Derived from Xbox::Kinect::RefCount (int32_t ref_count at +0x08; slot[0] is scalar deleting dtor).
 */
typedef struct IBodyTrackerVtbl {
    void*    (VV_CALL *ScalarDeletingDtor)(IBodyTracker* self, uint32_t flags);
    int32_t  (VV_CALL *BeginProcessFrame)(
        IBodyTracker* self,
        const uint16_t* depth_512x424,
        const uint16_t* min_depth_128x106,
        const uint16_t* max_ir_128x106,
        void* scratch_256b,
        uint8_t* out_body_index_512x424,
        uint32_t frame_index
    );
    int32_t  (VV_CALL *EndProcessFrame)(
        IBodyTracker* self,
        NuiBodyFrameOutput* out_body_frame_6288b,
        void* out_optional_extra_1280b,
        uint32_t frame_index,
        const float gravity_vec4[4]
    );
    int32_t  (VV_CALL *ResetTrackingState)(IBodyTracker* self, const void* optional_seed);
    void     (VV_CALL *SetCameraIntrinsics)(
        IBodyTracker* self,
        float focal_length_x,
        float principal_point_x,
        float principal_point_y,
        float radial_k1,
        float radial_k2,
        float radial_k3,
        const float* ray_table_512x424x2
    );
    void     (VV_CALL *ClearCameraIntrinsics)(IBodyTracker* self);
    void     (VV_CALL *Reset)(IBodyTracker* self);
    uint32_t (VV_CALL *GetLastStatus)(IBodyTracker* self);
    void     (VV_CALL *SetDebugOption)(IBodyTracker* self, void* arg);
} IBodyTrackerVtbl;

struct IBodyTracker {
    const IBodyTrackerVtbl* vtbl;
    volatile long           ref_count;
};

/*
 * VTable of Xbox::Kinect::VisionTechs::VisionTechsFactory (RVA 0xbc368 in vvtechs.dll)
 */
typedef struct IVisionTechsFactoryVtbl {
    void*   (VV_CALL *ScalarDeletingDtor)(IVisionTechsFactory* self, uint32_t flags);
    int32_t (VV_CALL *CreateDepthIrEngine)(
        IVisionTechsFactory* self,
        void** pp_engine,
        void* d3d11_device,
        void* d3d11_context,
        uint32_t gpu_mode,
        float param
    );
    int32_t (VV_CALL *CreateBodyTracker)(
        IVisionTechsFactory* self,
        IBodyTracker** pp_tracker,
        void* d3d11_device,
        void* d3d11_context,
        uint64_t debug_flags,
        void* gpu_info_0,
        void* gpu_info_1,
        void* reserved_null
    );
    int32_t (VV_CALL *CreateHandDetector)(IVisionTechsFactory* self, void** pp_hand, uint64_t debug_flags);
    int32_t (VV_CALL *CreateLeanDetector)(IVisionTechsFactory* self, void** pp_lean);
    int32_t (VV_CALL *CreateInteractionsTracker)(IVisionTechsFactory* self, void** pp_inter, void* hand_chooser, uint64_t debug_flags);
    int32_t (VV_CALL *CreateTrackedUserManager)(IVisionTechsFactory* self, void** pp_mgr);
    int32_t (VV_CALL *CreateSystemGestureDetector)(IVisionTechsFactory* self, void** pp_sys_gest, uint64_t debug_flags, const uint16_t* dll_dir);
    int32_t (VV_CALL *CreateLogger)(IVisionTechsFactory* self, void** pp_logger);
} IVisionTechsFactoryVtbl;

struct IVisionTechsFactory {
    const IVisionTechsFactoryVtbl* vtbl;
    volatile long                  ref_count;
};

typedef int32_t (VV_CALL *PFN_NuiVisionCreateFactory)(IVisionTechsFactory** pp_factory);

#ifdef __cplusplus
}
#endif

#endif /* KINECTV2_VVTECHS_ABI_H */
