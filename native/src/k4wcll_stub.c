#include <stddef.h>
#include <stdint.h>

/*
 * Zero-dependency, CRT-free clean-room stub for K4WCLL.dll.
 * Imports ZERO external DLLs so it loads unconditionally in any Wine prefix on Linux.
 */

static void zero_mem(void* dst, size_t n) {
    volatile uint8_t* p = (volatile uint8_t*)dst;
    for (size_t i = 0; i < n; ++i) {
        p[i] = 0;
    }
}

__declspec(dllexport) int32_t __cdecl EtxRegister(const void* provider_desc, void* reg_handle_out) {
    (void)provider_desc;
    if (reg_handle_out != 0) {
        zero_mem(reg_handle_out, sizeof(void*));
    }
    return 0;
}

__declspec(dllexport) int32_t __cdecl EtxUnregister(const void* provider_desc, void* reg_handle) {
    (void)provider_desc;
    (void)reg_handle;
    return 0;
}

__declspec(dllexport) void __cdecl EtxFillCommonFields_v7(void* event_data, void* buffer, uint32_t buffer_size) {
    if (event_data != 0) {
        zero_mem(event_data, 32);
    }
    if (buffer != 0 && buffer_size > 0) {
        zero_mem(buffer, buffer_size);
    }
}

__declspec(dllexport) int32_t __cdecl EtxEventWrite(
    const void* event_desc,
    const void* provider_desc,
    uint64_t reg_handle,
    uint32_t count,
    const void* data_desc
) {
    (void)event_desc;
    (void)provider_desc;
    (void)reg_handle;
    (void)count;
    (void)data_desc;
    return 0;
}

int __stdcall DllMain(void* hinst, unsigned long reason, void* reserved) {
    (void)hinst;
    (void)reason;
    (void)reserved;
    return 1;
}
