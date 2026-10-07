/* Rx Twister - C API for bindings (C#, Python ctypes, Unity / Unreal plugins, GUIs). */
#ifndef RX_TWISTER_C_API_H
#define RX_TWISTER_C_API_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define RX_API __declspec(dllexport)
#else
#define RX_API __attribute__((visibility("default")))
#endif

typedef void (*rx_progress_fn)(const char* stage, double fraction, void* user);

/* Runs a conversion. `args` uses exactly the command line syntax of the rxtwister tool, e.g.
 * "--max-texture 2048 --unsubdivide 1". Returns 0 on success; on failure `error` receives a
 * message (truncated to error_size). */
RX_API int rx_convert(const char* input, const char* output, const char* args, rx_progress_fn progress,
                      void* user, char* error, size_t error_size);

/* Writes a human readable description of a model into `out`. Returns 0 on success. */
RX_API int rx_describe(const char* input, char* out, size_t out_size);

RX_API const char* rx_version(void);

#ifdef __cplusplus
}
#endif
#endif
