#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    float clamp(float val, float inf, float sup);
    float normalize(float val, float min, float max);

#ifdef __cplusplus
}
#endif