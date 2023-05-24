#include <stdint.h>
#include <stdbool.h>

float clamp(float val, float inf, float sup) {
  float v = val < inf ? inf : val;
  return v > sup ? sup : v;
}