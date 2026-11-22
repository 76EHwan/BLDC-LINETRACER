#ifndef INC_ST_CONV2D_H_
#define INC_ST_CONV2D_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STCONV_SPATIAL_LEN 3
#define STCONV_TEMP_LEN    3

/**
 * Separable Spatio-Temporal 2D Convolution Filter for 1D Sensor Arrays.
 * Pure C module with zero HAL, CMSIS-DSP, or global state dependencies.
 */
typedef struct {
	float spatial_kernel[STCONV_SPATIAL_LEN]; /* e.g. {-1.0f, 2.0f, -1.0f} */
	float temp_kernel[STCONV_TEMP_LEN];       /* [h0=t, h1=t-1, h2=t-2], e.g. {7.0f, 2.0f, 1.0f} */
	float temp_scale;                         /* Output normalization (e.g. 0.1f) */
} STConv2D_Config_t;

/* Default kernel: spatial [-1, 2, -1] x causal temporal [7, 2, 1] * 0.1. */
STConv2D_Config_t STConv2D_DefaultConfig(void);

/* Resets temporal history priming flag. */
void STConv2D_Reset(uint8_t *primed);

/**
 * Runs 1D spatial convolution on padded input of length (n_inner + 2)
 * followed by causal 3-tap temporal FIR convolution per channel.
 *
 * in_padded: [left_outer, ch_0 .. ch_{n_inner-1}, right_outer]
 * hist:      caller-owned [n_inner][3] ring/shift buffer
 * out_conv:  filtered output [n_inner]
 */
void STConv2D_Update(const STConv2D_Config_t *cfg,
                     const float *in_padded, uint16_t n_inner,
                     float (*hist)[STCONV_TEMP_LEN], uint8_t *primed,
                     int16_t *out_conv);

#ifdef __cplusplus
}
#endif

#endif /* INC_ST_CONV2D_H_ */
