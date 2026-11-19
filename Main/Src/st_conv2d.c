#include "st_conv2d.h"

STConv2D_Config_t STConv2D_DefaultConfig(void) {
	STConv2D_Config_t cfg;
	cfg.spatial_kernel[0] = -1.0f;
	cfg.spatial_kernel[1] =  2.0f;
	cfg.spatial_kernel[2] = -1.0f;
	cfg.temp_kernel[0] = 7.0f;
	cfg.temp_kernel[1] = 2.0f;
	cfg.temp_kernel[2] = 1.0f;
	cfg.temp_scale = 0.1f;
	return cfg;
}

void STConv2D_Reset(uint8_t *primed) {
	if (primed) *primed = 0;
}

void STConv2D_Update(const STConv2D_Config_t *cfg,
                     const float *in_padded, uint16_t n_inner,
                     float (*hist)[STCONV_TEMP_LEN], uint8_t *primed,
                     int16_t *out_conv) {
	const float sk0 = cfg->spatial_kernel[0];
	const float sk1 = cfg->spatial_kernel[1];
	const float sk2 = cfg->spatial_kernel[2];
	const float tk0 = cfg->temp_kernel[0];
	const float tk1 = cfg->temp_kernel[1];
	const float tk2 = cfg->temp_kernel[2];
	const float scale = cfg->temp_scale;
	const uint8_t is_primed = (primed && *primed) ? 1U : 0U;

	for (uint16_t i = 0; i < n_inner; i++) {
		/* Spatial valid convolution at inner index i -> in_padded[i..i+2] */
		const float cur = sk2 * in_padded[i] + sk1 * in_padded[i + 1] + sk0 * in_padded[i + 2];

		if (!is_primed) {
			hist[i][0] = cur;
			hist[i][1] = cur;
			hist[i][2] = cur;
		} else {
			hist[i][0] = hist[i][1];
			hist[i][1] = hist[i][2];
			hist[i][2] = cur;
		}

		/* Causal temporal FIR: tk0 * x[t] + tk1 * x[t-1] + tk2 * x[t-2] */
		const float filt = (tk0 * hist[i][2] + tk1 * hist[i][1] + tk2 * hist[i][0]) * scale;
		out_conv[i] = (int16_t)filt;
	}

	if (primed) *primed = 1U;
}
