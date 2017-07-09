/*
 * trig_fix.h: Fixed-point cos() and sin() functions, based on a sixth
 * degree polynomial approximation.
 *
 * argument is in units of 2*M_PI/2^16.
 * result is in units of 1/2^14 (range = [-2^14 : 2^14]).
 *
 * The cosine function uses an even-polynomial approximation of
 * cos(M_PI/2*x) for x in [0:1], and symmetries when x is outside [0:1].
 * Sine is defined as sin(x) = cos(3*M_PI/2+x).
 *
 * Source: https://www.logre.eu/wiki/Trigonom%C3%A9trie_en_virgule_fixe
 */
 
#ifndef __trig_fix
#define __trig_fix
 
#include <stdint.h>
 
#ifdef __cplusplus
extern "C" {
#endif
 
#define M2_PI 6.283185307179586476925286
 
#define TO_FIXED(v) ((uint16_t) (0xFFFF * (v / M2_PI)))
#define TO_FIXED_DEG(v) ((uint16_t) (0xFFFF * (v / 360.0)))
#define FROM_FIXED(v) (v / (float)0x4000)

	int16_t _cos_fix(uint16_t x);
	uint16_t _atan_fix(uint16_t x);
	uint16_t _atan2_fix(int16_t y, int16_t x);
 
	/*
	 * Sixth degree polynomial:
	 *      cos(M_PI/2*x) ~ (1 - x^2)*(1 - x^2*(0.23352 - 0.019531*x^2))
	 * for x in [0:1]. Max error = 9.53e-5
	 */
	static inline float cos_fix(float x) { return FROM_FIXED(_cos_fix(TO_FIXED(x))); }
 
	/*
	 * Fixed point sin().
	 */
	static inline float sin_fix(float x) { return FROM_FIXED(_cos_fix(0xc000 + TO_FIXED(x))); }
	static inline int16_t _sin_fix(float x) { return _cos_fix(0xc000 + TO_FIXED(x)); }

	static inline float atan2_fix(int16_t y, int16_t x) { return _atan2_fix(y, x) / (0xFFFF / M2_PI); }

 
#ifdef __cplusplus
}
#endif
#endif