/* Make use of ARM4 assembly optimizations */
/* #undef ARM4_ASM */
//#define ARM4_ASM 1

/* Make use of ARM5E assembly optimizations */
/* #undef ARM5E_ASM */
//#define ARM5E_ASM 1

/* Make use of Blackfin assembly optimizations */
/* #undef BFIN_ASM */

/* Disable all parts of the API that are using floats */
/* #undef DISABLE_FLOAT_API */
#define DISABLE_FLOAT_API 1

/* Symbol visibility prefix */
#define EXPORT __attribute__((visibility("default")))

/* Debug fixed-point implementation */
/* #undef FIXED_DEBUG */

/* Compile as fixed-point */
#define FIXED_POINT 1

/* Compile as floating-point */
//#define FLOATING_POINT /**/

/* Resample with full SINC table (no interpolation) */
#define RESAMPLE_FULL_SINC_TABLE 1  /* lower mem usage when commented out */

/* Enable support for TI C55X DSP */
/* #undef TI_C55X */

/* Use FFTW3 for FFT */
//#define USE_GPL_FFTW3 1

/* Use Intel Math Kernel Library for FFT */
/* #undef USE_INTEL_MKL */

/* Use KISS Fast Fourier Transform */
#define USE_KISS_FFT 1

/* Enable NEON support */
//#define USE_NEON /**/

/* Use FFT from OggVorbis */
//#define USE_SMALLFT 1

/* Enable SSE support */
/* #undef USE_SSE */

/* Enable SSE2 support */
/* #undef USE_SSE2 */

/* Use C99 variable-size arrays */
#define VAR_ARRAYS /**/
