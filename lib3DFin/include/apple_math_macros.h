#pragma once

/*
 * Workaround for vanilla LLVM clang (conda-forge, homebrew) on macOS 27+ (with macOS SDK)
 *
 * The macOS SDK math.h gets the INFINITY/NAN macros from float.h's
 * __need_infinity_nan mechanism, which only Apple's clang float.h implements,
 * so with a vanilla LLVM clang the macros are never defined.
 * Eigen/Dense includes <complex>, which breaks
 * the build. Define them (with the same bodies as Apple's float.h) when missing.
 */

#if !defined(INFINITY)
#define INFINITY __builtin_huge_valf()
#endif

#if !defined(NAN)
#define NAN __builtin_nanf("0x7fc00000")
#endif
