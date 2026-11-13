/* platform.h for the SoftFloat sources in this directory, as Xemu builds them
 * (one compilation unit, targets/mega65/mcp65_softfloat.c). SoftFloat expects
 * each port to bring its own; this one is Xemu's, not SoftFloat's. It makes
 * the same choices as SoftFloat's build/Linux-ARM-VFPv2-GCC, which is what
 * the M65-MCP's model is built and verified with: portable C, no 128-bit
 * integers assumed. */
#ifndef xemu_mcp65_softfloat_platform_h
#define xemu_mcp65_softfloat_platform_h

#define LITTLEENDIAN 1
#define INLINE static inline
#define SOFTFLOAT_BUILTIN_CLZ 1
#define SOFTFLOAT_ROUND_ODD
#define INLINE_LEVEL 5
#include "opts-GCC.h"

#endif
