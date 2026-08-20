#pragma once

#if defined(__linux__)
#include <resolv.h>
#endif

#ifdef _res
#undef _res
#endif

#ifdef res
#undef res
#endif

