#ifndef ASSERT_H__
#define ASSERT_H__    1

#define __MODULE_NAME__ "ASSERT"

#include "trace.h"

#define ASSERT(x)   {   if (!(x)) {TRACE_printf("ASSERT", TRACE_LEVEL_FATAL, "%s[%d] : ASSERTED(%s)\n", __func__, __LINE__, #x); while(1);}}

#undef  __MODULE_NAME__

#endif
