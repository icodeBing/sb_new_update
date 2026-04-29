#ifdef main
#undef main
#endif
#define _NECRO_BOOTSTRAP_WRAP_MALLOC
#define _NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS
#define _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER __real_main
#define _NECRO_BOOTSTRAP_WEAKREF_MAINWRAPPER main
#include "necro_bootstrap_template.h"
