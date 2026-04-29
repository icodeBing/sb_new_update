

#include <necro/init.h>


#if !defined(_NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS_REAL) &&                    \
        defined(_NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS)
#if defined(__GLIBC__) && !defined(__UCLIBC__)
#define _NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS_REAL 1
#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

void necro_init(int *argcp, char *const **argvp);
int necro_main(int argc, char *const argv[]);

int necro_bootstrap_getargv(int *argc, char *const **argv);

#ifdef _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER
int _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER(int argc, char *const argv[]);
#endif

#if defined(_NECRO_INIT_HASFETCHARGV)
#define _necro_init_fetchargv(a, v) necro_init_fetchargv((a), (v))
#else
__attribute__((unused,always_inline))
static __inline__ int _necro_init_fetchargv(int *argcp, char *const **argvp);
#endif

#ifndef __bootstrap_ctor
#define __bootstrap_ctor __attribute__((constructor(220)))
#endif

static struct xbootstrap_state {
	int argc;
	char *const *argv;
} early_args;

#ifdef __cplusplus
}
#endif


int necro_bootstrap_getargv(int *argc, char *const **argv)
{
	if (early_args.argc) {
		*argc = early_args.argc;
		*argv = early_args.argv;
		return 1;
	}
	return 0;
}

static __inline__ void _necro_bootstrap_setargv(int argc, char *const *argv)
{
	early_args.argc = argc;
	early_args.argv = argv;
}

static __inline__ void call_necro_init(int *argcp, char *const **argvp)
{
#if !defined(_NECRO_BOOTSTRAP_INITFLAGS) &&                                  \
        !defined(_NECRO_BOOTSTRAP_MODNAME)

#ifdef _NECRO_BOOTSTRAP_DSO
	necro_init_dso(argcp, argvp);
#else
	necro_init(argcp, argvp);
#endif

#else
	int isDso = 0;
	unsigned long long bflags = 0;
	const char *modname = NULL;
#ifdef _NECRO_BOOTSTRAP_DSO
	isDso = 1;
#endif
#ifdef _NECRO_BOOTSTRAP_INITFLAGS
	bflags = _NECRO_BOOTSTRAP_INITFLAGS;
#endif
#ifdef _NECRO_BOOTSTRAP_MODNAME
	modname = _NECRO_BOOTSTRAP_MODNAME;
#endif
	necro_init_ext(argcp, argvp, isDso, modname, bflags);
#endif

	_necro_bootstrap_setargv(*argcp, *argvp);
}


#if _NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS_REAL == 1
__bootstrap_ctor __attribute__((cold))
static void necro_bootstrap(int argc, char *const argv[],
                                               char *const envp[])
{
	(void)envp;
	call_necro_init(&argc, &argv);
}
#else
__bootstrap_ctor __attribute__ ((cold))
static void necro_bootstrap(void)
{
	char *const *argv;
	int argc;
	if (_necro_init_fetchargv(&argc, &argv) != 0)
		return;

	call_necro_init(&argc, &argv);
}
#endif


#ifdef _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER
#ifdef _NECRO_BOOTSTRAP_DSO
#error "Main wrapper is not allowed for shared libraries"
#endif
int _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER(int argc, char *const argv[]);

int necro_main(int argc, char *const argv[])
{
#ifdef trace_me
	trace_me("necro_main entered");
#endif

	if (!necro_bootstrap_getargv(&argc, &argv)) {
		call_necro_init(&argc, &argv);
	}

#if defined(trace_me) && defined(__stringify)
	trace_me("necro_main call %s",
	         __stringify(_NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER));
#endif
	return _NECRO_BOOTSTRAP_DEFINE_MAINWRAPPER(argc, argv);
}

#ifdef _NECRO_BOOTSTRAP_WEAKREF_MAINWRAPPER
int _NECRO_BOOTSTRAP_WEAKREF_MAINWRAPPER(int argc, char *const argv[])
        __attribute__((alias("necro_main"), weak));
#endif 
#endif 


#if !defined(_NECRO_INIT_HASFETCHARGV) &&                                    \
        (!defined(_NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS_REAL) ||               \
         _NECRO_BOOTSTRAP_GLIBC_CONSTRUCTORS_REAL != 1)
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>


#ifdef _NECRO_BOOTSTRAP_WRAP_MALLOC
#define _WRM(x) __STD(x)
#else
#define _WRM(x) x
#endif


__attribute__((cold)) int _necro_init_fetchargv(int *argcp, char *const **argvp)
{
	char *arglist, *argend, *p, **v;
	ssize_t len, ret;
	int fd, n;

	len = 1024;

	for (;;) {
		fd = __STD(open("/proc/self/cmdline", O_RDONLY));
		if (fd < 0)
			return -1;

		arglist = (char*)_WRM(malloc(len));
		if (arglist == NULL) {
			__STD(close(fd));
			return -1;
		}

		ret = __STD(read(fd, arglist, len));
		__STD(close(fd));

		if (ret < 0) {
			__STD(free(arglist));
			return -1;
		}

		if (ret < len)
			break;

		__STD(free(arglist));
		len <<= 1;
	}

	argend = arglist + ret;
	p = arglist;
	n = 0;
	while (p < argend) {
		n++;
		p += strlen(p) + 1;
	}

	v = (char**)_WRM(malloc((n + 1) * sizeof(char *)));
	if (v == NULL) {
		__STD(free(arglist));
		return -1;
	}

	p = arglist;
	n = 0;
	while (p < argend) {
		v[n++] = p;
		p += strlen(p) + 1;
	}

	v[n] = NULL;
	*argcp = n;
	*argvp = v;
	return 0;
}
#endif
