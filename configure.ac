AC_INIT([buzzgps], [1.0])


AC_CONFIG_SRCDIR([src/buzz_gps.c])
AC_CONFIG_AUX_DIR([build-aux])

AM_INIT_AUTOMAKE([-Wall -Werror  foreign])

AC_PROG_CC
AM_PROG_AR
LT_INIT

AC_ARG_ENABLE(coverage,
            [AS_HELP_STRING([--enable-coverage],
              [enable coverage reporting @<:@default=no@:>@])],
            enable_coverage="yes",
            enable_coverage="no")

AC_ARG_ENABLE(coverage,
    [AS_HELP_STRING([--enable-debug],
            [include debug symbols @<:@default=no@:>@])],
            enable_debug="yes",
            enable_debug="no")

AC_CHECK_LIB(pthread, pthread_create, dummy=yes,
            AC_MSG_ERROR(posix thread support is required))


AM_CONDITIONAL([ENABLE_COVERAGE], [test "x$enable_coverage" = "xyes"])

AC_CHECK_LIB(cmocka, _cmocka_run_group_tests, dummy=yes,
            AC_MSG_ERROR(cmocka is needed for tests))

AS_IF([test "x$enable_debug" = "xyes"],
        [CFLAGS="$CFLAGS  -g"])


AS_IF([test "x$enable_coverage" = "xyes"],
        [CFLAGS="$CFLAGS -ftest-coverage -g -fprofile-arcs"])


AC_CONFIG_FILES([Makefile src/Makefile
                 tests/Makefile
                 examples/Makefile])

AC_OUTPUT
