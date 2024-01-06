AC_INIT([buzzgps], [1.0])


AC_CONFIG_SRCDIR([src/buzz_gps.c])
AC_CONFIG_AUX_DIR([build-aux])

AM_INIT_AUTOMAKE([-Wall -Werror  foreign])

AC_PROG_CC
AM_PROG_AR
LT_INIT

AC_CHECK_LIB(pthread, pthread_create, dummy=yes,
            AC_MSG_ERROR(posix thread support is required))

AC_CHECK_LIB(cunit, CU_add_suite, dummy=yes,
            AC_MSG_ERROR(pcunit is needed for tests))

AC_CONFIG_FILES([Makefile src/Makefile
                 tests/Makefile
                 examples/Makefile])

AC_OUTPUT
