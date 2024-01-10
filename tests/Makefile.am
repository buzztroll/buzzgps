TESTS = basic_tests
check_PROGRAMS = basic_tests
basic_tests_SOURCES = basic_tests.c $(top_srcdir)/src/buzz_gps.h
basic_tests_LDADD = $(top_builddir)/src/libbuzzgps.a -lcmocka
basic_tests_CFLAGS = -I$(top_srcdir)/src/ $(CFLAGS)

