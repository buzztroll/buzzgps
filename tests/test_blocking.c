#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include <buzz_gps.h>



static void test_bad_path(void **state)
{
   int rc;
   buzz_gps_handle_t gps_h;

   rc = buzz_gps_init(&gps_h, "/X/XXX/bad", 0, 0);
   assert_int_not_equal(BUZZ_GPS_SUCCESS, rc);
}
 
int main(int argc, char ** argv)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_bad_path),
    };
 
    return cmocka_run_group_tests(tests, NULL, NULL);
}

