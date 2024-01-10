#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cmocka.h>

#include <buzz_gps.h>


typedef struct test_fifo_obj_s
{
   char fifo_path[PATH_MAX];
} test_fifo_obj_t;


static int test_setup (void** state)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) calloc(1, sizeof(test_fifo_obj_t));

   getcwd(test_state->fifo_path, sizeof(test_state->fifo_path));
   strcat(test_state->fifo_path, "/gps_fifo");
   printf("%s\n", test_state->fifo_path);

   mkfifo(test_state->fifo_path, 0666);

   *state = test_state;
   return 0;
}

/**
 * This is run once after one given test
 */
static int test_teardown (void** state)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) *state;

   remove(test_state->fifo_path);

   free(test_state);
   return 0;
}


static void test_bad_path(void **state)
{
   int rc;
   buzz_gps_handle_t gps_h;

   rc = buzz_gps_init(&gps_h, "/X/XXX/bad", 0, 0);
   assert_int_not_equal(BUZZ_GPS_SUCCESS, rc);
}


static void test_init_destroy(void **state)
{
   int rc;
   buzz_gps_handle_t gps_h;
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) *state;


   rc = buzz_gps_init(&gps_h, test_state->fifo_path, 0, BUZZ_GPS_OPTIONS_DEBUG);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   rc = buzz_gps_destroy(gps_h);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
}


static void test_simple_rmc_gll(void **state)
{
   int rc;
   buzz_gps_handle_t gps_h;
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) *state;
   FILE * source_pipe;
   buzz_gps_raw_event_t raw;
   buzz_gps_event_t event;
   float tmp_float;


   rc = buzz_gps_init(&gps_h, test_state->fifo_path, 0, BUZZ_GPS_OPTIONS_DEBUG);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);

   source_pipe = fopen(test_state->fifo_path, "w");
   fprintf(source_pipe, "$GPRMC,171552.935,V,3854.825,N,07702.466,W,70.5,2.50,021116,,E*4A\r\n");
   fprintf(source_pipe, "$GPGLL,3854.777,N,07702.464,W,171848.935,V*34\r\n");
   fclose(source_pipe);

   rc = buzz_gps_get_event_blocking(gps_h, &raw, &event);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_int_equal(BUZZ_GPRMC, raw.type);
   assert_int_equal(12, raw.word_count);
   assert_ptr_not_equal(NULL, event.location);

   // Make sure location is as expected
   rc = buzz_gps_location_transform(raw.words[3], raw.words[4][0], &tmp_float);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_float_equal(tmp_float, event.location->lattitude, 0.0);
   assert_float_not_equal(0.0, event.location->lattitude, 0.0);
   rc = buzz_gps_location_transform(raw.words[5], raw.words[6][0], &tmp_float);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_float_equal(tmp_float, event.location->longitude, 0.0);
   assert_float_not_equal(0.0, event.location->longitude, 0.0);
   rc = buzz_gps_free_blocking_event(&event);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);

   rc = buzz_gps_get_event_blocking(gps_h, &raw, &event);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_int_equal(BUZZ_GPGLL, raw.type);
   assert_int_equal(7, raw.word_count);
   assert_ptr_not_equal(NULL, event.location);
   rc = buzz_gps_location_transform(raw.words[1], raw.words[2][0], &tmp_float);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_float_equal(tmp_float, event.location->lattitude, 0.0);
   assert_float_not_equal(0.0, event.location->lattitude, 0.0);
   rc = buzz_gps_location_transform(raw.words[3], raw.words[4][0], &tmp_float);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_float_equal(tmp_float, event.location->longitude, 0.0);
   assert_float_not_equal(0.0, event.location->longitude, 0.0);
   rc = buzz_gps_free_blocking_event(&event);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);

   rc = buzz_gps_destroy(gps_h);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
}

 
int main(int argc, char ** argv)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test_setup_teardown(test_init_destroy, test_setup, test_teardown),
        cmocka_unit_test_setup_teardown(test_simple_rmc_gll, test_setup, test_teardown),
        cmocka_unit_test_setup_teardown(test_bad_path, test_setup, test_teardown),
    };
 
    return cmocka_run_group_tests(tests, NULL, NULL);
}

