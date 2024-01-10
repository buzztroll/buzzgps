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
   pthread_cond_t cond;
   pthread_mutex_t mutex;

   int raw_received;
   int event_received;
   buzz_gps_raw_event_t raw;
   buzz_gps_event_t event;
} test_fifo_obj_t;


static int test_setup (void** state)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) calloc(1, sizeof(test_fifo_obj_t));

   getcwd(test_state->fifo_path, sizeof(test_state->fifo_path));
   strcat(test_state->fifo_path, "/gps_fifo");
   printf("%s\n", test_state->fifo_path);

   mkfifo(test_state->fifo_path, 0666);

   pthread_cond_init(&test_state->cond, NULL);
   pthread_mutex_init(&test_state->mutex, NULL);
   test_state->event_received = 0;
   test_state->raw_received = 0;


   *state = test_state;
   return 0;
}

/**
 * This is run once after one given test
 */
static int test_teardown (void** state)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) *state;

   pthread_cond_destroy(&test_state->cond);
   pthread_mutex_destroy(&test_state->mutex);

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


/*
 * asyn tests
 */
static void raw_cb(buzz_gps_raw_event_t * raw, void * user_arg)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) user_arg;

   pthread_mutex_lock(&test_state->mutex);
   {
      test_state->raw = *raw;
      test_state->raw_received = 1;
      pthread_cond_signal(&test_state->cond);
   }
   pthread_mutex_unlock(&test_state->mutex);

   printf("Got raw event: %s\n", raw->sentence);
}


static void event_cb(buzz_gps_event_t * event, void * user_arg)
{
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) user_arg;

   printf("got event\n");
   if (event->location != NULL)
   {
      pthread_mutex_lock(&test_state->mutex);
      {
         test_state->event = *event;
         test_state->event_received = 1;
         pthread_cond_signal(&test_state->cond);
      }
      pthread_mutex_unlock(&test_state->mutex);
   
      printf("Got location %f %f\n", event->location->lattitude, event->location->longitude);
   }
}


static void test_simple_async(void **state)
{
   int rc;
   buzz_gps_handle_t gps_h;
   test_fifo_obj_t * test_state = (test_fifo_obj_t *) *state;
   FILE * source_pipe;
   buzz_gps_location_t test_location;

   rc = buzz_gps_init(&gps_h, test_state->fifo_path, 0, BUZZ_GPS_OPTIONS_DEBUG);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);

   rc = buzz_gps_get_last_known_location(gps_h, &test_location);
   assert_int_not_equal(BUZZ_GPS_SUCCESS, rc);

   rc = buzz_gps_start(
      gps_h, 1, 1, raw_cb, event_cb, test_state);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);

   /* starting it twice should give an error */
   rc = buzz_gps_start(
      gps_h, 1, 1, raw_cb, event_cb, test_state);
   assert_int_not_equal(BUZZ_GPS_SUCCESS, rc);

   source_pipe = fopen(test_state->fifo_path, "w");
   fprintf(source_pipe, "$GPRMC,171552.935,V,3854.825,N,07702.466,W,70.5,2.50,021116,,E*4A\r\n");
   fclose(source_pipe);

   pthread_mutex_lock(&test_state->mutex);
   {
      while(!test_state->event_received || !test_state->raw_received)
      {
         pthread_cond_wait(&test_state->cond, &test_state->mutex);
      }
   }
   pthread_mutex_unlock(&test_state->mutex);

   assert_int_equal(BUZZ_GPRMC, test_state->raw.type);
   assert_int_equal(12, test_state->raw.word_count);
   assert_ptr_not_equal(NULL, test_state->event.location);

   rc = buzz_gps_get_last_known_location(gps_h, &test_location);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   assert_float_equal(test_location.lattitude, test_state->event.location->lattitude, 0.0);
   assert_float_equal(test_location.longitude, test_state->event.location->longitude, 0.0);

   /*  should fail a destroy until we stop */
   rc = buzz_gps_destroy(gps_h);
   assert_int_not_equal(BUZZ_GPS_SUCCESS, rc);

   rc = buzz_gps_stop(gps_h);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc);
   rc = buzz_gps_destroy(gps_h);
   assert_int_equal(BUZZ_GPS_SUCCESS, rc); 
}

 
int main(int argc, char ** argv)
{
    const struct CMUnitTest tests[] =
    {
        cmocka_unit_test_setup_teardown(test_init_destroy, test_setup, test_teardown),
        cmocka_unit_test_setup_teardown(test_simple_async, test_setup, test_teardown),
        cmocka_unit_test_setup_teardown(test_simple_rmc_gll, test_setup, test_teardown),
        cmocka_unit_test_setup_teardown(test_bad_path, test_setup, test_teardown),
    };
 
    return cmocka_run_group_tests(tests, NULL, NULL);
}
