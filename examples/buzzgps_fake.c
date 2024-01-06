#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <buzz_gps.h>
#include <buzz_logging.h>

static int g_done = 0;


static void sighandler(int sig)
{
    g_done = 1;
    printf("Stopping...\n");
}


static void run_blocking(buzz_gps_handle_t gps_handle)
{
    buzz_gps_raw_event_t raw;
    buzz_gps_event_t full_event;
    int rc;

    signal(SIGINT, sighandler);

    while (!g_done)
    {
        rc = buzz_gps_get_event_blocking(gps_handle, &raw, &full_event);
        if (rc == BUZZ_GPS_SUCCESS)
        {
            if (full_event.location != NULL)
            {
                printf("%f %f\n", full_event.location->lattitude, full_event.location->longitude);
            }
            buzz_gps_free_blocking_event(&full_event);
        }
        else if (rc == BUZZ_GPS_EVENT_NOT_FOUND)
        {
            printf("Unparsed event %s\n", raw.sentence);
        }
        else
        {
            fprintf(stderr, "Failed to get a location\n");
        }
    }
}


static void raw_cb(buzz_gps_raw_event_t * raw)
{
    printf("Got raw event: %s\n", raw->sentence);
}


static void event_cb(buzz_gps_event_t * event)
{
    printf("got event\n");
    if (event->location != NULL)
    {
        printf("Got location %f %f\n", event->location->lattitude, event->location->longitude);
    }
}


static void run_async(buzz_gps_handle_t gps_handle)
{
    int rc;

    rc = buzz_gps_start(
        gps_handle,
        2,
        1,
        raw_cb,
        event_cb);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        printf("ERROR\n");
        return;
    }
    while (!g_done)
    {
        sleep(1);
    }
    rc = buzz_gps_stop(gps_handle);
}


int main(int argc, char ** argv)
{
    buzz_gps_handle_t gps_handle;
    int rc;
    char * gps_device_path;
    int async = 0;

    gps_device_path = argv[1];
    if (argc > 2)
    {
        async = atoi(argv[2]);
    }

    buzz_set_log_level("ERROR");

    rc = buzz_gps_init(&gps_handle, gps_device_path, 0, BUZZ_GPS_OPTIONS_DEBUG);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        fprintf(stderr, "Failed to open device\n");
        return 1;
    }

    if (async)
    {
        run_async(gps_handle);
    }
    else
    {
        run_blocking(gps_handle);
    }

 
    buzz_gps_destroy(gps_handle);

    return 0;
}
