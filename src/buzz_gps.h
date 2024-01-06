#ifndef BUZZ_GPS_API_H
#define BUZZ_GPS_API_H 1

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#define BUZZ_SENTENCE_MAX_LENGTH 80
#define BUZZ_GPS_OPTIONS_NONE 0
#define BUZZ_GPS_OPTIONS_DEBUG 0x01


#define BUZZ_GPS_MAX_LINE 128
#define BUZZ_GPS_MAX_PARSE_WORDS 32

/*
 *  All supported RMEA sentence types
 */
typedef enum buzz_sentence_type_e
{
    BUZZ_GPGGA = 0, // 	Global positioning system fix data (time, position, fix type data)
    BUZZ_GPGLL, // 	Geographic position, latitude, longitude
    BUZZ_GPVTG, // 	Course and speed information relative to the ground
    BUZZ_GPRMC, // 	Time, date, position, course and speed data
    BUZZ_GPGSA, // 	GPS receiver operating mode, satellites used in the position solution, and DOP values.
    BUZZ_GPGSV, // 	The number of GPS satellites in view satellite ID numbers, elevation, azimuth and SNR values.
    BUZZ_GPMSS, // 	Signal to noise ratio, signal strength, frequency, and bit rate from a radio beacon receiver.
    BUZZ_GPTRF, // 	Transit fix data
    BUZZ_GPSTN, // 	Multiple data ID
    BUZZ_GPXTE, // 	cross track error, measured
    BUZZ_GPZDA, // 	Date and time (PPS timing message, synchronized to PPS).

    BUZZ_GPS_TYPE_COUNT
} buzz_sentence_type_t;

typedef enum buzz_gps_error_e
{
    BUZZ_GPS_SUCCESS = 0,
    BUZZ_GPS_ERROR,
    BUZZ_GPS_RAW_SENTENCE,
    BUZZ_GPS_NOT_FOUND,
    BUZZ_GPS_EVENT_NOT_FOUND
} buzz_gps_error_t;


typedef struct buzz_i_gps_handle_s * buzz_gps_handle_t;

/*
 * Parsed out raw string
 */
typedef struct buzz_gps_raw_event_s
{
    buzz_sentence_type_t type;

    char sentence[BUZZ_GPS_MAX_LINE];
    char buffer[BUZZ_GPS_MAX_LINE];
    char * words[BUZZ_GPS_MAX_PARSE_WORDS];
    int word_count;
} buzz_gps_raw_event_t;

typedef struct buzz_gps_location_s
{
    float lattitude;
    float longitude;
} buzz_gps_location_t;

typedef struct buzz_gps_speed_s
{
    float knots_per_hour;
    float direction;
} buzz_gps_speed_t;

typedef struct buzz_gps_altitude_s
{
    float altitude_meters;
} buzz_gps_altitude_t;

typedef struct buzz_gps_event_s
{
    buzz_sentence_type_t type;
    time_t time;

    buzz_gps_location_t * location;
    buzz_gps_speed_t * speed;
    buzz_gps_altitude_t * altitude;
} buzz_gps_event_t;

/*
 * Callback signature for raw sentences
 */
typedef void (*buzz_gps_raw_event_callback_t)(buzz_gps_raw_event_t * raw_event);

/*
 * Callback for parsed information
 */
typedef void (*buzz_gps_event_callback_t)(buzz_gps_event_t * event);

/*
 *  Initialize the GPS object
 *
 *  serial_path: location of the GPS device on the file system
 *  baud: The speed to read the serial GPS device
 */
int buzz_gps_init(
    buzz_gps_handle_t * out_handle, const char * serial_path, speed_t baud, int options);

/*
 * Clean up all resources associated with a GPS object
 */
int buzz_gps_destroy(buzz_gps_handle_t handle);

/*
 * Start a background thread to read GPS events as they come in. 
 */
int buzz_gps_start(buzz_gps_handle_t gps_handle,
                   int interval_time,
                   int error_interval,
                   buzz_gps_raw_event_callback_t raw_cb,
                   buzz_gps_event_callback_t event_cb);

/*
 * Stop reading GPS events 
 */
int buzz_gps_stop(buzz_gps_handle_t gps_handle);

/*
 * Get the last known GPS location
 */
int buzz_gps_get_last_known_location(
    buzz_gps_handle_t gps_handle, buzz_gps_location_t ** out_location);

/*
 * Block until a parsed event is ready. If this returns BUZZ_GPS_SUCCESS you
 * must free the memory associated with buzz_gps_event_t by using the
 * buzz_gps_free_blocking_event() function.
 * 
 *  Return code:
 *   - BUZZ_GPS_SUCCESS: raw and full event parsed
 *   - BUZZ_GPS_NOT_FOUND: raw event was processed but the full event was not parsed
 *   - BUZZ_GPS_ERROR: neither type was processed due to an error
 */
int buzz_gps_get_event_blocking(
    buzz_gps_handle_t gps_handle,
    buzz_gps_raw_event_t * out_raw,
    buzz_gps_event_t * out_event);

/*
 *  Free the memory associated with the buzz_gps_event_t which was
 *  passed back from a call to buzz_gps_get_event_blocking(). This
 *  should only be used in that case.
 */
int buzz_gps_free_blocking_event(buzz_gps_event_t * out_event);

/*
 *  Translate an NMEA location to a floating point location
 * 
 *   location_str: DDMM.MMM format
 *   hemisphere:   Compass direction <N,S,E,W>
 *   out_location: The floating point representation
 * 
 *   Returns 0 on success or non-zero on error.
 * 
 */
int buzz_gps_location_transform(
    const char * location_str,
    const char hemisphere,
    float * out_location);


#endif
