#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sqlite3.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

#include "buzz_gps.h"
#include "buzz_logging.h"

#define BUZZ_GPS_MAX_LINE 128
#define BUZZ_GPS_MAX_PARSE_WORDS 32

typedef int (*buzz_gps_parse_raw_func_t)(buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event);


typedef struct buzz_i_gps_handle_s {
    int serial_port;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    pthread_t thread_id;

    buzz_gps_location_t * last_location;
    buzz_gps_speed_t * last_speed;
    buzz_gps_altitude_t * last_altitude;

    int running;

    int error_interval;
    int interval_time;

    buzz_gps_raw_event_callback_t raw_cb;
    buzz_gps_event_callback_t event_cb;
    void * user_arg;
} buzz_i_gps_handle_t;


typedef struct nmea_i_parser_s {
    buzz_sentence_type_t type;
    buzz_gps_parse_raw_func_t parser_func;
    int valid;
 } nmea_i_parser_t;


nmea_i_parser_t g_nmea_sentence_map[BUZZ_GPS_TYPE_COUNT];
int g_nmea_sentence_map_initialized = 0;

static int buzz_l_parse_gprmc(buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event);

static int buzz_l_parse_gpgll(buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event);

static int buzz_l_get_raw_event(buzz_gps_handle_t gps_handle, buzz_gps_raw_event_t * raw_event);

static int buzz_l_get_full_event(
    buzz_gps_handle_t gps_handle,
    buzz_gps_raw_event_t * raw_event,
    buzz_gps_event_t * out_event);

/*
 * Each entry is a description of where the lattitude/longitude information is in the word list
 * The array is null terminated
 */
 static int buzz_l_initialize_map()
 {
    memset(&g_nmea_sentence_map, '\0', sizeof(nmea_i_parser_t)*BUZZ_GPS_TYPE_COUNT);

    g_nmea_sentence_map[BUZZ_GPRMC].type = BUZZ_GPRMC;
    g_nmea_sentence_map[BUZZ_GPRMC].valid = 1;
    g_nmea_sentence_map[BUZZ_GPRMC].parser_func = buzz_l_parse_gprmc;

    g_nmea_sentence_map[BUZZ_GPGLL].type = BUZZ_GPGLL;
    g_nmea_sentence_map[BUZZ_GPGLL].valid = 1;
    g_nmea_sentence_map[BUZZ_GPGLL].parser_func = buzz_l_parse_gpgll;

    g_nmea_sentence_map_initialized = 1;

    return 0;
 }


static int buzz_l_nmea_get_string_type(char * strtype)
{
    static char * type_names[] = 
    {
        "$GPGGA",
        "$GPGLL",
        "$GPVTG",
        "$GPRMC",
        "$GPGSA",
        "$GPGSV",
        "$GPMSS",
        "$GPTRF",
        "$GPSTN",
        "$GPXTE",
        "$GPZDA",
        NULL
    };

    for (int i = 0; type_names[i] != NULL; i++)
    {
        if(strcmp(strtype, type_names[i]) == 0)
        {
            return i;
        }
    }
    return -1;
}


static int buzz_l_read_until_char(int fd, char * buf, size_t max_size, char term_char)
{
    int done = 0;
    int n;
    size_t ndx = 0;

    while(!done)
    {
        n = read(fd, &buf[ndx], 1);
        if (n < 0) {
            buzz_logger(BUZZ_ERROR, "GPS error returned when reading the serial port: %s", strerror(errno));
            return BUZZ_GPS_NOT_FOUND;
        }
        if (buf[ndx] == term_char) {
            done = 1;
        }
        ndx = ndx + n;
        if (ndx >= max_size) {
            buzz_logger(BUZZ_WARN, "exceeded the max size of %d", max_size);
            return BUZZ_GPS_NOT_FOUND;
        }
    }
    return ndx;
}


static int buzz_l_daysmins_to_float(const char * daysmin, char hem, float * out_v)
{
    static const char * neg_hem = "sSwW";

    float sign = 1.0f;
    int rc;
    float raw;
    int day;
    float min;
    float v;

    buzz_logger(BUZZ_DEBUG, "converting %s %c", daysmin, hem);
    if (NULL != strchr(neg_hem, hem))
    {
        sign = -1.0f;
    }
    rc = sscanf(daysmin, "%f", &raw);
    if (rc <= 0)
    {
        return BUZZ_GPS_ERROR;
    }
    day = raw / 100;
    min = (raw - (day * 100.0)) / 60.0;
    v = min + day;

    *out_v = v * sign;;

    return BUZZ_GPS_SUCCESS;
}


static int buzz_l_read_sentence(
    buzz_gps_handle_t gps_handle,
    char * buffer,
    size_t buf_len)
{
    int rc;
    int char_count = 0;
    int n;

    memset(buffer, '\0', buf_len);

    /* read until we find the first character of a sentence */
    while (buffer[0] != '$')
    {
        n = read(gps_handle->serial_port, buffer, 1);
        if (n < 0)
        {
            buzz_logger(BUZZ_ERROR, "GPS error returned when reading the serial port: %s", strerror(errno));
            return BUZZ_GPS_NOT_FOUND;
        }
        char_count++;
        if (char_count > BUZZ_GPS_MAX_LINE)
        {
            buzz_logger(BUZZ_ERROR, "Read %d bytes before finding a $", char_count);
            return BUZZ_GPS_NOT_FOUND;
        }
    }

    rc = buzz_l_read_until_char(gps_handle->serial_port, &buffer[1], BUZZ_GPS_MAX_LINE-1, '\n');
    if (rc < 0)
    {
        buzz_logger(BUZZ_ERROR, "Did not find a CR in %d chars", BUZZ_GPS_MAX_LINE);
        return rc;
    }
 
    return BUZZ_GPS_SUCCESS;
}


/* We avoid the need for heap memory by spliting the sentence in place by overwriting ',' with '\0' */
static int buzz_l_split_sentence(char * buffer, size_t buf_len, char ** out_words, size_t word_count)
{
    int current_word = 0;
    char * ptr;

    ptr = buffer;
    out_words[current_word] = ptr;
    current_word++;
    ptr = strchr(ptr, ',');
    while(ptr != NULL && current_word < word_count)
    {
        *ptr = '\0';
        ptr++;
        out_words[current_word] = ptr;
        ptr = strchr(ptr, ',');
        current_word++; 
    }

    return current_word;
}


static buzz_gps_location_t * buzz_l_parse_out_location(
    char ** words,
    const int word_count,
    const int parsed_word_count,
    const int lat_ndx,
    const int lon_ndx,
    const int lat_hem_ndx,
    const int lon_hem_ndx)
{
    int rc;
    buzz_gps_location_t * location;
    float lat;
    float lon;

    if (parsed_word_count < word_count)
    {
        buzz_logger(BUZZ_ERROR, "bad word count %d %d", word_count, parsed_word_count);
        return NULL;
    }
    rc = buzz_l_daysmins_to_float(words[lat_ndx], *words[lat_hem_ndx], &lat);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        buzz_logger(BUZZ_ERROR, "Failed to get lat: %s %c", words[lat_ndx], *words[lat_hem_ndx]);
        return NULL;
    }
    rc = buzz_l_daysmins_to_float(words[lon_ndx], *words[lon_hem_ndx], &lon);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        buzz_logger(BUZZ_ERROR, "Failed to get lon: %s %c", words[lat_ndx], *words[lat_hem_ndx]);
        return NULL;
    }

    location = (buzz_gps_location_t *) calloc(1, sizeof(buzz_gps_location_t));
    location->lattitude = lat;
    location->longitude = lon;

    return location;
}


static int buzz_l_parse_gprmc(buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event)
{
    buzz_gps_location_t * location;

    buzz_logger(BUZZ_INFO, "In RMC parser");
    location = buzz_l_parse_out_location(raw_event->words, 7, raw_event->word_count, 3, 5, 4, 6);
    if (location == NULL)
    {
        return BUZZ_GPS_ERROR;
    }
    out_event->location = location;
    
    return BUZZ_GPS_SUCCESS;
}


static int buzz_l_parse_gpgll(buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event)
{
    buzz_gps_location_t * location;
    
    buzz_logger(BUZZ_INFO, "In GLL parser");
    location = buzz_l_parse_out_location(raw_event->words, 5, raw_event->word_count, 1, 3, 2, 4);
    if (location == NULL)
    {
        return BUZZ_GPS_ERROR;
    }
    out_event->location = location;
    
    return BUZZ_GPS_SUCCESS;
}


/*
 * read until we get a location or an error 
 *
 * must be called locked
 */
static int buzz_l_get_events(
    buzz_gps_handle_t gps_handle,
    buzz_gps_raw_event_t * out_raw,
    buzz_gps_event_t * out_event)
{
    int rc;
    
    rc = buzz_l_get_raw_event(gps_handle, out_raw);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        buzz_logger(BUZZ_INFO, "Error getting raw sentence");
        return BUZZ_GPS_RAW_SENTENCE;
    }
    buzz_logger(BUZZ_DEBUG, "Found event type %d", out_raw->type);
    memset(out_event, '\0', sizeof(buzz_gps_event_t));
    rc = buzz_l_get_full_event(gps_handle, out_raw, out_event);

    return rc;
}


static int buzz_l_get_raw_event(buzz_gps_handle_t gps_handle, buzz_gps_raw_event_t * raw_event)
{
    int rc;

    buzz_logger(BUZZ_DEBUG, "Reading a sentence from the GPS device...");
    rc = buzz_l_read_sentence(gps_handle, raw_event->buffer, BUZZ_GPS_MAX_LINE);
    if (rc != BUZZ_GPS_SUCCESS)
    {
        return BUZZ_GPS_ERROR;
    }

    memcpy(raw_event->sentence, raw_event->buffer, BUZZ_GPS_MAX_LINE);
    buzz_logger(BUZZ_INFO, "Read the sentence: %s", raw_event->buffer);

    raw_event->word_count = buzz_l_split_sentence(raw_event->buffer, BUZZ_GPS_MAX_LINE, raw_event->words, BUZZ_GPS_MAX_PARSE_WORDS);
    if(raw_event->word_count < 1)
    {
        return BUZZ_GPS_ERROR;
    }

    buzz_logger(BUZZ_DEBUG, "Parsing sentence type %s of %d words", raw_event->words[0], raw_event->word_count);
    raw_event->type = buzz_l_nmea_get_string_type(raw_event->words[0]);

    int word_count = buzz_l_split_sentence(raw_event->buffer, BUZZ_GPS_MAX_LINE, raw_event->words, BUZZ_GPS_MAX_PARSE_WORDS);
    if(word_count < 1)
    {
        return BUZZ_GPS_ERROR;
    }

    return BUZZ_GPS_SUCCESS;
}


static int buzz_l_get_full_event(buzz_gps_handle_t gps_handle, buzz_gps_raw_event_t * raw_event, buzz_gps_event_t * out_event)
{
    nmea_i_parser_t * parser_ent;
    int rc;

    parser_ent = &g_nmea_sentence_map[raw_event->type];
    if (parser_ent->parser_func == NULL)
    {
        buzz_logger(BUZZ_INFO, "parser func is null");
        return BUZZ_GPS_EVENT_NOT_FOUND; 
    }

    rc = parser_ent->parser_func(raw_event, out_event);
    buzz_logger(BUZZ_WARN, "parser func rc is %d", rc);

    return rc;
}


static void * buzz_l_gather_thread(void * arg)
{
    buzz_gps_handle_t gps_handle = (buzz_gps_handle_t) arg;
    int rc;
    struct timespec waittime;
    buzz_gps_raw_event_t raw_event;
    buzz_gps_event_t event;

    pthread_mutex_lock(&gps_handle->mutex);
    {
        while(gps_handle->running)
        {
            rc = buzz_l_get_raw_event(gps_handle, &raw_event);
            if (rc != BUZZ_GPS_SUCCESS)
            {
                buzz_logger(BUZZ_ERROR, "failed to get a sentence");

                waittime.tv_sec = gps_handle->error_interval + time(NULL);;
                waittime.tv_nsec = 0;
            }
            else
            {
                /* for now send out the callbacks under lock */
                if (gps_handle->raw_cb != NULL)
                {
                    gps_handle->raw_cb(&raw_event, gps_handle->user_arg);
                }
                rc = buzz_l_get_full_event(gps_handle, &raw_event, &event);
                if (rc == BUZZ_GPS_SUCCESS)
                {
                    /* freeing the data here is convenient but will allow users of callback 
                    think they can use the structures beyond the life time of the callbackk. We
                    must document this */
                    if (event.location != NULL)
                    {
                        if (gps_handle->last_location != NULL)
                        {
                            free(gps_handle->last_location);     
                        }
                        gps_handle->last_location = event.location;
                    }
                    if (event.speed != NULL)
                    {
                        if (gps_handle->last_speed != NULL)
                        {
                            free(gps_handle->last_speed);
                        }
                        gps_handle->last_speed = event.speed;

                    }
                    if (event.altitude != NULL)
                    {
                        if (gps_handle->last_altitude != NULL)
                        {
                            free(gps_handle->last_altitude);
                        }
                        gps_handle->last_altitude = event.altitude;
                    }
                    if (gps_handle->event_cb != NULL)
                    {
                        gps_handle->event_cb(&event, gps_handle->user_arg);
                    }
                }
                waittime.tv_sec = gps_handle->interval_time + time(NULL);
                waittime.tv_nsec = 0;
            }

            pthread_cond_timedwait(&gps_handle->cond, &gps_handle->mutex, &waittime);
        }
    }
    pthread_mutex_unlock(&gps_handle->mutex);

    return NULL;
}


int buzz_gps_init(
    buzz_gps_handle_t * out_handle,
    const char * serial_path,
    speed_t baud,
    int options)
{

    buzz_i_gps_handle_t * new_handle;
    struct termios tty;

    if (!g_nmea_sentence_map_initialized)
    {
        buzz_logger(BUZZ_DEBUG, "Setting up the sentence parse in global memory");
        buzz_l_initialize_map();
    }

    buzz_logger(BUZZ_DEBUG, "Opening the serial port for bluetooth");
    new_handle = (buzz_i_gps_handle_t *) calloc(1, sizeof(buzz_i_gps_handle_t));
    new_handle->serial_port = open(serial_path, O_RDWR);
    if (new_handle->serial_port < 0)
    {
        buzz_logger(BUZZ_ERROR, "Failed to open %s: %s", serial_path, strerror(errno));
        goto error;
    }

    /* Only set options in not in debug mode */
    if ((options | BUZZ_GPS_OPTIONS_DEBUG) == 0)
    {
        buzz_logger(BUZZ_DEBUG, "Setting tty options");
        memset(&tty, 0, sizeof(tty));
        if(tcgetattr(new_handle->serial_port, &tty) != 0) {
            buzz_logger(BUZZ_ERROR, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            goto error;
        }
        cfsetospeed (&tty, baud);
    }
    
    pthread_cond_init(&new_handle->cond, NULL);
    pthread_mutex_init(&new_handle->mutex, NULL);

    *out_handle = new_handle;

    return BUZZ_GPS_SUCCESS;
error:
    free(new_handle);
    return BUZZ_GPS_ERROR;
}


int buzz_gps_destroy(buzz_gps_handle_t handle)
{
    if (handle->running)
    {
        buzz_logger(BUZZ_WARN, "Trying to destroy a running handle. Call stop first");
        return BUZZ_GPS_ERROR;
    }
    close(handle->serial_port);
    free(handle);
    pthread_cond_destroy(&handle->cond);
    pthread_mutex_destroy(&handle->mutex);
    return BUZZ_GPS_SUCCESS;
}


int buzz_gps_get_event_blocking(
    buzz_gps_handle_t gps_handle,
    buzz_gps_raw_event_t * out_raw,
    buzz_gps_event_t * out_event)
{
    int rc;

    pthread_mutex_lock(&gps_handle->mutex);
    {
        rc = buzz_l_get_events(gps_handle, out_raw, out_event);
    }
    pthread_mutex_unlock(&gps_handle->mutex);

    return rc;
}



int buzz_gps_get_last_known_location(
    buzz_gps_handle_t gps_handle, buzz_gps_location_t * out_location)
{
    int rc;

    pthread_mutex_lock(&gps_handle->mutex);
    {
        if (gps_handle->last_location == NULL)
        {
            rc = BUZZ_GPS_ERROR;
        }
        else
        {
            out_location->lattitude = gps_handle->last_location->lattitude;
            out_location->longitude = gps_handle->last_location->longitude;
            rc = BUZZ_GPS_SUCCESS;
        }
    }
    pthread_mutex_unlock(&gps_handle->mutex);

    return rc;
}


int buzz_gps_start(buzz_gps_handle_t gps_handle,
                   int interval_time,
                   int error_interval,
                   buzz_gps_raw_event_callback_t raw_cb,
                   buzz_gps_event_callback_t event_cb,
                   void * user_arg)
{
    if (gps_handle->running)
    {
        buzz_logger(BUZZ_WARN, "Attempting to start a running handle");
        return BUZZ_GPS_ERROR;
    }

    pthread_mutex_lock(&gps_handle->mutex);
    {
        gps_handle->raw_cb = raw_cb;
        gps_handle->event_cb = event_cb;
        gps_handle->user_arg = user_arg;
        gps_handle->interval_time = interval_time;
        gps_handle->error_interval = error_interval;
        gps_handle->running = 1;
        pthread_create(&gps_handle->thread_id, NULL, buzz_l_gather_thread, gps_handle);
    }
    pthread_mutex_unlock(&gps_handle->mutex);

    return BUZZ_GPS_SUCCESS;
}


int buzz_gps_stop(buzz_gps_handle_t gps_handle)
{

    pthread_mutex_lock(&gps_handle->mutex);
    {
        buzz_logger(BUZZ_INFO, "Shutting down gps thread");
        gps_handle->running = 0;
        pthread_cond_broadcast(&gps_handle->cond);
    }
    pthread_mutex_unlock(&gps_handle->mutex);

    buzz_logger(BUZZ_INFO, "waiting for the thread to end");
    pthread_join(gps_handle->thread_id, NULL);

    return BUZZ_GPS_SUCCESS;
}


int buzz_gps_location_transform(
    const char * location_str,
    const char hemisphere,
    float * out_location)
{
    int rc;

    rc = buzz_l_daysmins_to_float(location_str, hemisphere, out_location);
    return rc;
}


int buzz_gps_free_blocking_event(buzz_gps_event_t * out_event)
{
    if (out_event == NULL)
    {
        return BUZZ_GPS_SUCCESS;
    }
    if (out_event->location != NULL)
    {
        free(out_event->location);
    }
    if (out_event->speed != NULL)
    {
        free(out_event->speed);
    }
    if (out_event->altitude != NULL)
    {
        free(out_event->altitude);
    }
    return BUZZ_GPS_SUCCESS;
}
