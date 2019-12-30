#include "main.h"
#include <float.h>

#define WS_PORT         7681
#define WS_INTERVAL         250
#define WS_INTERVAL_FAST    100

#define FFT_SIZE        1024
#define FFT_TIME_SMOOTH 0.999f // 0.0 - 1.0

#define AIRSPY_FREQ     745250000

#define AIRSPY_SAMPLE   10000000

#define AIRSPY_SERIAL	0x644064DC2354AACD // WB

/** LWS Vars **/
int max_poll_elements;
int debug_level = 3;
volatile int force_exit = 0;
struct lws_context *context;
uint32_t lws_count_fft = 0;
uint32_t lws_count_fft_fast = 0;
uint32_t lws_count_fft_m0dtslivetune = 0;
#define STDOUT_INTERVAL_CONNCOUNT 30*1000

pthread_t fftThread;
pthread_t wsThread;

static void sleep_ms(uint32_t _duration)
{
    struct timespec req, rem;
    req.tv_sec = _duration / 1000;
    req.tv_nsec = (_duration - (req.tv_sec*1000))*1000*1000;

    while(nanosleep(&req, &rem) == EINTR)
    {
        /* Interrupted by signal, shallow copy remaining time into request, and resume */
        req = rem;
    }
}

/** AirSpy Vars **/
struct airspy_device* device = NULL;
/* Sample type -> 32bit Complex Float */
enum airspy_sample_type sample_type_val = AIRSPY_SAMPLE_FLOAT32_IQ;
/* Sample rate */
uint32_t sample_rate_val = AIRSPY_SAMPLE;
/* DC Bias Tee -> 0 (disabled) */
uint32_t biast_val = 0;
/* Linear Gain */
#define LINEAR
uint32_t linearity_gain_val = 12; // MAX=21
/* Sensitive Gain */
//#define SENSITIVE
uint32_t sensitivity_gain_val = 10; // MAX=21
/* Frequency */
uint32_t freq_hz = AIRSPY_FREQ;

double hanning_window_const[FFT_SIZE];

int airspy_rx(airspy_transfer_t* transfer);

#define FLOAT32_EL_SIZE_BYTE (4)
fftw_complex* fft_in;
fftw_complex*   fft_out;
fftw_plan   fft_plan;

static const char *fftw_wisdom_filename = ".fftw_wisdom";

void setup_fft(void)
{
    int i;
    /* Set up FFTW */
    fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
    fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
    i = fftw_import_wisdom_from_filename(fftw_wisdom_filename);
    if(i == 0)
    {
        fprintf(stdout, "Computing plan...");
	fflush(stdout);
    }
    fft_plan = fftw_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_EXHAUSTIVE);
    if(i == 0)
    {
        fftw_export_wisdom_to_filename(fftw_wisdom_filename);
    }
}

static void close_airspy(void)
{
    int result;
    
    /* De-init AirSpy device */
    if(device != NULL)
    {
	    result = airspy_stop_rx(device);
	    if( result != AIRSPY_SUCCESS ) {
		    printf("airspy_stop_rx() failed: %s (%d)\n", airspy_error_name(result), result);
	    }

	    result = airspy_close(device);
	    if( result != AIRSPY_SUCCESS ) 
	    {
		    printf("airspy_close() failed: %s (%d)\n", airspy_error_name(result), result);
	    }
	
	    airspy_exit();
    }
}

static void close_fftw(void)
{
    /* De-init fftw */
    fftw_free(fft_in);
    fftw_free(fft_out);
    fftw_destroy_plan(fft_plan);
    fftw_forget_wisdom();
}

static uint8_t setup_airspy()
{
    int result;

    result = airspy_init();
    if( result != AIRSPY_SUCCESS ) {
	    printf("airspy_init() failed: %s (%d)\n", airspy_error_name(result), result);
	    return 0;
    }
    #ifdef AIRSPY_SERIAL
    	result = airspy_open_sn(&device, AIRSPY_SERIAL);
    #else
    	result = airspy_open(&device);
    #endif
    if( result != AIRSPY_SUCCESS ) {
	    printf("airspy_open() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_exit();
	    return 0;
    }

    result = airspy_set_sample_type(device, sample_type_val);
    if (result != AIRSPY_SUCCESS) {
	    printf("airspy_set_sample_type() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_close(device);
	    airspy_exit();
	    return 0;
    }

    result = airspy_set_samplerate(device, sample_rate_val);
    if (result != AIRSPY_SUCCESS) {
	    printf("airspy_set_samplerate() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_close(device);
	    airspy_exit();
	    return 0;
    }

    result = airspy_set_rf_bias(device, biast_val);
    if( result != AIRSPY_SUCCESS ) {
	    printf("airspy_set_rf_bias() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_close(device);
	    airspy_exit();
	    return 0;
    }

    #ifdef LINEAR
	    result =  airspy_set_linearity_gain(device, linearity_gain_val);
	    if( result != AIRSPY_SUCCESS ) {
		    printf("airspy_set_linearity_gain() failed: %s (%d)\n", airspy_error_name(result), result);
	    }
    #elif defined SENSITIVE
	    result =  airspy_set_sensitivity_gain(device, sensitivity_gain_val);
	    if( result != AIRSPY_SUCCESS ) {
		    printf("airspy_set_sensitivity_gain() failed: %s (%d)\n", airspy_error_name(result), result);
	    }
    #endif

    result = airspy_start_rx(device, airspy_rx, NULL);
    if( result != AIRSPY_SUCCESS ) {
	    printf("airspy_start_rx() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_close(device);
	    airspy_exit();
	    return 0;
    }

    result = airspy_set_freq(device, freq_hz);
    if( result != AIRSPY_SUCCESS ) {
	    printf("airspy_set_freq() failed: %s (%d)\n", airspy_error_name(result), result);
	    airspy_close(device);
	    airspy_exit();
	    return 0;
    }
    
    return 1;
}

/* transfer->sample_count is normally 65536 */
#define	AIRSPY_BUFFER_COPY_SIZE	65536

typedef struct {
	uint32_t index;
	uint32_t size;
	char data[AIRSPY_BUFFER_COPY_SIZE * FLOAT32_EL_SIZE_BYTE];
	pthread_mutex_t mutex;
	pthread_cond_t 	signal;
} rf_buffer_t;

rf_buffer_t rf_buffer = {
	.index = 0,
	.size = 0,
	.mutex = PTHREAD_MUTEX_INITIALIZER,
	.signal = PTHREAD_COND_INITIALIZER,
	.data = { 0 }
};

/* Airspy RX Callback, this is called by a new thread within libairspy */
int airspy_rx(airspy_transfer_t* transfer)
{    
    if(transfer->samples != NULL && transfer->sample_count >= AIRSPY_BUFFER_COPY_SIZE)
    {
        pthread_mutex_lock(&rf_buffer.mutex);
        rf_buffer.index = 0;
        memcpy(
            rf_buffer.data,
            transfer->samples,
            (AIRSPY_BUFFER_COPY_SIZE * FLOAT32_EL_SIZE_BYTE)
        );
        rf_buffer.size = AIRSPY_BUFFER_COPY_SIZE / (FFT_SIZE * 2);
        pthread_cond_signal(&rf_buffer.signal);
        pthread_mutex_unlock(&rf_buffer.mutex);
    }
	return 0;
}

typedef struct {
	float data[FFT_SIZE];
	pthread_mutex_t mutex;
} fft_buffer_t;

fft_buffer_t fft_buffer = {
	.mutex = PTHREAD_MUTEX_INITIALIZER,
};

/* FFT Thread */
void *thread_fft(void *dummy)
{
    (void) dummy;
    int             i, offset;
    fftw_complex    pt;
    double           pwr, lpwr;

	double pwr_scale = 1.0 / ((float)FFT_SIZE * (float)FFT_SIZE);

    while(1)
    {
    	/* Lock input buffer */
    	pthread_mutex_lock(&rf_buffer.mutex);

    	if(rf_buffer.index == rf_buffer.size)
    	{
	    	/* Wait for signalled input */
	    	pthread_cond_wait(&rf_buffer.signal, &rf_buffer.mutex);
    	}

    	offset = rf_buffer.index * FFT_SIZE * 2;

    	/* Copy data out of rf buffer into fft_input buffer */
    	for (i = 0; i < FFT_SIZE; i++)
	    {
	        fft_in[i][0] = ((float*)rf_buffer.data)[offset+(2*i)] * hanning_window_const[i];
	        fft_in[i][1] = ((float*)rf_buffer.data)[offset+(2*i)+1] * hanning_window_const[i];
	    }

	    rf_buffer.index++;

	    /* Unlock input buffer */
    	pthread_mutex_unlock(&rf_buffer.mutex);

    	/* Run FFT */
    	fftw_execute(fft_plan);

    	/* Lock output buffer */
    	pthread_mutex_lock(&fft_buffer.mutex);

    	for (i = 0; i < FFT_SIZE; i++)
	    {
	        /* shift, normalize and convert to dBFS */
	        if (i < FFT_SIZE / 2)
	        {
	            pt[0] = fft_out[FFT_SIZE / 2 + i][0] / FFT_SIZE;
	            pt[1] = fft_out[FFT_SIZE / 2 + i][1] / FFT_SIZE;
	        }
	        else
	        {
	            pt[0] = fft_out[i - FFT_SIZE / 2][0] / FFT_SIZE;
	            pt[1] = fft_out[i - FFT_SIZE / 2][1] / FFT_SIZE;
	        }
	        pwr = pwr_scale * (pt[0] * pt[0]) + (pt[1] * pt[1]);
	        lpwr = 10.f * log10(pwr + 1.0e-20);
	        
	        fft_buffer.data[i] = (lpwr * (1.f - FFT_TIME_SMOOTH)) + (fft_buffer.data[i] * FFT_TIME_SMOOTH);
	    }

	    /* Unlock output buffer */
    	pthread_mutex_unlock(&fft_buffer.mutex);
    }

}

#define WEBSOCKET_OUTPUT_LENGTH	4096
typedef struct {
	uint8_t buffer[LWS_PRE+WEBSOCKET_OUTPUT_LENGTH];
	uint32_t length;
	uint32_t sequence_id;
	pthread_mutex_t mutex;
} websocket_output_t;

websocket_output_t websocket_output = {
	.length = 0,
	.sequence_id = 0,
	.mutex = PTHREAD_MUTEX_INITIALIZER
};

websocket_output_t websocket_output_fast = {
    .length = 0,
    .sequence_id = 0,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};


/* OLD
#define FFT_OFFSET  85
#define FFT_SCALE   3000.0


#define FLOOR_TARGET    8500
#define FLOOR_TIME_SMOOTH 0.995
*/

#define FFT_PRESCALE 3.0

#define FFT_OFFSET  92
#define FFT_SCALE   (FFT_PRESCALE * 3000)


#define FLOOR_TARGET	(FFT_PRESCALE * 47000)
#define FLOOR_TIME_SMOOTH 0.995

#define FLOOR_OFFSET    (FFT_PRESCALE * 38000)

static uint32_t lowest_smooth = FLOOR_TARGET;

static uint32_t fft_output_data[FFT_SIZE];
void fft_to_buffer(websocket_output_t *_websocket_output)
{
	int32_t i, j;
    uint32_t lowest;
    int32_t offset;
	uint16_t *websocket_output_buffer_ptr;

    /* Create and append data points */
    i = 0;
    //uint32_t min = 0xFFFFFFFF, max = 0;
    //float fmin = FLT_MAX, fmax = -FLT_MAX; 

    /* Lock FFT output buffer for reading */
    pthread_mutex_lock(&fft_buffer.mutex);

    for(j=(FFT_SIZE*0.05);j<(FFT_SIZE*0.95);j++)
    {
        fft_output_data[i] = (uint32_t)(FFT_SCALE * (fft_buffer.data[j] + FFT_OFFSET)) + (FFT_PRESCALE*fft_line_compensation[j]); // (fft_line_compensation[j] / 3.0);
        //if(fft_buffer.data[j] > fmax) fmax = fft_buffer.data[j];
        //if(fft_buffer.data[j] < fmin) fmin = fft_buffer.data[j];

        //if(fft_output_data[i]  > max) max = fft_output_data[i] ;
        //if(fft_output_data[i]  < min) min = fft_output_data[i] ;

        i++;
    }

    /* Unlock FFT output buffer */
    pthread_mutex_unlock(&fft_buffer.mutex);

    //printf("min: %"PRIu32"\n", min);
    //printf("max: %"PRIu32"\n", max);
    //printf("fmin: %f\n", fmin);
    //printf("fmax: %f\n", fmax);

   	/* Calculate noise floor */
   	lowest = 0xFFFFFFFF;
   	for(j = (FFT_SIZE*0.05); j < i - (FFT_SIZE*0.1); j++)
    {
    	if(fft_output_data[j] < lowest)
    	{
    		lowest = fft_output_data[j];
    	}
    }
    lowest_smooth = (lowest * (1.f - FLOOR_TIME_SMOOTH)) + (lowest_smooth * FLOOR_TIME_SMOOTH);

    /* Compensate for noise floor */
    offset = (FLOOR_TARGET) - lowest_smooth;
    //printf("lowest: %d, lowest_smooth: %d, offset: %d\n", lowest, lowest_smooth, offset);

    for(j = 0; j < i; j++)
    {
        /* Add noise-floor AGC offset (can be negative) */
        fft_output_data[j] += offset;

        /* Subtract viewport floor offset and set to zero if underflow */
        if(__builtin_usub_overflow(fft_output_data[j], (uint32_t)FLOOR_OFFSET, &fft_output_data[j]))
        {
            fft_output_data[j] = 0;
        }

        /* Divide output by FFT_PRESCALE to scale for uint16_t */
        fft_output_data[j] /= FFT_PRESCALE;

        /* Catch data overflow */
        if(fft_output_data[j] > 0xFFFF)
        {
            fft_output_data[j] = 0xFFFF;
        }
    }

    /* Lock websocket output buffer for writing */
    pthread_mutex_lock(&_websocket_output->mutex);
    websocket_output_buffer_ptr = (uint16_t *)&_websocket_output->buffer[LWS_PRE];

    /* Copy data into websocket output buffer */
    for(j = 0; j < i; j++)
    {
        websocket_output_buffer_ptr[j] = fft_output_data[j];
    }

    _websocket_output->length = 2*i;
    _websocket_output->sequence_id++;

	pthread_mutex_unlock(&_websocket_output->mutex);
}

typedef struct websocket_user_session_t websocket_user_session_t;

struct websocket_user_session_t {
    websocket_user_session_t *websocket_user_session_list;
	struct lws *wsi;
	uint32_t last_sequence_id;
};

typedef struct {
	struct lws_context *context;
	struct lws_vhost *vhost;
	const struct lws_protocols *protocol;
	websocket_user_session_t *websocket_user_session_list;
} websocket_vhost_session_t;

int callback_fft(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    (void)in;
    (void)len;
    
	int32_t n;
	websocket_user_session_t *user_session = (websocket_user_session_t *)user;

	websocket_vhost_session_t *vhost_session =
			(websocket_vhost_session_t *)
			lws_protocol_vh_priv_get(lws_get_vhost(wsi),
					lws_get_protocol(wsi));

	switch (reason)
	{
		case LWS_CALLBACK_PROTOCOL_INIT:
			vhost_session = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
					lws_get_protocol(wsi),
					sizeof(websocket_vhost_session_t));
			vhost_session->context = lws_get_context(wsi);
			vhost_session->protocol = lws_get_protocol(wsi);
			vhost_session->vhost = lws_get_vhost(wsi);
			break;

		case LWS_CALLBACK_ESTABLISHED:
			/* add ourselves to the list of live pss held in the vhd */
			lws_ll_fwd_insert(
				user_session,
				websocket_user_session_list,
				vhost_session->websocket_user_session_list
			);
			user_session->wsi = wsi;
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft = n;
			break;

		case LWS_CALLBACK_CLOSED:
			/* remove our closing pss from the list of live pss */
			lws_ll_fwd_remove(
				websocket_user_session_t,
				websocket_user_session_list,
				user_session,
				vhost_session->websocket_user_session_list
			);
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft = n;
			break;


		case LWS_CALLBACK_SERVER_WRITEABLE:
			/* Write output data, if data exists */
			pthread_mutex_lock(&websocket_output.mutex);
			if(websocket_output.length != 0 && user_session->last_sequence_id != websocket_output.sequence_id)
			{
				n = lws_write(wsi, (unsigned char*)&websocket_output.buffer[LWS_PRE], websocket_output.length, LWS_WRITE_BINARY);
				if (!n)
				{
					pthread_mutex_unlock(&websocket_output.mutex);
					lwsl_err("ERROR %d writing to socket\n", n);
					return -1;
				}
				user_session->last_sequence_id = websocket_output.sequence_id;
			}
			pthread_mutex_unlock(&websocket_output.mutex);
			
			break;

		case LWS_CALLBACK_RECEIVE:
			/* Not expecting to receive anything */
			break;
		
		default:
			break;
	}

	return 0;
}

int callback_fft_modtslivetune(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    (void)in;
    (void)len;
    
    int32_t n;
    websocket_user_session_t *user_session = (websocket_user_session_t *)user;

    websocket_vhost_session_t *vhost_session =
            (websocket_vhost_session_t *)
            lws_protocol_vh_priv_get(lws_get_vhost(wsi),
                    lws_get_protocol(wsi));

    switch (reason)
    {
        case LWS_CALLBACK_PROTOCOL_INIT:
            vhost_session = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
                    lws_get_protocol(wsi),
                    sizeof(websocket_vhost_session_t));
            vhost_session->context = lws_get_context(wsi);
            vhost_session->protocol = lws_get_protocol(wsi);
            vhost_session->vhost = lws_get_vhost(wsi);
            break;

        case LWS_CALLBACK_ESTABLISHED:
            /* add ourselves to the list of live pss held in the vhd */
            lws_ll_fwd_insert(
                user_session,
                websocket_user_session_list,
                vhost_session->websocket_user_session_list
            );
            user_session->wsi = wsi;
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft_m0dtslivetune = n;
            break;

        case LWS_CALLBACK_CLOSED:
            /* remove our closing pss from the list of live pss */
            lws_ll_fwd_remove(
                websocket_user_session_t,
                websocket_user_session_list,
                user_session,
                vhost_session->websocket_user_session_list
            );
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft_m0dtslivetune = n;
            break;


        case LWS_CALLBACK_SERVER_WRITEABLE:
            /* Write output data, if data exists */
            pthread_mutex_lock(&websocket_output.mutex);
            if(websocket_output.length != 0 && user_session->last_sequence_id != websocket_output.sequence_id)
            {
                n = lws_write(wsi, (unsigned char*)&websocket_output.buffer[LWS_PRE], websocket_output.length, LWS_WRITE_BINARY);
                if (!n)
                {
                    pthread_mutex_unlock(&websocket_output.mutex);
                    lwsl_err("ERROR %d writing to socket\n", n);
                    return -1;
                }
                user_session->last_sequence_id = websocket_output.sequence_id;
            }
            pthread_mutex_unlock(&websocket_output.mutex);
            
            break;

        case LWS_CALLBACK_RECEIVE:
            /* Not expecting to receive anything */
            break;
        
        default:
            break;
    }

    return 0;
}

int callback_fft_fast(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    (void)in;
    (void)len;

    int32_t n;
    websocket_user_session_t *user_session = (websocket_user_session_t *)user;

    websocket_vhost_session_t *vhost_session =
            (websocket_vhost_session_t *)
            lws_protocol_vh_priv_get(lws_get_vhost(wsi),
                    lws_get_protocol(wsi));

    switch (reason)
    {
        case LWS_CALLBACK_PROTOCOL_INIT:
            vhost_session = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
                    lws_get_protocol(wsi),
                    sizeof(websocket_vhost_session_t));
            vhost_session->context = lws_get_context(wsi);
            vhost_session->protocol = lws_get_protocol(wsi);
            vhost_session->vhost = lws_get_vhost(wsi);
            break;

        case LWS_CALLBACK_ESTABLISHED:
            /* add ourselves to the list of live pss held in the vhd */
            lws_ll_fwd_insert(
                user_session,
                websocket_user_session_list,
                vhost_session->websocket_user_session_list
            );
            user_session->wsi = wsi;
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft_fast = n;
            break;

        case LWS_CALLBACK_CLOSED:
            /* remove our closing pss from the list of live pss */
            lws_ll_fwd_remove(
                websocket_user_session_t,
                websocket_user_session_list,
                user_session,
                vhost_session->websocket_user_session_list
            );
            /* Update connection count */
            n = 0;
            lws_start_foreach_ll(websocket_user_session_t *, ___pss, vhost_session->websocket_user_session_list) {
                n++;
            } lws_end_foreach_ll(___pss, websocket_user_session_list);
            lws_count_fft_fast = n;
            break;


        case LWS_CALLBACK_SERVER_WRITEABLE:
            /* Write output data, if data exists */
            pthread_mutex_lock(&websocket_output_fast.mutex);
            if(websocket_output_fast.length != 0 && user_session->last_sequence_id != websocket_output_fast.sequence_id)
            {
                n = lws_write(wsi, (unsigned char*)&websocket_output_fast.buffer[LWS_PRE], websocket_output_fast.length, LWS_WRITE_BINARY);
                if (!n)
                {
                    pthread_mutex_unlock(&websocket_output_fast.mutex);
                    lwsl_err("ERROR %d writing to socket\n", n);
                    return -1;
                }
                user_session->last_sequence_id = websocket_output_fast.sequence_id;
            }
            pthread_mutex_unlock(&websocket_output_fast.mutex);
            
            break;

        case LWS_CALLBACK_RECEIVE:
            /* Not expecting to receive anything */
            break;
        
        default:
            break;
    }

    return 0;
}

enum demo_protocols {
	PROTOCOL_FFT,
    PROTOCOL_FFT_M0DTSLIVETUNE,
    PROTOCOL_FFT_FAST,
	NOP
};

/* list of supported protocols and callbacks */
static struct lws_protocols protocols[] = {
	{
		.name = "fft",
		.callback = callback_fft,
		.per_session_data_size = 128,
		.rx_buffer_size = 4096,
	},
    {
        .name = "fft_m0dtslivetune",
        .callback = callback_fft_modtslivetune,
        .per_session_data_size = 128,
        .rx_buffer_size = 4096,
    },
    {
        .name = "fft_fast",
        .callback = callback_fft_fast,
        .per_session_data_size = 128,
        .rx_buffer_size = 4096,
    },
	{
		/* terminator */
		0
	}
};

int lws_err = 0;
/* Websocket Service Thread */
void *thread_ws(void *dummy)
{
    (void) dummy;

    while(!(lws_err < 0) && !force_exit)
    {
        lws_err = lws_service(context, 0);
    }

    return NULL;
}

void sighandler(int sig)
{
	(void) sig;
	
	force_exit = 1;
	lws_cancel_service(context);
}

int main(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	struct lws_context_creation_info info;
	struct timeval tv;
	unsigned int ms, oldms = 0, oldms_fast = 0, oldms_conn_count = 0;
	int i;

	signal(SIGINT, sighandler);

	/* we will only try to log things according to our debug_level */
	setlogmask(LOG_UPTO (LOG_DEBUG));
	openlog("lwsts", LOG_PID | LOG_PERROR, LOG_DAEMON);

	/* tell the library what debug level to emit and to send it to syslog */
	lws_set_log_level(debug_level, lwsl_emit_syslog);

	memset(&info, 0, sizeof info);
	info.port = WS_PORT;
	info.iface = NULL;
	info.protocols = protocols;
	info.gid = -1;
	info.uid = -1;
	info.max_http_header_pool = 16;
	info.options = LWS_SERVER_OPTION_VALIDATE_UTF8;
	info.timeout_secs = 5;

	fprintf(stdout, "Initialising FFT (%d bin).. ", FFT_SIZE);
	fflush(stdout);
	setup_fft();
	for(i=0; i<FFT_SIZE; i++)
	{
		hanning_window_const[i] = 0.5 * (1.0 - cos(2*M_PI*(((double)i)/FFT_SIZE)));
	}
	fprintf(stdout, "Done.\n");
	
	fprintf(stdout, "Initialising Websocket Server (LWS %d) on port %d.. ",LWS_LIBRARY_VERSION_NUMBER,info.port);
	fflush(stdout);
	context = lws_create_context(&info);
	if (context == NULL)
	{
		lwsl_err("LWS init failed\n");
		return -1;
	}
	
	fprintf(stdout, "Initialising AirSpy (%.01fMSPS, %.03fMHz).. ",(float)sample_rate_val/1000000,(float)freq_hz/1000000);
	fflush(stdout);
	if(!setup_airspy())
	{
	    fprintf(stderr, "AirSpy init failed.\n");
		return -1;
	}
	fprintf(stdout, "Done.\n");
	
	fprintf(stdout, "Starting FFT Thread.. ");
	if (pthread_create(&fftThread, NULL, thread_fft, NULL))
	{
		fprintf(stderr, "Error creating FFT thread\n");
		return -1;
	}
	pthread_setname_np(fftThread, "FFT Calculation");
	fprintf(stdout, "Done.\n");

    fprintf(stdout, "Starting Websocket Service Thread.. ");
    if (pthread_create(&wsThread, NULL, thread_ws, NULL))
    {
        fprintf(stderr, "Error creating Websocket Service thread\n");
        return -1;
    }
    pthread_setname_np(wsThread, "Websocket Srv");
    fprintf(stdout, "Done.\n");

	fprintf(stdout, "Server running.\n");
	fflush(stdout);

	while (!(lws_err < 0) && !force_exit)
	{
		gettimeofday(&tv, NULL);

		ms = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
		if ((ms - oldms) > WS_INTERVAL)
		{
			/* Copy latest FFT data to WS Output Buffer */
			fft_to_buffer(&websocket_output);

			/* Trigger send on all websockets */
			lws_callback_on_writable_all_protocol(context, &protocols[PROTOCOL_FFT]);
            lws_callback_on_writable_all_protocol(context, &protocols[PROTOCOL_FFT_M0DTSLIVETUNE]);

			/* Reset timer */
			oldms = ms;
		}
        if ((ms - oldms_fast) > WS_INTERVAL_FAST)
        {
            /* Copy latest FFT data to WS Output Buffer */
            fft_to_buffer(&websocket_output_fast);

            /* Trigger send on all websockets */
            lws_callback_on_writable_all_protocol(context, &protocols[PROTOCOL_FFT_FAST]);

            /* Reset timer */
            oldms_fast = ms;
        }
        if ((ms - oldms_conn_count) > STDOUT_INTERVAL_CONNCOUNT)
        {
            fprintf(stdout, "Connections: fft: %d, fft_m0dtslivetune: %d, fft_fast: %d\n",
                lws_count_fft,
                lws_count_fft_m0dtslivetune,
                lws_count_fft_fast
            );

            /* Reset timer */
            oldms_conn_count = ms;
        }
		
        sleep_ms(10);
	}

    /* Wait for ws thread to terminate before destroying ws */
    pthread_join(wsThread, NULL);
    lws_context_destroy(context);

	close_airspy();
	close_fftw();
	closelog();

	return 0;
}
