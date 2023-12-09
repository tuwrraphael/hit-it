#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <math.h>

#include <usb_midi/usb_midi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(foo);

#define LED_FLASH_DURATION_MS 60
#define TX_NOTE_NUMBER 38
#define TX_NOTE_VELOCITY 0x7f
#define NUM_ANALOG_SENSORS (1)

struct k_work button_press_work;
struct k_work note_on_work;
struct k_work_delayable tx_led_off_work;
struct k_work_delayable note_off_work;
struct k_work_delayable log_work;

#define SAADC_BUF_COUNT 2

static const nrfx_timer_t m_sample_timer = NRFX_TIMER_INSTANCE(1);
static nrf_ppi_channel_t m_timer_saadc_ppi_channel;

#define ANALYZE_INTERVAL_MS (2)
#define SAADC_SAMPLING_RATE (1000000 / 8000) // us
#define NUMBER_OF_SAMPLES ((NUM_ANALOG_SENSORS * ANALYZE_INTERVAL_MS * 1000) / SAADC_SAMPLING_RATE)
#define SAADC_BUF_SIZE (NUMBER_OF_SAMPLES)

#define DECAY_CONSTANT (0.997856241275233)

static nrf_saadc_value_t samples[SAADC_BUF_COUNT][SAADC_BUF_SIZE];

static uint32_t buffer_index = -1;

struct sample_app_state_t
{
	int usb_midi_is_available;
};

static bool note_on = false;
static uint8_t note_velocity = 127;

typedef enum
{
	DETECTION_STATE_NORMAL,
	DETECTION_STATE_THRESHOLD
} detection_state_t;

typedef struct
{
	detection_state_t state;
	bool hit;
	nrf_saadc_value_t last_hit_value;
	nrf_saadc_value_t max_value;
	nrf_saadc_value_t mask_threshold;
	uint32_t sample_ctr_since_threshold;
	uint32_t sample_ctr_masked;
} piezo_detection_state_t;

typedef struct
{
	piezo_detection_state_t dection_state;
	uint32_t min_threshold;
	uint32_t scan_samples;
	uint32_t mask_samples;
} piezo_sensor_t;

#define us_to_numer_of_samples(us) (us / SAADC_SAMPLING_RATE)

static piezo_sensor_t configured_piezo_sensors[] = {
	{.dection_state = {
		 .hit = false,
		 .max_value = 0,
		 .mask_threshold = 0,
		 .sample_ctr_since_threshold = 0,
		 .sample_ctr_masked = 0,
		 .state = DETECTION_STATE_NORMAL},
	 .min_threshold = 250,
	 .mask_samples = us_to_numer_of_samples(40000),
	 .scan_samples = us_to_numer_of_samples(20000)}};

static struct sample_app_state_t sample_app_state = {.usb_midi_is_available = 0};

static struct gpio_dt_spec midi_tx_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void init_leds()
{
	gpio_pin_configure_dt(&midi_tx_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&midi_tx_led, 0);
}

static void flash_tx_led()
{
	gpio_pin_set_dt(&midi_tx_led, 1);
	k_work_cancel_delayable(&tx_led_off_work);
	k_work_schedule(&tx_led_off_work, Z_TIMEOUT_MS(LED_FLASH_DURATION_MS));
}
static bool record_next = false;
void on_button_press(struct k_work *item)
{
	printk("button\n");
	record_next = true;
	if (sample_app_state.usb_midi_is_available)
	{
		note_velocity = 127;
		uint8_t msg[3] = {0x90, TX_NOTE_NUMBER,
						  note_velocity};
		usb_midi_tx(0, msg);
		note_on = true;
	}
}

void on_tx_led_off(struct k_work *item)
{
	gpio_pin_set_dt(&midi_tx_led, 0);
}

void on_note_off(struct k_work *item)
{
}

void on_note_on(struct k_work *item)
{
	if (sample_app_state.usb_midi_is_available)
	{
		note_on = true;
		uint8_t msg[3] = {0x90, TX_NOTE_NUMBER,
						  note_velocity};
		usb_midi_tx(0, msg);
		flash_tx_led();
	}
}

static struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_cb_data;

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&button_press_work);
}

static void init_button()
{
	__ASSERT_NO_MSG(device_is_ready(button.port));
	int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	__ASSERT_NO_MSG(ret == 0);
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	__ASSERT_NO_MSG(ret == 0);

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	__ASSERT_NO_MSG(ret == 0);
}

static void midi_message_cb(uint8_t *bytes, uint8_t num_bytes, uint8_t cable_num)
{
}

static void sysex_start_cb(uint8_t cable_num)
{
}

static void sysex_data_cb(uint8_t *data_bytes, uint8_t num_data_bytes, uint8_t cable_num)
{
}

static void sysex_end_cb(uint8_t cable_num)
{
}

static void usb_midi_available_cb(int is_available)
{
	sample_app_state.usb_midi_is_available = is_available;
	printk("midi available %d\n", is_available);
}

static void usb_midi_tx_done_cb()
{
	if (sample_app_state.usb_midi_is_available & note_on)
	{
		note_on = false;
		uint8_t msg[3] = {0x80, TX_NOTE_NUMBER,
						  note_velocity};
		usb_midi_tx(0, msg);
	}
}

static int32_t get_velocity(piezo_sensor_t *sensor)
{
	float resF = (float)(sensor->dection_state.max_value - sensor->min_threshold) / (float)(13000 - sensor->min_threshold) * 128;
	if (resF <= 1)
	{
		resF = 1;
	}
	if (resF > 127)
	{
		resF = 127;
	}
	resF = (126 / (pow(1.02, 126) - 1)) * (pow(1.02, resF - 1) - 1) + 1; // 1.02
	uint8_t res;
	res = (uint8_t)round(resF);
	return res;
}

static uint32_t decay_cycles = 0;
#define RECORD_SAMPLES (SAADC_BUF_SIZE * 100)
static nrf_saadc_value_t savedsamples[RECORD_SAMPLES];
static uint16_t samplectr = RECORD_SAMPLES;
static uint16_t logctr = 0;
#define LOG_BATCH (RECORD_SAMPLES / 20)

void on_log(struct k_work *item)
{
	if (logctr < RECORD_SAMPLES)
	{
		LOG_HEXDUMP_INF(savedsamples + logctr, LOG_BATCH * sizeof(nrf_saadc_value_t), "Non-printable:");
		logctr += LOG_BATCH;
		k_work_schedule(&log_work, Z_TIMEOUT_MS(1000));
	}
}

static void process(piezo_sensor_t piezo_sensors[], nrf_saadc_value_t samples[])
{
	for (uint32_t i = 0; i < SAADC_BUF_SIZE; i++)
	{
		uint32_t val = samples[i] < 0 ? 0 : samples[i];
		piezo_sensor_t *sensor = &piezo_sensors[i % NUM_ANALOG_SENSORS];
		switch (sensor->dection_state.state)
		{
		case DETECTION_STATE_NORMAL:

			bool is_hit = false;
			bool method = 0;
			if (sensor->dection_state.sample_ctr_masked >= sensor->mask_samples)
			{
				if (sensor->dection_state.mask_threshold > 0)
				{
					sensor->dection_state.mask_threshold = (nrf_saadc_value_t)(sensor->dection_state.mask_threshold * DECAY_CONSTANT);
					decay_cycles++;
					if (sensor->dection_state.mask_threshold == 0)
					{
						 printk("decay %d\n", decay_cycles);
					}
				}
				is_hit = val > (sensor->min_threshold + sensor->dection_state.mask_threshold);
			}
			else
			{
				sensor->dection_state.sample_ctr_masked++;
				is_hit = val > sensor->dection_state.last_hit_value * 1.5;
				method = 1;
				if (val > sensor->dection_state.mask_threshold)
				{
					sensor->dection_state.mask_threshold = val;
				}
			}
			if (is_hit)
			{
				printk("newhit %d, %d, %d, %d, %d \n",val,sensor->min_threshold, sensor->dection_state.mask_threshold, decay_cycles, method);
				sensor->dection_state.sample_ctr_since_threshold = 0;
				sensor->dection_state.max_value = val;
				if (record_next == true)
				{
					record_next = false;
					samplectr = 0;
					savedsamples[0] = val;
					printk("start recording\n");
				}
				sensor->dection_state.state = DETECTION_STATE_THRESHOLD;
			}
			break;
		case DETECTION_STATE_THRESHOLD:
			if (val > sensor->dection_state.max_value)
			{
				sensor->dection_state.max_value = val;
				sensor->dection_state.mask_threshold = val;
				// printk("2smask %d\n", sensor->dection_state.mask_threshold);
			}
			sensor->dection_state.sample_ctr_since_threshold++;
			if (sensor->dection_state.sample_ctr_since_threshold >= sensor->scan_samples)
			{
				sensor->dection_state.hit = true;
				sensor->dection_state.last_hit_value = sensor->dection_state.max_value;
				note_velocity = get_velocity(sensor);
				printk("hit %d\n", sensor->dection_state.max_value);
				k_work_submit(&note_on_work);
				sensor->dection_state.state = DETECTION_STATE_NORMAL;
				sensor->dection_state.sample_ctr_masked = 0;
				decay_cycles = 0;
			}
			break;
		}
	}
	if (samplectr < RECORD_SAMPLES)
	{
		memcpy(&savedsamples[samplectr], samples, SAADC_BUF_SIZE * sizeof(nrf_saadc_value_t));
		samplectr += SAADC_BUF_SIZE;
		if (samplectr >= RECORD_SAMPLES)
		{
			printk("start logging\n");
			logctr = 0;
			k_work_schedule(&log_work, Z_TIMEOUT_MS(1000));
		}
	}
}

static uint32_t next_free_buf_index(void)
{

	buffer_index = (buffer_index + 1) % SAADC_BUF_COUNT;
	return buffer_index;
}

static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
}

static void event_handler(nrfx_saadc_evt_t const *p_event)
{
	nrfx_err_t err_code;
	switch (p_event->type)
	{
	case NRFX_SAADC_EVT_DONE:
		// printk("ADC: %d", );
		// for (uint32_t i = 0; i < SAADC_BUF_SIZE; i++)
		// {
		// 	printk("%6d, ", p_event->data.done.p_buffer[i]);
		// }
		// printk("\n");
		process(configured_piezo_sensors, p_event->data.done.p_buffer);
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		// Set up the next available buffer
		err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
		__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_buffer_set failed");
		break;
	default:
		printk("SAADC evt %d", p_event->type);
		break;
	}
}

static void timer_init(void)
{
	nrfx_err_t err_code;
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.frequency = NRF_TIMER_FREQ_31250Hz;
	err_code = nrfx_timer_init(&m_sample_timer, &timer_config, timer_handler);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_timer_init failed");
	nrfx_timer_extended_compare(&m_sample_timer,
								NRF_TIMER_CC_CHANNEL0,
								nrfx_timer_us_to_ticks(&m_sample_timer, SAADC_SAMPLING_RATE),
								NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
								false);
	nrfx_timer_resume(&m_sample_timer);
}

static void ppi_init(void)
{
	// Trigger task sample from timer
	nrfx_err_t err_code = nrfx_ppi_channel_alloc(&m_timer_saadc_ppi_channel);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_ppi_channel_alloc failed");
	err_code = nrfx_ppi_channel_assign(m_timer_saadc_ppi_channel,
									   nrfx_timer_event_address_get(&m_sample_timer, NRF_TIMER_EVENT_COMPARE0),
									   nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_ppi_channel_assign failed");

	err_code = nrfx_ppi_channel_enable(m_timer_saadc_ppi_channel);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_ppi_channel_enable failed");
}

static void adc_configure(void)
{
	nrfx_err_t err_code;

	nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	saadc_adv_config.internal_timer_cc = 0;
	saadc_adv_config.start_on_end = true;

	err_code = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_init failed");

	static nrfx_saadc_channel_t channel_configs[1];

	nrfx_saadc_channel_t config = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
	memcpy(&channel_configs[0], &config, sizeof(config));

	// channel_configs[0].channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
	// channel_configs[0].channel_config.gain = NRF_SAADC_GAIN1_6;

	uint8_t channel_mask = (1 << 0);

	err_code = nrfx_saadc_channels_config(channel_configs, 1);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_channels_config failed");

	err_code = nrfx_saadc_advanced_mode_set(channel_mask,
											NRF_SAADC_RESOLUTION_14BIT,
											&saadc_adv_config,
											event_handler);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_advanced_mode_set failed");

	// Configure two buffers to ensure double buffering of samples, to avoid data loss when the sampling frequency is high
	err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_buffer_set failed");

	err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_buffer_set failed");

	err_code = nrfx_saadc_mode_trigger();
	__ASSERT(err_code == NRFX_SUCCESS, "nrfx_saadc_mode_trigger failed");
}

int main(void)
{
	init_leds();
	init_button();

	k_work_init(&button_press_work, on_button_press);
	k_work_init_delayable(&note_off_work, on_note_off);
	k_work_init_delayable(&tx_led_off_work, on_tx_led_off);
	k_work_init(&note_on_work, on_note_on);
	k_work_init_delayable(&log_work, on_log);

	struct usb_midi_cb_t callbacks = {.available_cb = usb_midi_available_cb,
									  .tx_done_cb = usb_midi_tx_done_cb,
									  .midi_message_cb = midi_message_cb,
									  .sysex_data_cb = sysex_data_cb,
									  .sysex_end_cb = sysex_end_cb,
									  .sysex_start_cb = sysex_start_cb};
	usb_midi_register_callbacks(&callbacks);

	/* Init USB */
	int enable_rc = usb_enable(NULL);
	__ASSERT(enable_rc == 0, "Failed to enable USB");

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
				DT_IRQ(DT_NODELABEL(adc), priority),
				nrfx_saadc_irq_handler, NULL, 0);

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer1)),
				DT_IRQ(DT_NODELABEL(timer1), priority),
				nrfx_timer_1_irq_handler, NULL, 0);

	adc_configure();
	ppi_init();
	timer_init();

	// /* Send MIDI messages periodically */
	// while (1) {
	// 	k_work_submit(&event_tx_work);
	// 	k_msleep(TX_INTERVAL_MS);
	// }
	return 0;
}