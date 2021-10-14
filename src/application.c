#include <application.h>

#define TEMPERATURE_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_PUB_VALUE_CHANGE 0.2f
#define TEMPERATURE_UPDATE_INTERVAL (1 * 1000)

TWR_DATA_STREAM_INT_BUFFER(stream_buffer, 512)

#define MAX 1024.f

// LED instance
twr_led_t led;
bool led_state = false;

// Button instance
twr_button_t button;

// Temperature instance
twr_tag_temperature_t temperature;
event_param_t temperature_event_param = { .next_pub = 0 };

// Led strip
static uint32_t _twr_module_power_led_strip_dma_buffer[LED_STRIP_COUNT * LED_STRIP_TYPE * 2];
const twr_led_strip_buffer_t led_strip_buffer =
{
    .type = LED_STRIP_TYPE,
    .count = LED_STRIP_COUNT,
    .buffer = _twr_module_power_led_strip_dma_buffer
};

twr_led_strip_t led_strip;

twr_data_stream_t stream;

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == TWR_BUTTON_EVENT_PRESS)
    {
        twr_led_pulse(&led, 100);
    }
}

void temperature_tag_event_handler(twr_tag_temperature_t *self, twr_tag_temperature_event_t event, void *event_param)
{
    float value;
    event_param_t *param = (event_param_t *)event_param;

    if (event == TWR_TAG_TEMPERATURE_EVENT_UPDATE)
    {
        if (twr_tag_temperature_get_temperature_celsius(self, &value))
        {
            if ((fabs(value - param->value) >= TEMPERATURE_PUB_VALUE_CHANGE) || (param->next_pub < twr_scheduler_get_spin_tick()))
            {
                twr_radio_pub_temperature(param->channel, &value);

                param->value = value;
                param->next_pub = twr_scheduler_get_spin_tick() + TEMPERATURE_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
}

void twr_radio_node_on_state_get(uint64_t *id, uint8_t state_id)
{
    (void) id;

    if (state_id == TWR_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        bool state = twr_module_power_relay_get_state();

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_POWER_MODULE_RELAY, &state);
    }
    else if (state_id == TWR_RADIO_NODE_STATE_LED)
    {
        twr_radio_pub_state(TWR_RADIO_PUB_STATE_LED, &led_state);
    }
}

void twr_radio_node_on_state_set(uint64_t *id, uint8_t state_id, bool *state)
{
    (void) id;

    if (state_id == TWR_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        twr_module_power_relay_set_state(*state);

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_POWER_MODULE_RELAY, state);
    }
    else if (state_id == TWR_RADIO_NODE_STATE_LED)
    {
        led_state = *state;

        twr_led_set_mode(&led, led_state ? TWR_LED_MODE_ON : TWR_LED_MODE_OFF);

        twr_radio_pub_state(TWR_RADIO_PUB_STATE_LED, &led_state);
    }
}

void twr_radio_node_on_led_strip_brightness_set(uint64_t *id, uint8_t *brightness)
{
    (void) id;

    twr_led_strip_set_brightness(&led_strip, *brightness);
}

void adc_event_handler(twr_adc_channel_t channel, twr_adc_event_t event, void *event_param)
{
    uint16_t result;
    static uint16_t max = 0;
    static int num = 0;
    twr_adc_async_get_value(channel, &result);
    result >>= 4;

    if (result > MAX) result = MAX;

    if (result > max) max = result;

    if (num++ > 5) {
        int r = max;
        twr_data_stream_feed(&stream, &r);
        num = 0;
        max = 0;
    }

    int cnt = (result / MAX) * LED_STRIP_COUNT;

    int avg = 0;
    twr_data_stream_get_max(&stream, &avg);

    int cnt_max = (avg / MAX) * LED_STRIP_COUNT;

    twr_log_debug("%d", cnt);

    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        if (i < cnt)
        {
            twr_led_strip_set_pixel_rgbw(&led_strip, i, 255,0,0, 0);
        }
        else if (i < cnt_max)
        {
            twr_led_strip_set_pixel_rgbw(&led_strip, i, 0,0,0, 16);
        }
        else
        {
            twr_led_strip_set_pixel_rgbw(&led_strip, i, 0,0,0, 0);
        }
    }

    twr_led_strip_write(&led_strip);

    twr_adc_async_measure(TWR_ADC_CHANNEL_A4);
}

void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    twr_system_pll_enable();

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_set_mode(&led, TWR_LED_MODE_OFF);

    twr_radio_init(TWR_RADIO_MODE_NODE_LISTENING);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_scan_interval(&button, 20);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize temperature
    temperature_event_param.channel = TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE;
    twr_tag_temperature_init(&temperature, TWR_I2C_I2C0, TWR_TAG_TEMPERATURE_I2C_ADDRESS_ALTERNATE);
    twr_tag_temperature_set_update_interval(&temperature, TEMPERATURE_UPDATE_INTERVAL);
    twr_tag_temperature_set_event_handler(&temperature, temperature_tag_event_handler, &temperature_event_param);

    // Initialize power module
    twr_module_power_init();

    // Initialize led-strip on power module
    twr_led_strip_init(&led_strip, twr_module_power_get_led_strip_driver(), &led_strip_buffer);

    twr_adc_init();
    twr_adc_resolution_set(TWR_ADC_CHANNEL_A4, TWR_ADC_RESOLUTION_8_BIT);
    twr_adc_oversampling_set(TWR_ADC_CHANNEL_A4, TWR_ADC_OVERSAMPLING_4);
    twr_adc_set_event_handler(TWR_ADC_CHANNEL_A4, adc_event_handler, NULL);
    twr_adc_async_measure(TWR_ADC_CHANNEL_A4);

    twr_data_stream_init(&stream, 1, &stream_buffer);

    twr_radio_pairing_request("noise", VERSION);

    twr_led_pulse(&led, 2000);
}


void application_task(void)
{
}
