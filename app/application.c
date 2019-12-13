#include <application.h>

#define TEMPERATURE_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_PUB_VALUE_CHANGE 0.2f
#define TEMPERATURE_UPDATE_INTERVAL (1 * 1000)

BC_DATA_STREAM_INT_BUFFER(stream_buffer, 512)

#define MAX 1024.f

// LED instance
bc_led_t led;
bool led_state = false;

// Button instance
bc_button_t button;

// Temperature instance
bc_tag_temperature_t temperature;
event_param_t temperature_event_param = { .next_pub = 0 };

// Led strip
static uint32_t _bc_module_power_led_strip_dma_buffer[LED_STRIP_COUNT * LED_STRIP_TYPE * 2];
const bc_led_strip_buffer_t led_strip_buffer =
{
    .type = LED_STRIP_TYPE,
    .count = LED_STRIP_COUNT,
    .buffer = _bc_module_power_led_strip_dma_buffer
};

bc_led_strip_t led_strip;

bc_data_stream_t stream;

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == BC_BUTTON_EVENT_PRESS)
    {
        bc_led_pulse(&led, 100);
    }
}

void temperature_tag_event_handler(bc_tag_temperature_t *self, bc_tag_temperature_event_t event, void *event_param)
{
    float value;
    event_param_t *param = (event_param_t *)event_param;

    if (event == BC_TAG_TEMPERATURE_EVENT_UPDATE)
    {
        if (bc_tag_temperature_get_temperature_celsius(self, &value))
        {
            if ((fabs(value - param->value) >= TEMPERATURE_PUB_VALUE_CHANGE) || (param->next_pub < bc_scheduler_get_spin_tick()))
            {
                bc_radio_pub_temperature(param->channel, &value);

                param->value = value;
                param->next_pub = bc_scheduler_get_spin_tick() + TEMPERATURE_PUB_NO_CHANGE_INTEVAL;
            }
        }
    }
}

void bc_radio_node_on_state_get(uint64_t *id, uint8_t state_id)
{
    (void) id;

    if (state_id == BC_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        bool state = bc_module_power_relay_get_state();

        bc_radio_pub_state(BC_RADIO_PUB_STATE_POWER_MODULE_RELAY, &state);
    }
    else if (state_id == BC_RADIO_NODE_STATE_LED)
    {
        bc_radio_pub_state(BC_RADIO_PUB_STATE_LED, &led_state);
    }
}

void bc_radio_node_on_state_set(uint64_t *id, uint8_t state_id, bool *state)
{
    (void) id;

    if (state_id == BC_RADIO_NODE_STATE_POWER_MODULE_RELAY)
    {
        bc_module_power_relay_set_state(*state);

        bc_radio_pub_state(BC_RADIO_PUB_STATE_POWER_MODULE_RELAY, state);
    }
    else if (state_id == BC_RADIO_NODE_STATE_LED)
    {
        led_state = *state;

        bc_led_set_mode(&led, led_state ? BC_LED_MODE_ON : BC_LED_MODE_OFF);

        bc_radio_pub_state(BC_RADIO_PUB_STATE_LED, &led_state);
    }
}

void bc_radio_node_on_led_strip_brightness_set(uint64_t *id, uint8_t *brightness)
{
    (void) id;

    bc_led_strip_set_brightness(&led_strip, *brightness);
}

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *event_param)
{
    uint16_t result;
    static uint16_t max = 0;
    static int num = 0;
    bc_adc_async_get_value(channel, &result);
    result >>= 4;

    if (result > MAX) result = MAX;

    if (result > max) max = result;

    if (num++ > 5) {
        int r = max;
        bc_data_stream_feed(&stream, &r);
        num = 0;
        max = 0;
    }

    int cnt = (result / MAX) * LED_STRIP_COUNT;

    int avg = 0;
    bc_data_stream_get_max(&stream, &avg);

    int cnt_max = (avg / MAX) * LED_STRIP_COUNT;

    bc_log_debug("%d", cnt);

    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        if (i < cnt)
        {
            bc_led_strip_set_pixel_rgbw(&led_strip, i, 255,0,0, 0);
        }
        else if (i < cnt_max)
        {
            bc_led_strip_set_pixel_rgbw(&led_strip, i, 0,0,0, 16);
        }
        else
        {
            bc_led_strip_set_pixel_rgbw(&led_strip, i, 0,0,0, 0);
        }
    }

    bc_led_strip_write(&led_strip);

    bc_adc_async_measure(BC_ADC_CHANNEL_A4);
}

void application_init(void)
{
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    bc_system_pll_enable();

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    bc_radio_init(BC_RADIO_MODE_NODE_LISTENING);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_scan_interval(&button, 20);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize temperature
    temperature_event_param.channel = BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE;
    bc_tag_temperature_init(&temperature, BC_I2C_I2C0, BC_TAG_TEMPERATURE_I2C_ADDRESS_ALTERNATE);
    bc_tag_temperature_set_update_interval(&temperature, TEMPERATURE_UPDATE_INTERVAL);
    bc_tag_temperature_set_event_handler(&temperature, temperature_tag_event_handler, &temperature_event_param);

    // Initialize power module
    bc_module_power_init();

    // Initialize led-strip on power module
    bc_led_strip_init(&led_strip, bc_module_power_get_led_strip_driver(), &led_strip_buffer);

    bc_adc_init();
    bc_adc_resolution_set(BC_ADC_CHANNEL_A4, BC_ADC_RESOLUTION_8_BIT);
    bc_adc_oversampling_set(BC_ADC_CHANNEL_A4, BC_ADC_OVERSAMPLING_4);
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A4, adc_event_handler, NULL);
    bc_adc_async_measure(BC_ADC_CHANNEL_A4);

    bc_data_stream_init(&stream, 1, &stream_buffer);

    bc_radio_pairing_request("noise", VERSION);

    bc_led_pulse(&led, 2000);
}


void application_task(void)
{
}
