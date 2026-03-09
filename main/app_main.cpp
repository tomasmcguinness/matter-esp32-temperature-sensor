/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <bsp/esp-bsp.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <nvs_flash.h>

#include <app_openthread_config.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "onewire_bus.h"
#include "ds18b20.h"

#include <math.h>

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

#define ABORT_APP_ON_FAILURE(x, ...)               \
    do                                             \
    {                                              \
        if (!(unlikely(x)))                        \
        {                                          \
            __VA_ARGS__;                           \
            vTaskDelay(5000 / portTICK_PERIOD_MS); \
            abort();                               \
        }                                          \
    } while (0)

#define ADC1_CHANNEL_0 ADC_CHANNEL_0
#define ADC1_CHANNEL_2 ADC_CHANNEL_2

#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define BCOEFFICIENT 3969
#define SERIESRESISTOR 10000

#define DS18B20_GPIO_NUM GPIO_NUM_10

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_chan0_handle = NULL;
adc_cali_handle_t adc1_cali_chan2_handle = NULL;
bool do_calibration1_chan0;
bool do_calibration1_chan2;

ds18b20_device_handle_t ds18b20_handle = NULL;

static int16_t read_thermistor(adc_channel_t channel, adc_cali_handle_t cali_handle)
{
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, channel, adc_raw);

    int voltage_mv;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, channel, voltage_mv);

    float resistance = (voltage_mv * SERIESRESISTOR) / (3300 - voltage_mv);

    double temperature;
    temperature = resistance / THERMISTORNOMINAL;       // (R/Ro)
    temperature = log(temperature);                     // ln(R/Ro)
    temperature /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
    temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    temperature = 1.0 / temperature;                    // Invert
    temperature -= 273.15;                              // Convert to Celsius

    ESP_LOGI(TAG, "Channel[%d] Temperature: %f", channel, temperature);
    return static_cast<int16_t>(temperature * 100);
}

static int16_t read_ds18b20()
{
    esp_err_t err = ds18b20_trigger_temperature_conversion(ds18b20_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "DS18B20 conversion failed: %d", err);
        return INT16_MIN;
    }

    float temperature;
    err = ds18b20_get_temperature(ds18b20_handle, &temperature);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "DS18B20 read failed: %d", err);
        return INT16_MIN;
    }

    ESP_LOGI(TAG, "DS18B20 Temperature: %f", temperature);
    return static_cast<int16_t>(temperature * 100);
}

static void update_endpoint_temperature(uint16_t endpoint_id, int16_t value)
{
    attribute_t *attr = attribute::get(endpoint_id, TemperatureMeasurement::Id,
                                       TemperatureMeasurement::Attributes::MeasuredValue::Id);
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attr, &val);
    val.val.i16 = value;
    attribute::update(endpoint_id, TemperatureMeasurement::Id,
                      TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
}

void read_temperature(void *pvParameters)
{
    while (1)
    {
        int16_t value0 = read_thermistor(ADC1_CHANNEL_0, adc1_cali_chan0_handle);
        int16_t value2 = read_thermistor(ADC1_CHANNEL_2, adc1_cali_chan2_handle);
        int16_t value_ds18b20 = read_ds18b20();

        chip::DeviceLayer::SystemLayer().ScheduleLambda([value0, value2, value_ds18b20]()
        {
            update_endpoint_temperature(1, value0);
            update_endpoint_temperature(2, value2);
            if (value_ds18b20 != INT16_MIN)
            {
                update_endpoint_temperature(3, value_ds18b20);
            }
        });

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);

    // After removing last fabric, this example does not remove the Wi-Fi credentials
    // and still has IP connectivity so, only advertising on DNS-SD.
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                                chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully");
        open_commissioning_window_if_necessary();
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    // Since this is just a sensor and we don't expect any writes on our temperature sensor,
    // so, return success.
    return ESP_OK;
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

extern "C" void app_main()
{
    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Setup the ADC */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_0, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_2, &adc_config));

    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);
    do_calibration1_chan2 = adc_calibration_init(ADC_UNIT_1, ADC1_CHANNEL_2, ADC_ATTEN_DB_12, &adc1_cali_chan2_handle);

    /* Setup the DS18B20 1-Wire sensor */
    onewire_bus_handle_t onewire_bus;
    onewire_bus_config_t onewire_bus_config = {
        .bus_gpio_num = DS18B20_GPIO_NUM,
    };
    onewire_bus_rmt_config_t onewire_rmt_config = {
        .max_rx_bytes = 10,
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&onewire_bus_config, &onewire_rmt_config, &onewire_bus));

    onewire_device_iter_handle_t iter;
    ESP_ERROR_CHECK(onewire_new_device_iter(onewire_bus, &iter));
    onewire_device_t onewire_device;
    if (onewire_device_iter_get_next(iter, &onewire_device) == ESP_OK)
    {
        ds18b20_config_t ds18b20_cfg = {};
        ESP_ERROR_CHECK(ds18b20_new_device(&onewire_device, &ds18b20_cfg, &ds18b20_handle));
        ESP_LOGI(TAG, "DS18B20 found and initialised");
    }
    else
    {
        ESP_LOGW(TAG, "No DS18B20 found on 1-Wire bus");
    }
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // add temperature sensor endpoints (Channel 0 on ep1, Channel 2 on ep2)
    temperature_sensor::config_t temp_sensor_config;
    endpoint_t *temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint 1"));

    temperature_sensor::config_t temp_sensor_config2;
    endpoint_t *temp_sensor_ep2 = temperature_sensor::create(node, &temp_sensor_config2, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep2 != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint 2"));

    temperature_sensor::config_t temp_sensor_config3;
    endpoint_t *temp_sensor_ep3 = temperature_sensor::create(node, &temp_sensor_config3, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep3 != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint 3"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    esp_err_t err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    xTaskCreate(read_temperature, "ReadTemperature", 4096, NULL, 1, NULL);
}
