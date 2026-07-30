#include "BNO08x.hpp"
#include "esp_log.h"
#include "gaussian.h"
#include "sensor_hal.h"

static const char* TAG = "bno085_gyro";
static BNO08x* imu = nullptr;

extern "C" bool gyro_init(SensorContext_t* ctx) {
  if (ctx == NULL || ctx->hw_config == NULL) return false;

  imu = (BNO08x*)ctx->hw_config;
  imu->rpt.uncal_gyro.enable(1000UL);

  ESP_LOGI(TAG, "Uncalibrated Gyroscope enabled");
  return true;
}

extern "C" bool gyro_read_sample(SensorContext_t* ctx, float* data_out) {
  if (ctx == NULL || data_out == NULL || imu == nullptr) return false;

  if (!imu->rpt.uncal_gyro.has_new_data()) {
    return false;
  }

  bno08x_gyro_t d = imu->rpt.uncal_gyro.get_vel();
  data_out[0] = d.x;
  data_out[1] = d.y;
  data_out[2] = d.z;

  if (noise_injection_is_enabled()) {
    uint8_t sid = (uint8_t)ctx->id;
    float n0 = rand_gaussian_tracked(sid);
    float n1 = rand_gaussian_tracked(sid);
    float n2 = rand_gaussian_tracked(sid);
    data_out[0] += data_out[0] * n0;
    data_out[1] += data_out[1] * n1;
    data_out[2] += data_out[2] * n2;
  }
  return true;
}
