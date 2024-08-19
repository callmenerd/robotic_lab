#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DPEng_BMX160.h>
//=============================================================================================
#ifndef __CalibrateBMX160_h__
#define __CalibrateBMX160_h__
#include <math.h>
#include <stdint.h>

class CalibrateBMX160{
  private:
    bool init = false;
    bool add_mag = false;
    DPEng_BMX160 dpEng = DPEng_BMX160(0x160A, 0x160B, 0x160C);

    float raw_mag[3] = {0, 0, 0};
    float raw_gyro[3] = {0, 0, 0};
    float raw_accel[3] = {0, 0, 0};

    // Offsets applied to raw x/y/z mag values
    float mag_offsets[3]            = { 9.83F, 4.42F, -6.97F };

    // Soft iron error compensation matrix
    float mag_softiron_matrix[3][3] = { {  0.586,  0.006,  0.001 },
                                        {  0.006,  0.601, -0.002 },
                                        {  0.001,  -0.002,  2.835 } };

    float mag_field_strength        = 56.33F;


    // Calib GYROSCOPE
    float gyro_bias[3] = {-0.04005197, 0.004573256, 0.03303826};
    const float alpha = 0.2;  // Smoothing factor, 0 < alpha < 1

    // Calib ACCELEROMETER
    float A[3][3] = {{0.998244, 0.002527, -0.002566}, 
                      {0.002527, 0.999236, -0.000588},
                      {-0.002566, -0.000588, 0.995214}};
    float b[3] = {-0.045349, -0.018113, 0.027794};

    const float alpha_accel = 0.2;

    // Calib MAGNETOMETER
    float Amag[3][3] = {{0.209628, 0.002664, -0.006057},
                        {0.002664, 0.203327, 0.000065},
                        {-0.006057, 0.000065, 0.307588}};
    float bmag[3] = {101.705416, -76.472890, -17.594554};
    const float alpha_mag = 0.2;

  public:
    // Calibrated Data
    float mag[3] = {0, 0, 0};
    float gyro[3] = {0, 0, 0};
    float accel[3] = {0, 0, 0};

    CalibrateBMX160(void){
      this->init = true;
    }
    void activate_mag(bool activate=false){
      this->add_mag = activate;
    }
    void begin(){
      if(!dpEng.begin(BMX160_ACCELRANGE_4G, GYRO_RANGE_250DPS)){
        while(1);
      }
    }
    void getAllUncalibratedData(){
        sensors_event_t accel_event;
        sensors_event_t gyro_event;
        sensors_event_t mag_event;
        /* Get a new sensor event */
        dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

        // Apply mag offset compensation (base values in uTesla)
        float x = mag_event.magnetic.x - mag_offsets[0];
        float y = mag_event.magnetic.y - mag_offsets[1];
        float z = mag_event.magnetic.z - mag_offsets[2];

        // Apply mag soft iron error compensation
        this->raw_mag[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
        this->raw_mag[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
        this->raw_mag[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

        this->raw_gyro[0] = gyro_event.gyro.x;
        this->raw_gyro[1] = gyro_event.gyro.y;
        this->raw_gyro[2] = gyro_event.gyro.z;

        this->raw_accel[0] = accel_event.acceleration.x;
        this->raw_accel[1] = accel_event.acceleration.y;
        this->raw_accel[2] = accel_event.acceleration.z;
    }

    void getAllDataCalibrated(){
      if (this->init){
        int n = 32;
        while (n >= 0){
          getAllUncalibratedData();
          calibrate_accelerometer_readings();
          calibrate_gyroscope_readings();
          calibrate_magnetometer_readings();
          n = n-1;
        }
        UpdateGyroCalibration(5000);
        this->init = false;
      } else{
          getAllUncalibratedData();
          calibrate_accelerometer_readings();
          calibrate_gyroscope_readings();
          calibrate_magnetometer_readings();
      }
    }

    void UpdateGyroCalibration(unsigned long duration){
      // Do not call this function every time this is only for updating the gyro_bias, once it is complete keep calm!
      float gyros_bias[3] = {0, 0, 0};  // Initialize bias values for each axis
      int sample_count = 0;

      unsigned long start_time = millis();

      while ((millis() - start_time) < duration){
          // Read gyroscope data from the sensor
          sensors_event_t accel_event;
          sensors_event_t gyro_event;
          sensors_event_t mag_event;

          // Get new data samples
          dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

          // Accumulate the data for bias calculation
          gyros_bias[0] += gyro_event.gyro.x;
          gyros_bias[1] += gyro_event.gyro.y;
          gyros_bias[2] += gyro_event.gyro.z;

          sample_count ++;
          delay(10);  // Add a delay to control the sample rate
      }
      // Calculate the average bias values
      this->gyro_bias[0] = gyros_bias[0] / sample_count;
      this->gyro_bias[1] = gyros_bias[1] / sample_count;
      this->gyro_bias[2] = gyros_bias[2] / sample_count;
    }

    void calibrate_gyroscope_readings(){
      float calibrated_gx = this->raw_gyro[0] - this->gyro_bias[0];
      float calibrated_gy = this->raw_gyro[1] - this->gyro_bias[1];
      float calibrated_gz = this->raw_gyro[2] - this->gyro_bias[2];

      float filtered_gyro_X = this->alpha * calibrated_gx + (1 - this->alpha) * this->gyro[0];
      float filtered_gyro_Y = this->alpha * calibrated_gy + (1 - this->alpha) * this->gyro[1];
      float filtered_gyro_Z = this->alpha * calibrated_gz + (1 - this->alpha) * this->gyro[2];

      this->gyro[0] = filtered_gyro_X;
      this->gyro[1] = filtered_gyro_Y;
      this->gyro[2] = filtered_gyro_Z;
    }

    void calibrate_accelerometer_readings(){
      float cax = 0.0;
      float cay = 0.0;
      float caz = 0.0;

      for(int i=0; i<3; i++){
        float calib_value = 0.0;
        calib_value += this->A[i][0] * (this->raw_accel[0] - this->b[0]);
        calib_value += this->A[i][1] * (this->raw_accel[1] - this->b[1]);
        calib_value += this->A[i][2] * (this->raw_accel[2] - this->b[2]);

        if(i == 0){
          cax = calib_value;
        } else if(i == 1){
          cay = calib_value;
        } else if(i == 2){
          caz = calib_value;
        }
      }

      //Apply low-pass filter
      float filtered_cax = this->alpha_accel * cax + (1 - this->alpha_accel) * this->accel[0];
      float filtered_cay = this->alpha_accel * cay + (1 - this->alpha_accel) * this->accel[1];
      float filtered_caz = this->alpha_accel * caz + (1 - this->alpha_accel) * this->accel[2];

      // Update previous values
      this->accel[0] = filtered_cax;
      this->accel[1] = filtered_cay;
      this->accel[2] = filtered_caz;
    }

    void calibrate_magnetometer_readings(){
      // Apply calibration
      float cmx = 0.0;
      float cmy = 0.0;
      float cmz = 0.0;

      for(int i = 0; i < 3; i++){
          float calib_value = 0.0;
          calib_value += this->Amag[i][0] * (this->raw_mag[0] - this->bmag[0]);
          calib_value += this->Amag[i][1] * (this->raw_mag[1] - this->bmag[1]);
          calib_value += this->Amag[i][2] * (this->raw_mag[2] - this->bmag[2]);

          if(i == 0){
            cmx = calib_value;
          } else if (i == 1){
            cmy = calib_value;
          } else if (i == 2){
            cmz = calib_value;
          }
      }
      float filtered_cmx = this->alpha_mag * cmx + (1 - this->alpha_mag) * this->mag[0];
      float filtered_cmy = this->alpha_mag * cmy + (1 - this->alpha_mag) * this->mag[1];
      float filtered_cmz = this->alpha_mag * cmz + (1 - this->alpha_mag) * this->mag[2];
      this->mag[0] = filtered_cmx;
      this->mag[1] = filtered_cmy;
      this->mag[2] = filtered_cmz;
    }
};
#endif