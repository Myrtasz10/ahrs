#include "Wire.h"
#include <math.h>  // For sqrt()

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro_1(0x68);
MPU9250 accelgyro_2(0x69);
I2Cdev I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float radRoll;
float radPitch;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float Displacementxy[2] = { 0.0, 0.0};

#define sample_num_adate 50
#define sample_num_gdate 50
#define sample_num_mdate 50

static float ax_centre = 0;
static float ay_centre = 0;
static float az_centre = 0;

static float gx_centre = 0;
static float gy_centre = 0;
static float gz_centre = 0;

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

float speed_x = 0;
float speed_y = 0;

static float xn = 0;
static float xe = 0;
static float yn = 0;
static float ye = 0;

unsigned long lastTime = 0;  // Stores the last update time
unsigned long currentTime = 0;
float dt = 0.0;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0,
      KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0,
      KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float dt) {

  KalmanState = KalmanState + dt * KalmanInput;

  KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;

  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro_1.initialize();
  accelgyro_2.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro_1.testConnection() ? "[1] MPU9250 connection successful" : "[1] MPU9250 connection failed");
  Serial.println(accelgyro_2.testConnection() ? "[2] MPU9250 connection successful" : "[2] MPU9250 connection failed");

  delay(1000);
  Serial.println("     ");

  if(accelgyro_1.testConnection()) {
    Axyz_init_calibrated(accelgyro_1);
    Gxyz_init_calibrated(accelgyro_1);
    Mxyz_init_calibrated(accelgyro_1);
  }

  if(accelgyro_2.testConnection()) {
    Axyz_init_calibrated(accelgyro_2);
    Gxyz_init_calibrated(accelgyro_2);
    Mxyz_init_calibrated(accelgyro_2);
  }
}

void loop() {
  currentTime = millis();                
  dt = (currentTime - lastTime) / 1000.0;  
  lastTime = currentTime;                  

  getAccel_Data(accelgyro_2);
  getGyro_Data(accelgyro_2);
  getCompassDate_calibrated(accelgyro_2);

  AngleRoll = atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2])) * 1 / (3.142 / 180);
  AnglePitch = -atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 1 / (3.142 / 180);

  RateRoll = Gxyz[0];
  RatePitch = Gxyz[1];

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll, dt);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, dt);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);

  radRoll = KalmanAngleRoll / 180 * 3.142;
  radPitch = KalmanAnglePitch / 180 * 3.142;

  getHeading();
  getTiltHeading();

  float radTiltHeading = tiltheading / 180 * 3.142;

  float acceleration_x = ((Axyz[0] + sin(radPitch)) / cos(radPitch)) * 9.81 * dt;
  float acceleration_y = ((Axyz[1] - sin(radRoll)) / cos(radRoll)) * 9.81 * dt;

  speed_x += acceleration_x;
  speed_y += acceleration_y;

  xe = speed_x * sin(radTiltHeading);
  xn = speed_x * cos(radTiltHeading);
  yn = speed_y * sin(radTiltHeading);
  ye = speed_y * cos(radTiltHeading);

  Displacementxy[0] += (xn-yn)*dt;
  Displacementxy[1] += (xe+ye)*dt;

  Serial.println("Displacement(m) of X,Y:");
  Serial.print(Displacementxy[0], 6);
  Serial.print(",");
  Serial.print(Displacementxy[1], 6);
}


/*
MAGNETOMETER READING
*/


void getHeading(void) {
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
  float xh = Mxyz[0] * cos(radPitch) + Mxyz[2] * sin(radPitch);
  float yh = Mxyz[0] * sin(radRoll) * sin(radPitch) + Mxyz[1] * cos(radRoll) - Mxyz[2] * sin(radRoll) * cos(radPitch);
  float zh = -Mxyz[0] * cos(radRoll) * sin(radPitch) + Mxyz[1] * sin(radRoll) + Mxyz[2] * cos(radRoll) * cos(radPitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0) tiltheading += 360;
}



/*
CALIBRATION
*/


void Axyz_init_calibrated(MPU9250 accelgyro) {

  Serial.println(F("Before using 9DOF,we need to calibrate the accelerometer frist"));
  Serial.print("  ");
  Serial.println(F("If you are ready, please sent a command data 'ready' to start sample and calibrate."));
  while (!Serial.find("ready"))
    ;
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  calibrate_accelerometer(accelgyro);

  Serial.println("     ");
  Serial.println("accelerometer calibration parameter ");
  Serial.print(ax_centre);
  Serial.print("     ");
  Serial.print(ay_centre);
  Serial.print("     ");
  Serial.println(az_centre);
  Serial.println("    ");
}

void calibrate_accelerometer(MPU9250 accelgyro) {

  float ax_samples[sample_num_adate];
  float ay_samples[sample_num_adate];
  float az_samples[sample_num_adate];

  // Collect samples
  for (int i = 0; i < sample_num_adate; i++) {
    getRawAccel_Data(accelgyro);
    ax_samples[i] = Axyz[0];
    ay_samples[i] = Axyz[1];
    az_samples[i] = Axyz[2];
    delay(100);
  }

  // Calculate mean and standard deviation for each axis
  float ax_mean = calculate_mean(ax_samples, sample_num_adate);
  float ay_mean = calculate_mean(ay_samples, sample_num_adate);
  float az_mean = calculate_mean(az_samples, sample_num_adate);

  float ax_stddev = calculate_stddev(ax_samples, sample_num_adate, ax_mean);
  float ay_stddev = calculate_stddev(ay_samples, sample_num_adate, ay_mean);
  float az_stddev = calculate_stddev(az_samples, sample_num_adate, az_mean);

  // Filter outliers
  float ax_filtered[sample_num_mdate], ay_filtered[sample_num_mdate], az_filtered[sample_num_mdate];
  int ax_filtered_size = filter_outliers(ax_samples, sample_num_mdate, ax_mean, ax_stddev, ax_filtered);
  int ay_filtered_size = filter_outliers(ay_samples, sample_num_mdate, ay_mean, ay_stddev, ay_filtered);
  int az_filtered_size = filter_outliers(az_samples, sample_num_mdate, az_mean, az_stddev, az_filtered);

  // Calculate the final mean for each axis
  ax_centre = calculate_median(ax_filtered, ax_filtered_size);
  ay_centre = calculate_median(ay_filtered, ay_filtered_size);
  az_centre = calculate_median(az_filtered, az_filtered_size) - 1.0;

  // Print the results
  Serial.print("Calibration Results: ");
  Serial.print("ax_centre: ");
  Serial.print(ax_centre);
  Serial.print(" ay_centre: ");
  Serial.print(ay_centre);
  Serial.print(" az_centre: ");
  Serial.println(az_centre);
}


void Gxyz_init_calibrated(MPU9250 accelgyro) {
  calibrate_gyroscope(accelgyro);

  Serial.println("     ");
  Serial.println("gyroscope calibration parameter ");
  Serial.print(gx_centre);
  Serial.print("     ");
  Serial.print(gy_centre);
  Serial.print("     ");
  Serial.println(gz_centre);
  Serial.println("    ");
}

void calibrate_gyroscope(MPU9250 accelgyro) {

  float gx_samples[sample_num_gdate];
  float gy_samples[sample_num_gdate];
  float gz_samples[sample_num_gdate];

  // Collect samples
  for (int i = 0; i < sample_num_adate; i++) {
    getRawGyro_Data(accelgyro);
    gx_samples[i] = Gxyz[0];
    gy_samples[i] = Gxyz[1];
    gz_samples[i] = Gxyz[2];
    delay(100);
  }

  // Calculate mean and standard deviation for each axis
  float gx_mean = calculate_mean(gx_samples, sample_num_adate);
  float gy_mean = calculate_mean(gy_samples, sample_num_adate);
  float gz_mean = calculate_mean(gz_samples, sample_num_adate);

  float gx_stddev = calculate_stddev(gx_samples, sample_num_adate, gx_mean);
  float gy_stddev = calculate_stddev(gy_samples, sample_num_adate, gy_mean);
  float gz_stddev = calculate_stddev(gz_samples, sample_num_adate, gz_mean);

  // Filter outliers
  float gx_filtered[sample_num_mdate], gy_filtered[sample_num_mdate], gz_filtered[sample_num_mdate];
  int gx_filtered_size = filter_outliers(gx_samples, sample_num_mdate, gx_mean, gx_stddev, gx_filtered);
  int gy_filtered_size = filter_outliers(gy_samples, sample_num_mdate, gy_mean, gy_stddev, gy_filtered);
  int gz_filtered_size = filter_outliers(gz_samples, sample_num_mdate, gz_mean, gz_stddev, gz_filtered);

  // Calculate the final mean for each axis
  gx_centre = calculate_median(gx_filtered, gx_filtered_size);
  gy_centre = calculate_median(gy_filtered, gy_filtered_size);
  gz_centre = calculate_median(gz_filtered, gz_filtered_size);

  // Print the results
  Serial.print("Calibration Results: ");
  Serial.print("gx_centre: ");
  Serial.print(gx_centre);
  Serial.print(" gy_centre: ");
  Serial.print(gy_centre);
  Serial.print(" gz_centre: ");
  Serial.println(gz_centre);
}

void Mxyz_init_calibrated(MPU9250 accelgyro) {

  calibrate_magnetometer(accelgyro);

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}

void calibrate_magnetometer(MPU9250 accelgyro) {
  float mx_samples[sample_num_mdate];
  float my_samples[sample_num_mdate];
  float mz_samples[sample_num_mdate];

  // Collect samples
  for (int i = 0; i < sample_num_mdate; i++) {
    getCompass_Data(accelgyro);
    mx_samples[i] = Mxyz[0];
    my_samples[i] = Mxyz[1];
    mz_samples[i] = Mxyz[2];
  }

  // Calculate mean and standard deviation for each axis
  float mx_mean = calculate_mean(mx_samples, sample_num_mdate);
  float my_mean = calculate_mean(my_samples, sample_num_mdate);
  float mz_mean = calculate_mean(mz_samples, sample_num_mdate);

  float mx_stddev = calculate_stddev(mx_samples, sample_num_mdate, mx_mean);
  float my_stddev = calculate_stddev(my_samples, sample_num_mdate, my_mean);
  float mz_stddev = calculate_stddev(mz_samples, sample_num_mdate, mz_mean);

  // Filter outliers
  float mx_filtered[sample_num_mdate], my_filtered[sample_num_mdate], mz_filtered[sample_num_mdate];
  int mx_filtered_size = filter_outliers(mx_samples, sample_num_mdate, mx_mean, mx_stddev, mx_filtered);
  int my_filtered_size = filter_outliers(my_samples, sample_num_mdate, my_mean, my_stddev, my_filtered);
  int mz_filtered_size = filter_outliers(mz_samples, sample_num_mdate, mz_mean, mz_stddev, mz_filtered);

  // Calculate the final mean for each axis
  mx_centre = calculate_median(mx_filtered, mx_filtered_size);
  my_centre = calculate_median(my_filtered, my_filtered_size);
  mz_centre = calculate_median(mz_filtered, mz_filtered_size);

  // Print the results
  Serial.print("Calibration Results: ");
  Serial.print("mx_centre: ");
  Serial.print(mx_centre);
  Serial.print(" my_centre: ");
  Serial.print(my_centre);
  Serial.print(" mz_centre: ");
  Serial.println(mz_centre);
}

void getRawAccel_Data(MPU9250 accelgyro) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Axyz[0] = ((double)ax / 16384);
  Axyz[1] = ((double)ay / 16384);
  Axyz[2] = ((double)az / 16384);
}

void getAccel_Data(MPU9250 accelgyro) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  float rawAx = ((double)ax / 16384) - ax_centre;
  float rawAy = ((double)ay / 16384) - ay_centre;
  float rawAz = ((double)az / 16384) - az_centre;

  Axyz[0] = rawAx;
  Axyz[1] = rawAy;
  Axyz[2] = rawAz;
}


void getRawGyro_Data(MPU9250 accelgyro) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double)gx * 250 / 32768;
  Gxyz[1] = (double)gy * 250 / 32768;
  Gxyz[2] = (double)gz * 250 / 32768;
}

void getGyro_Data(MPU9250 accelgyro) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((double)gx * 250 / 32768) - gx_centre;
  Gxyz[1] = ((double)gy * 250 / 32768) - gy_centre;
  Gxyz[2] = ((double)gz * 250 / 32768) - gz_centre;
}

void getCompass_Data(MPU9250 accelgyro) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Mxyz[0] = (double)mx * 250 / 32768;
  Mxyz[1] = (double)my * 250 / 32768;
  Mxyz[2] = (double)mz * 250 / 32768;
}

void getCompassDate_calibrated(MPU9250 accelgyro) {
  getCompass_Data(accelgyro);
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}

void resetSpeed() {
  speed_x = 0;
  speed_y = 0;
}



/*
MATH FUNCTIONS
*/



// Function to compute median
float calculate_median(float* data, int size) {
  // Simple Bubble Sort to sort the data
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (data[j] > data[j + 1]) {
        // Swap data[j] and data[j + 1]
        float temp = data[j];
        data[j] = data[j + 1];
        data[j + 1] = temp;
      }
    }
  }

  // Find the median
  if (size % 2 == 0) {
    // If even, average the two middle values
    return (data[size / 2 - 1] + data[size / 2]) / 2.0;
  } else {
    // If odd, return the middle value
    return data[size / 2];
  }
}

// Function to compute mean
float calculate_mean(float* data, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

// Function to compute standard deviation
float calculate_stddev(float* data, int size, float mean) {
  float variance = 0;
  for (int i = 0; i < size; i++) {
    variance += (data[i] - mean) * (data[i] - mean);
  }
  return sqrt(variance / size);
}

// Function to filter values within 2*sigma
int filter_outliers(float* data, int size, float mean, float stddev, float* filtered_data) {
  int filtered_size = 0;
  float lower_bound = mean - 2 * stddev;
  float upper_bound = mean + 2 * stddev;

  for (int i = 0; i < size; i++) {
    if (data[i] >= lower_bound && data[i] <= upper_bound) {
      filtered_data[filtered_size++] = data[i];
    }
  }
  return filtered_size;
}
