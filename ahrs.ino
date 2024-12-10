#include "Wire.h"
#include <math.h> // For sqrt()

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP180.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro(0x69);
I2Cdev   I2C_M;

uint8_t buffer_m[6];



int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float pitch;
float roll;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float Displacementxyz[3] = {0.0, 0.0, 0.0};


#define sample_num_adate  10
#define sample_num_gdate  10
#define sample_num_mdate  10

static float ax_centre = 0;
static float ay_centre = 0;
static float az_centre = 0;

static float gx_centre = 0;
static float gy_centre = 0;
static float gz_centre = 0;

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

// Kalman filter variables for each axis
float accelEstimateX = 0.0, accelEstimateY = 0.0, accelEstimateZ = 0.0;
float accelErrorEstimateX = 1.0, accelErrorEstimateY = 1.0, accelErrorEstimateZ = 1.0;
float accelErrorMeasureX = 0.1, accelErrorMeasureY = 0.1, accelErrorMeasureZ = 0.1; // Initial error in measurement, tune this
float kalmanGainX = 0.0, kalmanGainY = 0.0, kalmanGainZ = 0.0;

float gyroEstimateX = 0.0, gyroEstimateY = 0.0, gyroEstimateZ = 0.0; // Gyro estimates
float gyroErrorEstimateX = 1.0, gyroErrorEstimateY = 1.0, gyroErrorEstimateZ = 1.0; // Error estimates for gyro
float kalmanGainGyroX = 0.0, kalmanGainGyroY = 0.0, kalmanGainGyroZ = 0.0; // Kalman gains for gyro


float temperature;
float pressure;
float atm;
float altitude;

// Kalman filter variables per axis
struct KalmanAxis {
    float state;         // Current angle estimate
    float uncertainty;   // Estimate uncertainty
    float gyroBias;      // Corrected gyro bias
};

KalmanAxis kalman[3]; // Indices 0 = Roll, 1 = Pitch, 2 = Yaw

// Kalman filter function for one axis
float kalmanFilter(int axis, float gyroRate, float accelAngle, float dt) {
    // Unpack the axis state
    float &state = kalman[axis].state;
    float &uncertainty = kalman[axis].uncertainty;
    float &gyroBias = kalman[axis].gyroBias;

    // Tuning parameters
    const float processNoise = 0.02;   // System noise (gyro)
    const float measurementNoise = 0.02; // Measurement noise (accelerometer)

    // 1. Prediction Step
    state += dt * (gyroRate - gyroBias);  // Predict the next angle based on gyro rate
    uncertainty += processNoise * dt;    // Increase uncertainty

    // 2. Update Step
    float kalmanGain = uncertainty / (uncertainty + measurementNoise);
    state += kalmanGain * (accelAngle - state); // Update estimate using accelerometer
    uncertainty *= (1 - kalmanGain);           // Update uncertainty

    return state; // Return filtered value
}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    delay(1000);
    Serial.println("     ");

    Axyz_init_calibrated();
    Gxyz_init_calibrated();
    //Mxyz_init_calibrated();

}

unsigned long lastTime = 0;  // Stores the last update time
unsigned long currentTime = 0;
float dt = 0.0;

void loop()
{
    currentTime = millis();          // Get the current time
    dt = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
    lastTime = currentTime;                        // Update the last time

    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();
    getTiltHeading();

    // Serial.println("calibration parameter: ");
    // Serial.print(mx_centre);
    // Serial.print("         ");
    // Serial.print(my_centre);
    // Serial.print("         ");
    // Serial.println(mz_centre);
    // Serial.println("     ");

    Displacementxyz[0] += Axyz[0] * dt;
    Displacementxyz[1] += Axyz[1] * dt;
    Displacementxyz[2] += (Axyz[2]-1.0) * dt;

    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);

    Serial.println("Displacement(g) of X,Y,Z:");
    Serial.print(Displacementxyz[0]);
    Serial.print(",");
    Serial.print(Displacementxyz[1]);
    Serial.print(",");
    Serial.println(Displacementxyz[2]);

    Serial.println("Gyro(degress/s) of X,Y,Z:");
    Serial.print(Gxyz[0]);
    Serial.print(",");
    Serial.print(Gxyz[1]);
    Serial.print(",");
    Serial.println(Gxyz[2]);

    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Roll: ");
    Serial.println(roll);
    // Serial.println("Compass Value of X,Y,Z:");
    // Serial.print(Mxyz[0]);
    // Serial.print(",");
    // Serial.print(Mxyz[1]);
    // Serial.print(",");
    // Serial.println(Mxyz[2]);
    // Serial.println("The clockwise angle between the magnetic north and X-Axis:");
    // Serial.print(heading);
    // Serial.println(" ");
    // Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
    // Serial.println(tiltheading);
    // Serial.println("   ");

    // temperature = barometer.bmp180GetTemperature(barometer.bmp180ReadUT()); //Get the temperature, bmp180ReadUT MUST be called first
    // pressure = barometer.bmp180GetPressure(barometer.bmp180ReadUP());//Get the temperature
    // altitude = barometer.calcAltitude(pressure); //Uncompensated caculation - in Meters
    // atm = pressure / 101325;

    // Serial.print("Temperature: ");
    // Serial.print(temperature, 2); //display 2 decimal places
    // Serial.println("deg C");

    // Serial.print("Pressure: ");
    // Serial.print(pressure, 0); //whole number only.
    // Serial.println(" Pa");

    // Serial.print("Ralated Atmosphere: ");
    // Serial.println(atm, 4); //display 4 decimal places

    // Serial.print("Altitude: ");
    // Serial.print(altitude, 2); //display 2 decimal places
    // Serial.println(" m");

    Serial.println();
    delay(500);

}


void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    pitch = asin(-Axyz[0]);
    roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}


void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    calibrate_magnetometer();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void Axyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the accelerometer frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    calibrate_accelerometer();

    Serial.println("     ");
    Serial.println("accelerometer calibration parameter ");
    Serial.print(ax_centre);
    Serial.print("     ");
    Serial.print(ay_centre);
    Serial.print("     ");
    Serial.println(az_centre);
    Serial.println("    ");
}

void Gxyz_init_calibrated ()
{
    calibrate_gyroscope();

    Serial.println("     ");
    Serial.println("accelerometer gyroscope parameter ");
    Serial.print(gx_centre);
    Serial.print("     ");
    Serial.print(gy_centre);
    Serial.print("     ");
    Serial.println(gz_centre);
    Serial.println("    ");
}


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

void calibrate_magnetometer() {
    float mx_samples[sample_num_mdate];
    float my_samples[sample_num_mdate];
    float mz_samples[sample_num_mdate];

    // Collect samples
    for (int i = 0; i < sample_num_mdate; i++) {
        getCompass_Data();
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

void calibrate_accelerometer() {

    float ax_samples[sample_num_adate];
    float ay_samples[sample_num_adate];
    float az_samples[sample_num_adate];

    // Collect samples
    for (int i = 0; i < sample_num_adate; i++) {
        getRawAccel_Data();
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

void calibrate_gyroscope() {

    float gx_samples[sample_num_gdate];
    float gy_samples[sample_num_gdate];
    float gz_samples[sample_num_gdate];

    // Collect samples
    for (int i = 0; i < sample_num_adate; i++) {
        getRawGyro_Data();
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

void getRawAccel_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Axyz[0] = ((double) ax / 16384);
    Axyz[1] = ((double) ay / 16384);
    Axyz[2] = ((double) az / 16384);
}

void getAccel_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    float rawAx = ((double) ax / 16384) - ax_centre;
    float rawAy = ((double) ay / 16384) - ay_centre;
    float rawAz = ((double) az / 16384) - az_centre;

    float rawGx = (double) gx * 250 / 32768;
    float rawGy = (double) gy * 250 / 32768;
    float rawGz = (double) gz * 250 / 32768;

    // Apply Kalman filter to each axis
    // Axyz[0] = kalmanFilter(0, rawAx, rawGx, dt); // Filtered X-axis
    // Axyz[1] = kalmanFilter(1, rawAy, rawGy, dt); // Filtered Y-axis
    // Axyz[2] = kalmanFilter(2, rawAz, rawGz, dt); // Filtered Z-axis

    Axyz[0] = rawAx;
    Axyz[1] = rawAy;
    Axyz[2] = rawAz;
}


void getRawGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = ((double) gx * 250 / 32768) - gx_centre;
    Gxyz[1] = ((double) gy * 250 / 32768) - gy_centre;
    Gxyz[2] = ((double) gz * 250 / 32768) - gz_centre;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}


