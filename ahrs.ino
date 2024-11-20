#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP180.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float pitch;
float roll;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

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
BMP180 Barometer;

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
    Barometer.init();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    delay(1000);
    Serial.println("     ");

    //Mxyz_init_calibrated ();

}

void loop()
{

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


    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);

    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Roll: ");
    Serial.println(roll);
    // Serial.println("Gyro(degress/s) of X,Y,Z:");
    // Serial.print(Gxyz[0]);
    // Serial.print(",");
    // Serial.print(Gxyz[1]);
    // Serial.print(",");
    // Serial.println(Gxyz[2]);
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

    // temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); //Get the temperature, bmp180ReadUT MUST be called first
    // pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());//Get the temperature
    // altitude = Barometer.calcAltitude(pressure); //Uncompensated caculation - in Meters
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
    delay(1000);

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

    get_mcalibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void get_mcalibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();
        
        Serial.print("Sample data:");
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];

    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;

}



void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

float kalmanFilterFusionX(float accelReading, float gyroReading) {
    // Prediction step using gyro data
    gyroEstimateX += gyroReading; // Integrate gyro reading to predict new state

    // Kalman gain for accelerometer
    kalmanGainX = accelErrorEstimateX / (accelErrorEstimateX + accelErrorMeasureX);
    // Update step with accelerometer reading
    accelEstimateX = accelEstimateX + kalmanGainX * (accelReading - accelEstimateX);

    // Kalman gain for gyroscope
    kalmanGainGyroX = gyroErrorEstimateX / (gyroErrorEstimateX + accelErrorEstimateX);
    // Combine gyro and accelerometer data
    accelEstimateX = accelEstimateX + kalmanGainGyroX * (gyroEstimateX - accelEstimateX);

    // Update error estimates
    accelErrorEstimateX = (1 - kalmanGainX) * accelErrorEstimateX + 0.01;
    gyroErrorEstimateX = (1 - kalmanGainGyroX) * gyroErrorEstimateX + 0.01;

    return accelEstimateX;
}

float kalmanFilterFusionY(float accelReading, float gyroReading) {
    gyroEstimateY += gyroReading;

    kalmanGainY = accelErrorEstimateY / (accelErrorEstimateY + accelErrorMeasureY);
    accelEstimateY = accelEstimateY + kalmanGainY * (accelReading - accelEstimateY);

    kalmanGainGyroY = gyroErrorEstimateY / (gyroErrorEstimateY + accelErrorEstimateY);
    accelEstimateY = accelEstimateY + kalmanGainGyroY * (gyroEstimateY - accelEstimateY);

    accelErrorEstimateY = (1 - kalmanGainY) * accelErrorEstimateY + 0.01;
    gyroErrorEstimateY = (1 - kalmanGainGyroY) * gyroErrorEstimateY + 0.01;

    return accelEstimateY;
}

float kalmanFilterFusionZ(float accelReading, float gyroReading) {
    gyroEstimateZ += gyroReading;

    kalmanGainZ = accelErrorEstimateZ / (accelErrorEstimateZ + accelErrorMeasureZ);
    accelEstimateZ = accelEstimateZ + kalmanGainZ * (accelReading - accelEstimateZ);

    kalmanGainGyroZ = gyroErrorEstimateZ / (gyroErrorEstimateZ + accelErrorEstimateZ);
    accelEstimateZ = accelEstimateZ + kalmanGainGyroZ * (gyroEstimateZ - accelEstimateZ);

    accelErrorEstimateZ = (1 - kalmanGainZ) * accelErrorEstimateZ + 0.01;
    gyroErrorEstimateZ = (1 - kalmanGainGyroZ) * gyroErrorEstimateZ + 0.01;

    return accelEstimateZ;
}


void getAccel_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    float rawAx = ((double) ax / 16384);
    float rawAy = ((double) ay / 16384);
    float rawAz = ((double) az / 16384);

    float rawGx = (double) gx * 250 / 32768;
    float rawGy = (double) gy * 250 / 32768;
    float rawGz = (double) gz * 250 / 32768;

    // Apply Kalman filter to each axis
    Axyz[0] = kalmanFilterFusionZ(rawAx, rawGx); // Filtered X-axis
    Axyz[1] = kalmanFilterFusionY(rawAy, rawGy); // Filtered Y-axis
    Axyz[2] = kalmanFilterFusionZ(rawAz, rawGz); // Filtered Z-axis
}


void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
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


