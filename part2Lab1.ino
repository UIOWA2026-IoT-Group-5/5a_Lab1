/*
  IoT Lab 1 - Part II (Arduino Nano 33 BLE Sense)
  Requirements met:
  - Temp + Humidity from HTS221
  - One additional sensor: Gyroscope (LSM9DS1)
  - Sample once per second
  - Print current values + average of last 10 samples
*/
 
#include <Arduino_HTS221.h>   // Temperature + Humidity sensor
#include <Arduino_LSM9DS1.h>  // IMU (gyro)
 
static const int N = 10;
 
// Ring buffers
float tempBuf[N] = {0};
float humBuf[N]  = {0};
 
float gxBuf[N] = {0};
float gyBuf[N] = {0};
float gzBuf[N] = {0};
 
int idx = 0;       // next write position
int countN = 0;    // how many samples collected so far (<= N)
 
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial Monitor */ }
 
  // Init HTS221
  if (!HTS.begin()) {
    Serial.println("ERROR: Failed to initialize HTS221 (temp/humidity).");
    while (1) {}
  }
 
  // Init IMU (LSM9DS1)
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU (LSM9DS1).");
    while (1) {}
  }
 
  Serial.println("Lab 1 Part II started.");
  Serial.println("Sampling once per second. Printing CURRENT + AVG10.\n");
}
 
void loop() {
  // ---- Read sensors (CURRENT) ----
  float tempC = HTS.readTemperature();
  float humRH = HTS.readHumidity();
 
  // Gyro read (deg/sec). Only update if gyro data is available.
  float gx = NAN, gy = NAN, gz = NAN;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  } else {
    // If gyro isn't available this cycle, treat it as 0 or keep last.
    // We'll choose 0 so the loop remains deterministic.
    gx = 0; gy = 0; gz = 0;
  }
 
  // ---- Store into ring buffer ----
  tempBuf[idx] = tempC;
  humBuf[idx]  = humRH;
 
  gxBuf[idx] = gx;
  gyBuf[idx] = gy;
  gzBuf[idx] = gz;
 
  idx = (idx + 1) % N;
  if (countN < N) countN++;
 
  // ---- Compute averages over last countN samples ----
  float tempAvg = 0, humAvg = 0;
  float gxAvg = 0, gyAvg = 0, gzAvg = 0;
 
  for (int i = 0; i < countN; i++) {
    tempAvg += tempBuf[i];
    humAvg  += humBuf[i];
 
    gxAvg += gxBuf[i];
    gyAvg += gyBuf[i];
    gzAvg += gzBuf[i];
  }
 
  tempAvg /= countN;
  humAvg  /= countN;
 
  gxAvg /= countN;
  gyAvg /= countN;
  gzAvg /= countN;
 
  // ---- Print CURRENT + AVG10 ----
  Serial.print("lastN=");
  Serial.println(countN);
 
  Serial.print("TEMP (C)   CURRENT=");
  Serial.print(tempC, 2);
  Serial.print("   AVG10=");
  Serial.println(tempAvg, 2);
 
  Serial.print("HUM  (%)   CURRENT=");
  Serial.print(humRH, 2);
  Serial.print("   AVG10=");
  Serial.println(humAvg, 2);
 
  Serial.print("GYRO (dps) CURRENT gx=");
  Serial.print(gx, 2);
  Serial.print(" gy=");
  Serial.print(gy, 2);
  Serial.print(" gz=");
  Serial.print(gz, 2);
  Serial.print("   AVG10 gx=");
  Serial.print(gxAvg, 2);
  Serial.print(" gy=");
  Serial.print(gyAvg, 2);
  Serial.print(" gz=");
  Serial.println(gzAvg, 2);
 
  Serial.println();
 
  // ---- Sample once per second ----
  delay(1000);
}