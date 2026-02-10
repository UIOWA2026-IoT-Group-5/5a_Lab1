#include "RTIMULib.h"
#include <vector>
#include <numeric>

using namespace std;

// Helper to manage the 10-sample moving average
void updateBuffer(vector<float>& buffer, float newValue) {
    if (buffer.size() >= 10) buffer.erase(buffer.begin());
    buffer.push_back(newValue);
}

float getAvg(const vector<float>& buffer) {
    if (buffer.empty()) return 0.0f;
    return accumulate(buffer.begin(), buffer.end(), 0.0f) / buffer.size();
}

int main() {
    int sampleCount = 0;
    int sampleRate = 0;
    uint64_t rateTimer, displayTimer, now;

    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
    RTIMU *imu = RTIMU::createIMU(settings);
    RTPressure *pressure = RTPressure::createPressure(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    imu->IMUInit();
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    if (pressure != NULL) pressure->pressureInit();

    // Buffers for the last 10 samples
    vector<float> rollBuf, pitchBuf, yawBuf, tempBuf;

    rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();

    while (1) {
        // Poll at the recommended rate
        usleep(imu->IMUGetPollInterval() * 1000);

        while (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            if (pressure != NULL) pressure->pressureRead(imuData);

            sampleCount++;
            now = RTMath::currentUSecsSinceEpoch();

            // Execute once per second
            if ((now - displayTimer) > 1000000) {
                // 1. Get current values
                float roll  = imuData.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
                float pitch = imuData.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
                float yaw   = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
                float temp  = imuData.temperature;

                // 2. Update 10-sample buffers
                updateBuffer(rollBuf, roll);
                updateBuffer(pitchBuf, pitch);
                updateBuffer(yawBuf, yaw);
                updateBuffer(tempBuf, temp);

                // 3. Display Current and Average
                printf("\n--- CURRENT VALUES ---\n");
                printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Temp: %6.2f°C\n", roll, pitch, yaw, temp);

                printf("--- 10-SAMPLE AVERAGES ---\n");
                printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Temp: %6.2f°C\n", 
                        getAvg(rollBuf), getAvg(pitchBuf), getAvg(yawBuf), getAvg(tempBuf));

                fflush(stdout);
                displayTimer = now;
            }

            if ((now - rateTimer) > 1000000) {
                sampleRate = sampleCount;
                sampleCount = 0;
                rateTimer = now;
            }
        }
    }
}