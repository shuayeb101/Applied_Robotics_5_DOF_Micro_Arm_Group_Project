
#include "main.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// I2C and UART handles
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#define PCA9685_ADDRESS 0x40 // I2C address for PCA9685
#define SERVO_MIN 150        // Minimum pulse length
#define SERVO_MID 300        // Mid pulse length
#define SERVO_MAX 600        // Maximum pulse length

const int numServos = 5;
int currentServoPos[numServos] = {90, 90, 90, 90, 90};  // Initial positions
const int servoLimits[numServos][2] = {{0, 180}, {10, 170}, {0, 180}, {0, 180}, {30, 150}};

// Link lengths (for inverse kinematics)
float link1 = 10.0;
float link2 = 10.0;

int speedDelay = 10;  // Speed control delay

// Function Prototypes
void moveServoSmooth(int servoNum, int targetPos);
void inverseKinematics(float x, float y);
void sendSerial(char *msg);
void pca9685_setPWM(uint8_t channel, uint16_t on, uint16_t off);

// Function to set the PWM on the PCA9685 using I2C
void pca9685_setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[5];
    data[0] = 0x06 + 4 * channel; // Set channel
    data[1] = on & 0xFF;
    data[2] = on >> 8;
    data[3] = off & 0xFF;
    data[4] = off >> 8;
    HAL_I2C_Master_Transmit(&hi2c1, PCA9685_ADDRESS << 1, data, 5, HAL_MAX_DELAY);
}

// Function to smoothly move a servo
void moveServoSmooth(int servoNum, int targetPos) {
    int currentPos = currentServoPos[servoNum];
    targetPos = (targetPos < servoLimits[servoNum][0]) ? servoLimits[servoNum][0] : (targetPos > servoLimits[servoNum][1]) ? servoLimits[servoNum][1] : targetPos;

    while (currentPos != targetPos) {
        if (currentPos < targetPos) {
            currentPos++;
        } else if (currentPos > targetPos) {
            currentPos--;
        }

        int pulse = (currentPos * (SERVO_MAX - SERVO_MIN)) / 180 + SERVO_MIN;
        pca9685_setPWM(servoNum, 0, pulse);
        currentServoPos[servoNum] = currentPos;
        HAL_Delay(speedDelay);
    }
}

// Inverse Kinematics for 2-link planar arm
void inverseKinematics(float x, float y) {
    float d = sqrt(x * x + y * y);  // Distance from base to end-effector
    if (d > (link1 + link2) || d < fabs(link1 - link2)) {
        sendSerial("Target unreachable\n");
        return;
    }

    float angle2 = acos((x * x + y * y - link1 * link1 - link2 * link2) / (2 * link1 * link2));  // Elbow angle
    float angle1 = atan2(y, x) - atan2(link2 * sin(angle2), link1 + link2 * cos(angle2));  // Shoulder angle

    int servoAngle1 = (int)(angle1 * 180.0 / M_PI);
    int servoAngle2 = (int)(angle2 * 180.0 / M_PI);

    moveServoSmooth(2, servoAngle1);
    moveServoSmooth(3, servoAngle2);
}

// Safety mechanism to prevent out-of-bounds movements
int isSafePosition(float x, float y) {
    if (x < 0 || y < 0 || x > (link1 + link2) || y > 10) {
        sendSerial("Unsafe position!\n");
        return 0;
    }
    return 1;
}

// Function to handle incoming commands from UART
void handleCommand(char *input) {
    if (input[0] == 'S') {
        int servoNum = input[1] - '0';  // Extract servo number
        int servoValue = atoi(&input[3]);  // Extract angle value
        if (servoNum >= 0 && servoNum < numServos) {
            moveServoSmooth(servoNum, servoValue);
        } else {
            sendSerial("Invalid servo number\n");
        }
    } else {
        sendSerial("Invalid command\n");
    }
}

// Function to send data over UART
void sendSerial(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// Main setup function
void setup() {
    sendSerial("Ready to receive commands. Use format: S<num> <angle>\n");
    HAL_Delay(1000);  // Wait for initialization
}

// Main loop function
void loop() {
    char inputBuffer[100];
    while (1) {
        if (HAL_UART_Receive(&huart1, (uint8_t *)inputBuffer, sizeof(inputBuffer), HAL_MAX_DELAY) == HAL_OK) {
            inputBuffer[strcspn(inputBuffer, "\r\n")] = 0;  // Remove any trailing newlines
            handleCommand(inputBuffer);
        }
    }
}
