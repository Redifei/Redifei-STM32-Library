/*
 * Redifei: E02_1 MPU6050 Library Example
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 */
#include <stdbool.h>
#include <stm32f10x_conf.h>

#include "systickTimer.h"
#include "i2c.h"
#include "pwm.h"
#include "printf.h"
#include "serial.h"
#include "vcom.h"

#include "mpu6050.h"

serialUartPort_t* serialPort;
vComPort_t* vcomPort;
i2cPort_t* i2cPort;
timPwmPort_t* pwmInPort;
timPwmPort_t* pwmOutPort;

int main(void) {
  // MPU6050 Library Example
  // Config systemTick Timer
  systickTimerConfig();

  // Open Serial Port
  serialPort = openSerial3(SERIAL_INTERRPUT_MODE);
  serialPort->printf("Hello World!\r\n");

  // Config MPU6050
  mpu6050_t* myMPU = mpu6050Config();

  // accDetect function operate to same gyroDetect function
  if (!myMPU->gyroDetect()) {
    serialPort->printf("Not Detected\r\n");
    while (1)
      ;
  }
  myMPU->gyroInit();
  myMPU->accInit();

  serialPort->printf("Detected : 0x%x\r\n", myMPU->deviceAddr);
  delay(500);

  while (1) {
    uint8_t err;
    int16_t buf[3];

    // Read Mpu6050 Acc 3 axis
    err = myMPU->accRead(buf);
    serialPort->printf("%d, %d, %d | %d\r\n", buf[0], buf[1], buf[2], err);
    delay(100);
  }
}
