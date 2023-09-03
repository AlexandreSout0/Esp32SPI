#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "SPIbus.h"

#include "defines.h"

#define TIMEOUT_RESET                  100

#define SPI_MODE  1
#define MISO_PIN  GPIO_NUM_27
#define MOSI_PIN  GPIO_NUM_14
#define SCLK_PIN  GPIO_NUM_26
#define CS_PIN    GPIO_NUM_25
#define SPI_CLOCK 8000000

// #define SPI_MODE  0
// #define MISO_PIN  GPIO_NUM_17
// #define MOSI_PIN  GPIO_NUM_5
// #define SCLK_PIN  GPIO_NUM_23
// #define CS_PIN    GPIO_NUM_16
// #define SPI_CLOCK 8000000

  
spi_device_handle_t device;
SPI_t &mySPI = hspi;

uint8_t val0[5] = {0x80,0x22,0x06,0x68,0x0C};
uint8_t val1[5] = {0x81,0x55,0x40,0x00,0xF0};
uint8_t val2[5] = {0x82,0x20,0x00,0x00,0x0A};
uint8_t val3[5] = {0x83,0x18,0x00,0x00,0x0B};
uint8_t val4[5] = {0x84,0x20,0x00,0x40,0x01};
uint8_t val5[5] = {0x85,0x00,0x00,0x00,0x02};
uint8_t val6[5] = {0x86,0x00,0x00,0x00,0x03};


uint32_t registers_data[7] = { 0 };

#define TDC_WRITE_TO_REGISTER 0x80
#define TDC_READ_FROM_REGISTER 0xB0
#define TDC_REG0 0x00
#define TDC_REG1 0x01
#define TDC_REG2 0x02
#define TDC_REG3 0x03
#define TDC_REG4 0x04
#define TDC_REG5 0x05
#define TDC_REG6 0x06
#define TDC_STATUS 0x04
#define TDC_RESULT1 0x00
#define TDC_RESULT2 0x01
#define TDC_RESULT3 0x02
#define TDC_RESULT4 0x03

#define TDC_READ_CONFIG_FROM_EEPROM 0xF0
#define TDC_INIT 0x70
#define TDC_RESET 0x50
#define TDC_START_CAL 0x04
#define TDC_START_CAL_RES 0x03
#define TDC_START_TOF 0x01

enum class MEASUREMENT_ERROR {
	NO_ERROR = 0,
	TIMEOUT_START,
	TIMEOUT_STOP,
	OVERFLOW
};

void GP22_init(){

   mySPI.writeByte(device, 0x0 , 0x50);
   vTaskDelay(250 / portTICK_PERIOD_MS);
   mySPI.writeByte(device, 0x0, 0x70);
   vTaskDelay(250 / portTICK_PERIOD_MS);

   mySPI.writeBytes(device,0x80,5,val0);
   mySPI.writeBytes(device,0x81,5,val1); 
   mySPI.writeBytes(device,0x83,5,val3);
   mySPI.writeBytes(device,0x84,5,val4);
   mySPI.writeBytes(device,0x85,5,val5);
   mySPI.writeBytes(device,0x86,5,val6);
   vTaskDelay(25 / portTICK_PERIOD_MS);

}

void teste(){
   uint8_t buffer[2];
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
         mySPI.readBytes(device, 0xB1,2,buffer);
         vTaskDelay(2);
      }
   }      

__attribute__((unused)) void setup(){
    
   Serial.begin(9600);
   
   mySPI.begin(MOSI_PIN, MISO_PIN, SCLK_PIN);
   mySPI.addDevice(SPI_MODE, SPI_CLOCK, CS_PIN, &device);
}


__attribute__((unused)) void loop() {

   GP22_init();
   teste();

   while(true){


      vTaskDelay(2000 / portTICK_PERIOD_MS);
   }



}
