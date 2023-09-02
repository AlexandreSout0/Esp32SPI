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





//  void spi_init() {

//     spi_bus_config_t bus_config = {};
//     bus_config.miso_io_num=GPIO_MISO;
//     bus_config.mosi_io_num=GPIO_MOSI;
//     bus_config.sclk_io_num=GPIO_SCLK;
//     bus_config.quadwp_io_num=-1;
//     bus_config.quadhd_io_num=-1;
//     bus_config.max_transfer_sz=0;
    
//     spi_device_interface_config_t dev_config = {};
//     dev_config.clock_speed_hz=10000000;
//     dev_config.mode=1;
//     dev_config.flags = 0;
//     dev_config.duty_cycle_pos = 128; // 50% duty cycle
//     dev_config.spics_io_num=GPIO_CS;
//     dev_config.cs_ena_posttrans=3;
//     dev_config.cs_ena_pretrans=3;
//     dev_config.queue_size=20;
//     dev_config.input_delay_ns = 40,
//     dev_config.pre_cb = NULL;

    
//     spi_bus_initialize(HSPI_HOST,&bus_config,SPI_DMA_CH_AUTO);
//     spi_bus_add_device(HSPI_HOST,&dev_config,&spi);
//     gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
//     gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
//     gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

// }

  
spi_device_handle_t device;  

SPI_t &mySPI = hspi;  // vspi and hspi are the default objects

uint8_t val0[5] = {0x80,0x43,0x0B,0xE8,0x00};
uint8_t val1[5] = {0x81,0x21,0x44,0x40,0x00};
uint8_t val2[5] = {0x82,0xA0,0x13,0x88,0x00};
uint8_t val3[5] = {0x83,0xD0,0xA2,0x48,0x00};
uint8_t val4[5] = {0x84,0x10,0x00,0x40,0x00};
uint8_t val5[5] = {0x85,0x40,0x00,0x00,0x00};
uint8_t val6[5] = {0x86,0xC0,0xC0,0x61,0x00};

void setup()
{
    
   // gpio_set_pull_mode(MOSI_PIN, GPIO_PULLUP_ONLY);
   //  gpio_set_pull_mode(SCLK_PIN, GPIO_PULLUP_ONLY);
   //  gpio_set_pull_mode(CS_PIN, GPIO_PULLUP_ONLY);
   Serial.begin(9600);
}

void loop(){
    uint8_t buffer[1];
   int auxiliar = 0;
   mySPI.begin(MOSI_PIN, MISO_PIN, SCLK_PIN);
   mySPI.addDevice(SPI_MODE, SPI_CLOCK, CS_PIN, &device);
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
  
      //mySPI.readBytes(device, 0x5,2,buffer);
      auxiliar;
      Serial.println(auxiliar);
      vTaskDelay(20 / portTICK_PERIOD_MS);
 



}
