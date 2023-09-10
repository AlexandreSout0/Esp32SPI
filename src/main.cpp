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
#define INT_PIN   GPIO_NUM_15
#define SPI_CLOCK 8000000

// #define SPI_MODE  0
// #define MISO_PIN  GPIO_NUM_17
// #define MOSI_PIN  GPIO_NUM_5
// #define SCLK_PIN  GPIO_NUM_23
// #define CS_PIN    GPIO_NUM_16
// #define SPI_CLOCK 8000000

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

const double HF_CLOCK_FREQ = 4E6;
const double LF_CLOCK_FREQ = 32768;


enum class MEASUREMENT_ERROR {
	NO_ERROR = 0,
	TIMEOUT_START,
	TIMEOUT_STOP,
	OVERFLOW
};

spi_device_handle_t device;
SPI_t &mySPI = hspi;

void teste(){
   uint8_t buffer[2];
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
         mySPI.readBytes(device, 0xB1,2,buffer);
         vTaskDelay(2);
   }
}      

uint32_t GP22::registers_data[7];


void printReg(uint32_t reg[7]) {
  
  Serial.println(F("Reg contains: "));

  for (int i = 0; i<7; i++) {
    Serial.print(F("0x"));
    Serial.println(reg[i], HEX);
  }
  Serial.println(F("***"));
}

void writeConfigReg(uint8_t targetReg, uint32_t data) {
   uint8_t valor[5];
	// Store the 32 bit int in the same memory as 4 bytes
   valor[0] = (TDC_WRITE_TO_REGISTER | targetReg);
   valor[1] = (data >> 24) & 0xFF;
   valor[2] = (data >> 16) & 0xFF;
   valor[3] = (data >> 8) & 0xFF;
   valor[4] = data & 0xFF;

//   Serial.print("Valores em hexadecimal: ");
//   for (int i = 0; i < 5; i++) {
//     Serial.print("0x");
//     Serial.print(valor[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
  mySPI.writeBytes(device,0x80,5,valor);

}

void updateTDC(const uint32_t * registers) {
	// Write the values to the TDC's registers
	for (int i = 0; i < 7; i++) {
		writeConfigReg(i, registers[i]);
	}
}

// Write to the GP22 then read to check communication
uint8_t testTDC() {

   mySPI.writeByte(device, 0x0 , 0x50);
   vTaskDelay(100);

	// Get previous value of reg1's top 8 bits
	uint8_t prevReg1 = GP22::bitmaskRead(GP22::REG1, GP22::REG1_TEST_DATA);

	// Write test data (0xAB) into reg1
	const uint8_t testdata = 0xBB;
	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_TEST_DATA, testdata);
	updateTDC(GP22::registers_data);

	// Read back from the first 8 bits of register 1 (should match testdata)
   uint8_t buffer[2];
   mySPI.readBytes(device,TDC_REG5,2,buffer);

	// Restore settings to previous
	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_TEST_DATA, prevReg1);
	updateTDC(GP22::registers_data);

	// Return 0 for success
	if (buffer[1] == testdata)
		return 0;

	// If we failed, return the value we just read, unless that value is 0 in which case return "0xFF"
	else
		if (buffer != 0)
			return buffer[1];
		else
			return 0xFF;
}

void testConnection() {

	// Run the test
	uint8_t testResult = testTDC();

	// Report the result
	if (testResult) {
		Serial.print("FAILED with code returned 0x");
		Serial.println(testResult, HEX);
	}
	else {
		Serial.println("PASSED");
	}
}

inline bool inMeasurementMode2() {
	return bitmaskRead(GP22::REG0, GP22::REG0_MESSB2);
}

void setupForSTARTToSTOP1Mode() {

   // Ler o tempo desde o trigger em START até o trigger em STOP1
   // A configuração para isso depende se o GP22 está no modo 1 ou modo 2
	if (inMeasurementMode2()) { // mode 2
		bitmaskWrite(GP22::REG1, GP22::REG1_HIT1, 1);
		bitmaskWrite(GP22::REG1, GP22::REG1_HIT2, 2);
		//Configura o número de hits esperados
		bitmaskWrite(GP22::REG1, GP22::REG1_HITIN1, 2); // 1 hit on STOP1 + 1 on START = 2
	} else { // mode 1
		bitmaskWrite(GP22::REG1, GP22::REG1_HIT1, 1);
		bitmaskWrite(GP22::REG1, GP22::REG1_HIT2, 0);
		//Configura o número de hits esperados
		bitmaskWrite(GP22::REG1, GP22::REG1_HITIN1, 1); // 1 hit on STOP1
	}

	bitmaskWrite(GP22::REG1, GP22::REG1_HITIN2, 0); // No hits on STOP2

	// Trigger interrupt on timeout or finishing calculation
	bitmaskWrite(GP22::REG2, GP22::REG2_EN_INT_ALU, 1);
	bitmaskWrite(GP22::REG2, GP22::REG2_EN_INT_TDC_TIMEOUT, 1);

   // Modo de resolução dupla desativado, pois limita o tempo máximo do modo 1 à metade
	bitmaskWrite(GP22::REG6, GP22::REG6_DOUBLE_RES, false);
	bitmaskWrite(GP22::REG6, GP22::REG6_QUAD_RES, false);

	// Update the TDC's settings
	updateTDC(GP22::registers_data);
}


// Realiza uma única medição e saída para a variável passada
// As leituras uint32_t retornadas por esta função são obtidas diretamente do GP22
// Registradores de saída. Eles representam um número inteiro assinado e bruto de portas passadas para uma medição,
// ou um tempo de ponto flutuante calibrado com base no relógio interno, dependendo do estado
// dos registradores do GP22.
// Esta função não se importa: ela retorna o valor como um unsigned int de 32 bits não assinado. Isso é
// cabe ao usuário converter para qualquer formato aplicável.
//
// Parâmetros:
// &out - Variável de saída para o resultado da medição
// &completionTime - Variável de saída para o contador de milissegundos quando a medição for concluída
// tempo limite (padrão 500ms) - tempo limite para esperar antes de emitir um erro TIMEOUT_START
//
// Retorna: enum MEASUREMENT_ERROR listando o status do erro, se houver

void resetCommand(){
   mySPI.writeByte(device, 0x0 , 0x50);
   vTaskDelay(100);
   
}

void initCommand(){
   mySPI.writeByte(device, 0x0, 0x70);
   vTaskDelay(100);
}

void sentOpcode(uint8_t opcode){
   mySPI.writeByte(device,0x0,opcode);
}

//Lê o resultado
// O formato do dispositivo é um número de ponto fixo de 32 bits com 16 bits para o
// partes fracionadas. O seguinte lê 16 ou 32 bits, primeiro os MSBs.
// Se for 16, o valor de retorno será 0x0000RRRR onde RRRR são os bytes do resultado
// Se os 16 bits forem 0xFFFF, o valor de retorno será 0xFFFFFFFF
uint32_t read_bytes(uint8_t reg, bool read16bits){
   
   size_t len = 4;
   uint8_t val[len];
   
   mySPI.readBytes(device,reg,len,val);

  Serial.print("read_bytes: ");
  for (int i = 1; i < len + 1; i++) {
    Serial.print("0x");
    Serial.print(val[i], HEX);
    Serial.print(" ");
  }
   Serial.print("\r\n");

   uint32_t result = (uint32_t)val[1] << 24 | (uint32_t)val[2] << 16 | (uint32_t)val[3] << 8 | val[4];
    Serial.print(result, HEX);
	return result;
}



void read_bytes2(uint8_t reg, size_t len){
   
   uint8_t val[len + 1];
   for (int i = 1; i < len + 1; i++) {
      val[i] = 0xFF;
   }
   for (int i = 1; i < len + 1; i++) {
    Serial.print("0x");
    Serial.print(val[i], HEX);
    Serial.print(" ");
  }
   Serial.print("\r\n");
   
   mySPI.readBytes(device,reg,len,val);

  Serial.print("read_bytes: ");
  for (int i = 1; i < len + 1; i++) {
    Serial.print("0x");
    Serial.print(val[i], HEX);
    Serial.print(" ");
  }
   Serial.print("\r\n");
   
}


//Lê o status
// O formato do dispositivo é um número de 16 bits
// Veja a página 36 da ficha técnica do ACAM
uint16_t readStatus(){

	// Read in the data
	uint16_t status = read_bytes(TDC_STATUS, true);

	return status;
}

uint32_t regBackup[7];
void backup_config(){
	for (int i = 0; i < 7; i++) {
		regBackup[i] = regRead(GP22::registers(i));
	}
}

void restore_config(){
	for (int i = 0; i < 7; i++) {
		regWrite(GP22::registers(i), regBackup[i]);
	}
	updateTDC(GP22::registers_data);
}

void dump_config(){
	for (int i = 0; i < 7; i++) {
		Serial.print("0x");
		Serial.println(GP22::registers_data[i], HEX);
	}
}

MEASUREMENT_ERROR measure(uint32_t& out, unsigned int timeout, unsigned long& completionTime_ms, unsigned long& completionTime_us) {

	// Envie o opcode INIT para começar a esperar por um evento de temporização
   initCommand();

	// Se estivermos no modo FIRE_START, dispare um pulso inicial
	if (bitmaskRead(GP22::REG1, GP22::REG1_SEL_START_FIRE)) {
      sentOpcode(TDC_START_TOF);
	}

	// Espere até que a interrupção diminua, indicando uma leitura bem-sucedida
	uint64_t start = esp_timer_get_time()*1000;

	while (gpio_get_level(INT_PIN) == 1) {
		// Give up if we've been waiting <timeout>ms 
		if (esp_timer_get_time()*1000 - start > timeout) {
			return MEASUREMENT_ERROR::TIMEOUT_START;
		} 
	}

	// Salve o tempo
	completionTime_us = micros()*1000;
	completionTime_ms = esp_timer_get_time();

	// Leia o resultado
	out = read_bytes(TDC_RESULT1, !(bool)bitmaskRead(GP22::REG0, GP22::REG0_CALIBRATE));

	// Obtenha o status
	uint16_t TDC_stat = readStatus();

	// const int nextAdr = TDC_stat & 0b111;
	const bool stopTimeout = TDC_stat & ((1<<9)|(1<<10));
	const bool counterOverflow = out == 0xFFFFFFFF;

	if (stopTimeout)
		return MEASUREMENT_ERROR::TIMEOUT_STOP;
	if (counterOverflow)
		return MEASUREMENT_ERROR::OVERFLOW;

	return MEASUREMENT_ERROR::NO_ERROR;
}


//Executa uma rotina de calibração e então retorna o número de LSBs em 2 ciclos de clock
// A configuração de clock padrão é 4 MHz, então uma medição de x LSBs em 2 ciclos de clock corresponde a
// uma precisão de 1/4 MHz * 2 ciclos / x
uint16_t calibrate() {

   // Esta sequência é adaptada do código fonte do software de avaliação ACAM (em Labview)
	// Backup registers
	backup_config();

	// Goto quad res. mode
	GP22::bitmaskWrite(GP22::REG0, GP22::REG0_MESSB2, true); // Mes. mode. 2
	GP22::bitmaskWrite(GP22::REG0, GP22::REG0_CALIBRATE, false); // No auto cal
	GP22::bitmaskWrite(GP22::REG6, GP22::REG6_QUAD_RES, true); // Quad res on
	GP22::bitmaskWrite(GP22::REG6, GP22::REG6_DOUBLE_RES, false); // Double res off
	updateTDC(GP22::registers_data);

   // Envia INIT para que o TDC esteja pronto para dar uma resposta
   initCommand();

   // Envia o opcode START_CAL_TDC para medir os dados de calibração
   sentOpcode(TDC_START_CAL);

   // Solicita que a ALU calcule a diferença de calibração escrevendo
   // no registro 1. Isso informa à ALU o que calcular e também aciona o cálculo
   // Veja p.52 do manual ACAM
   // Nosso cálculo é CALI2 - CALI1 == T_ref
	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_HIT1, 7); // Request Cal2...
	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_HIT2, 6); // ...minus Cal1

	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_HITIN1, 0); // Expect 0 hits
	GP22::bitmaskWrite(GP22::REG1, GP22::REG1_HITIN2, 0);

	writeConfigReg(GP22::REG1, regRead(GP22::REG1)); // Go!
	vTaskDelay(1);

   // Lê ALU_PTR do status e subtrai 1 para obter a localização do último escrito
   // medição (a calibração)
	uint8_t storageLocation = (readStatus() & 0x7) - 1;

	// Ler resultado
	uint16_t calibration;

	// Verifique se realmente fizemos uma medição. Se ALU_PTR fosse 0, ALU_PTR-1 == 0xFF e falhamos
	if (storageLocation == 0xFF) { calibration = 0xFFFF; } // Return error
	else { // Read and return data
		calibration = read_bytes(storageLocation, true);
	}

	// Restore registers
	restore_config();

	return calibration;
}

uint32_t calibrateHF() {

	// Backup registers
	backup_config();

	// Set EN_AUTOCALC=0
	GP22::bitmaskWrite(GP22::REG3, GP22::REG3_EN_AUTOCALC_MB2, 0);
	// Turn on calibration
	GP22::bitmaskWrite(GP22::REG0, GP22::REG0_CALIBRATE, 1);

	updateTDC(GP22::registers_data);

	// Init
   initCommand();

	// Start the calibration
   sentOpcode(TDC_START_CAL_RES);


	// Espere até que a interrupção diminua, indicando uma leitura bem-sucedida
	uint32_t start = esp_timer_get_time()*1000;
	bool timeout = false;
	while (gpio_get_level(INT_PIN) == 1) {    
		if (esp_timer_get_time()*1000 - start > 500){ 
         timeout=true; break; 
      } // Give up if we've been waiting 500ms
   }
   // O intervalo de tempo a ser medido é definido por ANZ_PER_CALRES
   // que define o número de períodos do clock de 32,768 kHz:
   // 2 períodos = 61,03515625 nós
   // A calibração está ativada, então o GP22 emitirá uma resposta em Hz para a frequência medida do oscilador
	uint32_t result = timeout ? 0xFFFFFFFF : read_bytes(TDC_RESULT1, false);

	// Restore previous config
	restore_config();

	return result;
}


void setMeasurementMode2(bool enabled){

	// Enable / disable measurement mode 2
	bitmaskWrite(GP22::REG0, GP22::REG0_MESSB2, enabled);

	if (enabled) {
		// Calibration must be on in mode 2
		// We'll also turn on autocalibration (calibrates after every shot)
		bitmaskWrite(GP22::REG0, GP22::REG0_CALIBRATE, 	true);
		bitmaskWrite(GP22::REG0, GP22::REG0_NO_CAL_AUTO, false);
	}

	// We must call the setup again, since other settings need to be changed between modes
	setupForSTARTToSTOP1Mode();

	// Send settings
	updateTDC(GP22::registers_data);

}

void fireStartMode(bool enabled){

	// In both cases...
	// Generate 1 pulse only
	bitmaskWrite(GP22::REG0, GP22::REG0_ANZ_FIRE_LSB, 1);
	// Set pulse generator divider to 2 (lowest available)
	bitmaskWrite(GP22::REG0, GP22::REG0_DIV_FIRE, 1);

	if (enabled) {
		// Connect START to the pulse generator
		bitmaskWrite(GP22::REG1, GP22::REG1_SEL_START_FIRE, 1);
		// Output pulses from FIRE_UP (which is sent to START)
		bitmaskWrite(GP22::REG5, GP22::REG5_CONF_FIRE, 2);
	} else {
		// Don't connect START to the pulse generator
		bitmaskWrite(GP22::REG1, GP22::REG1_SEL_START_FIRE, 0);
		// Don't output any pulses
		bitmaskWrite(GP22::REG5, GP22::REG5_CONF_FIRE, 0);
	}
	
	updateTDC(GP22::registers_data);
}

// Retorna a frequência do clock de alta velocidade, levando em consideração quaisquer configurações de divisão
inline double HSClockFreq(){
	uint8_t START_CLKHS = bitmaskRead(GP22::REG0, GP22::REG0_DIV_CLKHS);
	switch (START_CLKHS) {
		case 0:
			return HF_CLOCK_FREQ;
		case 1:
			return HF_CLOCK_FREQ/2.0;
		default:
			return HF_CLOCK_FREQ/4.0;
	}
}



__attribute__((unused)) void setup(){
    
   Serial.begin(115200);
   gpio_pad_select_gpio(INT_PIN); //Configuração do pino
   gpio_set_direction(INT_PIN, GPIO_MODE_INPUT); //Direção do pino
   

   mySPI.begin(MOSI_PIN, MISO_PIN, SCLK_PIN);
   mySPI.addDevice(SPI_MODE, SPI_CLOCK, CS_PIN, &device);

   //printReg(GP22::registers_data);
	regWrite(GP22::REG0, 0x2206680C);
	regWrite(GP22::REG1, 0x5540000F);
	regWrite(GP22::REG2, 0x2000000A);
	regWrite(GP22::REG3, 0x1800000B);
	regWrite(GP22::REG4, 0x20000001);
	regWrite(GP22::REG5, 0x00000002);
	regWrite(GP22::REG6, 0x00000003);
   printReg(GP22::registers_data);


   resetCommand();
   initCommand();
   bitmaskWrite(GP22::REG0, GP22::REG0_DIV_CLKHS, 2);
	updateTDC(GP22::registers_data);

   setupForSTARTToSTOP1Mode();
}

__attribute__((unused)) void loop() {

   read_bytes(TDC_STATUS, 4);

   //testConnection();
   //Serial.println(status,HEX);
   while(true){

      vTaskDelay(2000 / portTICK_PERIOD_MS);
   }

}
