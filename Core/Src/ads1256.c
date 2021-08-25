#include "ads1256.h"
#include "file_handling.h"
#include "main.h"

SPI_HandleTypeDef hspi1;

int DRDY_state = 1;
char *exchange_buffer[1];
char uart_buffer[64];

// -------------------------------------------------------------
// Code to configure and to communicate with ADS1256 through SPI
// -------------------------------------------------------------

void initADS(void) {
	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, 0); // Put RST pin to LOW
	HAL_Delay(10); // LOW at least 4 clock cycles of onboard clock. 100 microseconds is enough
	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, 1); // Put RST pin to HIGH

	HAL_Delay(1000); // Waits approximately 1 second to continue the process

	// Reseting the ADS1256
	ADS1256_Reset();

	// Let the system settle. According to the datasheet, it takes up to 800.4ms. (datasheet pg 24)
	HAL_Delay(1000);

	// The following command activates the entrance buffer. Theoretically, it allows more stable readings
	// SetRegisterValue(STATUS,0b00110010);
	uint8_t RegisterValue = GetRegisterValue(STATUS);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);

	//next set the mux register
	//we are only trying to read differential values from pins 0 and 1. your needs may vary.
	//this is the default setting so we can just reset it
	SetRegisterValue(MUX, 0b00001000); //set the mux register
	//B00001000 for single ended measurement

	//now set the ADCON register
	//set the PGA to 64x
	//you need to adjust the constants for the other ones according to datasheet pg 31 if you need other values
	SetRegisterValue(ADCON, PGA_1); //set the adcon register

	//next set the data rate
	SetRegisterValue(DRATE, DR_1000); //set the drate register

	//we're going to ignore the GPIO for now...

	//lastly, we need to calibrate the system

	//let it settle
	HAL_Delay(2000);

	//then do calibration
	SendCMD(SELFCAL); //send the calibration command

	//then print out the values
	HAL_Delay(5);

	send_uart("\nOFC0: ");
	RegisterValue = GetRegisterValue(OFC0);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
	send_uart("\nOFC1: ");
	RegisterValue = GetRegisterValue(OFC1);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
	send_uart("\nOFC2: ");
	RegisterValue = GetRegisterValue(OFC2);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
	send_uart("\nFSC0: ");
	RegisterValue = GetRegisterValue(FSC0);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
	send_uart("\nFSC1: ");
	RegisterValue = GetRegisterValue(FSC1);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
	send_uart("\nFSC2: ");
	RegisterValue = GetRegisterValue(FSC2);
	sprintf(uart_buffer, "%d\n", RegisterValue);
	send_uart(uart_buffer);
}

// -------------------------------------------------------------
// Some cool interruptions
// -------------------------------------------------------------

void waitforDRDY(void) {
	while (DRDY_state) {
		continue;
	}
	__disable_irq();
	DRDY_state = 1;
	__enable_irq();
}

// Interrupt function
void DRDY_Interuppt(void) {
	DRDY_state = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SPI1_RDY_Pin) {
		// Write your code here
		DRDY_state = 0;
	}
}

// -------------------------------------------------------------
// Function to reset the ADS1256
// -------------------------------------------------------------

void ADS1256_Reset(void) {
	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, 0);
	delayMicroseconds(10);
	exchange_buffer[0] = (char*) RESET1;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); //Reset
	HAL_Delay(2); //Minimum 0.6ms required for Reset to finish.
	exchange_buffer[0] = (char*) SDATAC;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); //Issue SDATAC
	delayMicroseconds(100);
	HAL_GPIO_WritePin(SPI1_SYNC_GPIO_Port, SPI1_SYNC_Pin, 1);
}

// -------------------------------------------------------------
// Function to get the value of a given register
// -------------------------------------------------------------

uint8_t GetRegisterValue(uint8_t regAdress) {
	uint8_t bufr;
	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 0);
	delayMicroseconds(10);
	exchange_buffer[0] = (char*) (RREG | regAdress);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100);
	exchange_buffer[0] = (char*) 0x00;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // send 2nd command byte, read only one register
	delayMicroseconds(10);
	HAL_SPI_Receive(&hspi1, &bufr, 1, 100); // Read the 3 bytes transmitted by the ADS1256
	delayMicroseconds(10);
	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 1);
	return bufr;
//
}

// -------------------------------------------------------------
// Sending commands to ADS1256
// -------------------------------------------------------------

void SendCMD(uint8_t command) {
	waitforDRDY();
	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 0);
	delayMicroseconds(10);
	exchange_buffer[0] = (char*) command;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100);
	delayMicroseconds(10);
	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 1);
}

// -------------------------------------------------------------
// Writing certain values to ADS1256 registers
// -------------------------------------------------------------

void SetRegisterValue(uint8_t regAdress, uint8_t regValue) {

	uint8_t regValuePre = GetRegisterValue(regAdress);
//	send_uart("Cheguei aqui feraaaa!\n");
	HAL_Delay(10);
	if (regValue != regValuePre) {
		//digitalWrite(_START, HIGH);
		delayMicroseconds(10);
//		send_uart("E entrei aqui!\n");
		waitforDRDY(); // Does it work?
		HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 0);
		delayMicroseconds(10);
		exchange_buffer[0] = (char*) (WREG | regAdress);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // 1st write command byte is sent with the register to be written on
		exchange_buffer[0] = (char*) 0x00;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // 2nd command byte, write only one register
		HAL_SPI_Transmit(&hspi1, &regValue, sizeof(regValue), 100); // write data (1 Byte) for the register
		delayMicroseconds(10);
		HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 1);
		//digitalWrite(_START, LOW);
		if (regValue != GetRegisterValue(regAdress)) { //Check if write was succesfull
			send_uart("Escrevendo registrador 0x");
			char regAddress_char[64];
			sprintf(regAddress_char, "%d", regAdress);
			send_uart(regAddress_char);
			send_uart("Falha ao inicializar dispositivo. Reinicie o sistema.\n");
		} else {
			send_uart("Sucesso ao inicializar o dispositivo.\n");
		}
	}

}

// -------------------------------------------------------------
// Reading one value from a given channel
// -------------------------------------------------------------

uint32_t read_Value(uint8_t channel) {
//	send_uart("EU VOU LER HEIN!\n");
//	HAL_Delay(2000);
//	static const uint32_t* adc_val;
	uint8_t adc_val[3];
	uint32_t adc_conv;
//	uint8_t adc_val2;
//	uint8_t adc_val3;
	waitforDRDY(); // Wait until DRDY is LOW
	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 0); // Pull SS Low to Enable Communications with ADS1247
	//delayMicroseconds(5); // RD: Wait 25ns for ADC12xx to get ready

	exchange_buffer[0] = (char*) (WREG | MUX);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // Transmit change of MUX register
	exchange_buffer[0] = (char*) 0x00;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // One byte to be written
//	send_uart("Enviando buffer...\n");
	exchange_buffer[0] = (char*) channel;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // Declaring that we want AIN 7 - AINCOM

	delayMicroseconds(2);

	exchange_buffer[0] = (char*) SYNC;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // Issue the SYNC command
	delayMicroseconds(5);
	exchange_buffer[0] = (char*) WAKEUP;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100); // Issue the wake up command
	delayMicroseconds(1);
	exchange_buffer[0] = (char*) RDATA;
	HAL_SPI_Transmit(&hspi1, (uint8_t*) exchange_buffer, 1, 100);

	delayMicroseconds(8);

//	send_uart("Tentando ler valores...\n");
	HAL_SPI_Receive(&hspi1,  adc_val, 3, 100);
	adc_conv = (uint32_t)adc_val[0] << 24 | (uint32_t)adc_val[1] << 16 | (uint32_t)adc_val[2] << 8;
//	HAL_SPI_Receive(&hspi1, &adc_val[1], 1, 100);
//	HAL_SPI_Receive(&hspi1, &adc_val[2], 1, 100);
//	send_uart("Valores lidos.\n");

	HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, 1); //Pull SS High to Disable Communications with ADS1247

//	if (adc_val3 > 0x7fffff) { //if MSB == 1
//		adc_val3 = adc_val1 - 16777216; //do 2's complement, keep the sign this time!
//	}

//	return (ReverseBytes(adc_val) >> 8);
	return adc_conv >> 8;
}

uint32_t ReverseBytes(uint32_t value) {
	return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8
			| (value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}
