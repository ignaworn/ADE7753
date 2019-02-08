/* ADE7753.h
====================================================================================================
v0.2
fng
By: Ezequiel Pedace & Ignacio Worn
Created:     7 Dic 2012
Last update: 02 Feb 2019

ADE7753 library for esp32 w/ esp-idf framework
*/

#ifndef ADE7753_H
#define ADE7753_H

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
/*
=================================================================================================
Defines
=================================================================================================
*/

// Class Atributes

// Chip Select ADE7753 se le suma luego 8 para compatibilizar con la funcion digitalWrite()
#define WRITE       0x80			// Valor del addres para la funcion Write. // TODO: No_arduino
#define CLKIN       3579545         // ADE7753 freq. (4MHz max)
#define PERIODO     50				// Frecuencia de Red

// Default Pins
#define DEF_DOUT GPIO_NUM_12
#define DEF_DIN GPIO_NUM_13
#define DEF_SCLK GPIO_NUM_14
#define DEF_CS GPIO_NUM_15
#define DEF_SPI_FREQ 1E6

// Register address

//      Name        Address         Length
#define WAVEFORM 	0x01            //  24
#define AENERGY 	0x02            //  24
#define RAENERGY  	0x03            //  24
#define LAENERGY 	0x04            //  24
#define VAENERGY 	0x05            //  24
#define RVAENERGY 	0x06            //  24
#define LVAENERGY 	0x07            //  24
#define LVARENERGY 	0x08            //  24
#define MODE 		0x09            //  16
#define IRQEN 		0x0A            //  16
#define STATUSR 	0x0B            //  16
#define RSTSTATUS 	0x0C            //  16
#define CH1OS 		0x0D            //  8
#define CH2OS 		0x0E            //  8
#define GAIN 		0x0F            //  8
#define PHCAL 		0x10            //  6
#define APOS 		0x11            //  16
#define WGAIN 		0x12            //  12
#define WDIV 		0x13            //  8
#define CFNUM 		0x14            //  12
#define CFDEN 		0x15            //  12
#define IRMS 		0x16            //  24
#define VRMS 		0x17            //  24
#define IRMSOS 		0x18            //  12
#define VRMSOS 		0x19            //  12
#define VAGAIN 		0x1A            //  12
#define VADIV 		0x1B            //  8
#define LINECYC 	0x1C            //  16
#define ZXTOUT 		0x1D            //  12
#define SAGCYC 		0x1E            //  8
#define SAGLVL 		0x1F            //  8
#define IPKLVL 		0x20            //  8
#define VPKLVL 		0x21            //  8
#define IPEAK 		0x22            //  24
#define RSTIPEAK 	0x23            //  24
#define VPEAK 		0x24            //  24
#define RSTVPEAK  	0x25            //  24
#define TEMP 		0x26            //  8
#define PERIOD 		0x27            //  16
#define TMODE 		0x3D            //  8
#define CHKSUM 		0x3E            //  6
#define DIEREV 		0x3F            //  8

// MODE REGISTER (0x09)

#define DISHPF      0x01 << 0
#define DISLPF2     0x01 << 1
#define DISCF       0x01 << 2
#define DISSAG      0x01 << 3
#define ASUSPEND    0x01 << 4
#define TEMPSEL     0x01 << 5
#define SWRST       0x01 << 6
#define CYCMODE     0x01 << 7
#define DISCH1      0x01 << 8
#define DISCH2      0x01 << 9
#define SWAP	    0x01 << 10
#define DTRT1       0x01 << 11
#define DTRT0       0x01 << 12
#define WAVSEL1     0x01 << 13
#define WAVSEL0     0x01 << 14
#define POAM        0x01 << 15

// INTERRUPT REGISTERS (0x0A - 0x0C)

#define AEHF	    0x01 << 0
#define SAG		    0x01 << 1
#define CYCEND	    0x01 << 2
#define WSMP	    0x01 << 3
#define ZX		    0x01 << 4
#define TEMPC	    0x01 << 5
#define RESET	    0x01 << 6
#define AEOF	    0x01 << 7
#define PKV		    0x01 << 8
#define PKI		    0x01 << 9
#define VAEHF	    0x01 << 10
#define VAEOF	    0x01 << 11
#define ZXTO	    0x01 << 12
#define PPOS	    0x01 << 13
#define PNEG	    0x01 << 14

// CH1OS/CH2OS REGISTER (0x0D - 0x0E)

#define INTEGRATOR  0x01 << 7

// GAIN REGISTER (0x0F)

#define GAIN_1      0
#define GAIN_2      1
#define GAIN_4      2
#define GAIN_8      3
#define GAIN_16     4
#define INTEGRATOR_ON   1
#define INTEGRATOR_OFF  0
#define FULLSCALESELECT_0_5V    0
#define FULLSCALESELECT_0_25V   1
#define FULLSCALESELECT_0_125V  2


class ADE7753 {

	// Public methods
	public:

		ADE7753();
		~ADE7753();

		/**
		* @brief SPI configuration
		* 
		* @param DOUT, Master In Serial Out (MISO) Pin
		* @param DIN, Master Out Serial In (MOSI) Pin
		* @param SCLK, Serial Clock (CLK) Pin
		* @param CS, Chip Select (CS) Pin
		* @param spiFreq, SPI Data transfer frequency
		* 
		*/
		void configSPI(gpio_num_t DOUT, gpio_num_t DIN, gpio_num_t SCLK, gpio_num_t CS, int spiFreq);

		void closeSPI(void);

		//----------------------------------------------------------------------------
		// Modos y configs
		//----------------------------------------------------------------------------

		/**
		 * @brief Die Revision Register.
		 *
		 * @return revision number of the silicon.
		 * 
		 */
		uint8_t getVersion();

		void setMode(uint16_t m);
		uint16_t getMode();
		void gainSetup(uint8_t integrator, uint8_t scale, uint8_t PGA2, uint8_t PGA1);
		uint16_t getStatus(void);
		uint16_t resetStatus(void);
		uint32_t getIRMS(void);
		uint32_t getVRMS(void);
		float vrms();
		float irms();
		uint16_t getPeriod(void);
		float getFrecuency(void);
		void setLineCyc(uint16_t d);
		void setZeroCrossingTimeout(uint16_t d);
		uint16_t getZeroCrossingTimeout();
		uint8_t getSagCycles();
		void setSagCycles(uint8_t d);
		uint8_t getSagVoltageLevel();
		void setSagVoltageLevel(uint8_t d);
		uint8_t getIPeakLevel();
		void setIPeakLevel(uint8_t d);
		uint8_t getVPeakLevel();
		void setVPeakLevel(uint8_t d);
		uint32_t getIpeakReset(void);
		uint32_t getVpeakReset(void);
		uint8_t setPotLine(uint16_t Ciclos);
		uint32_t getWatt(void);
		uint32_t getVar(void);


		/**
		 * @brief 
		 * 
		 * @param vconst float number different from zero.
		 * 
		 * @return
		 *         - ESP_OK                on success
		 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
		*/
		esp_err_t setVconst(float vconst);


		/**
		 * @brief 
		 * 
		 * @param vconst float number different from zero.
		 * 
		 * @return
		 *         - ESP_OK                on success
		 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
		*/   
		esp_err_t setIconst(float iconst);


		void setReadingsNum(uint8_t readingsNum);


		uint32_t getVa(void);


		/**
		 * @brief Get Interrupt Enable Register from ADE7753
		 * 
		 * @return Interrupt enable register values
		 */
		uint16_t getInterrupt(void);


		/**
		 * @brief
		 * 
		 * @param  
		 */
		void setInterrupt(uint16_t reg);


		/**
		 * @brief Interrupt function handler
		 * 
		 * When an interrupt IRQ is triggered, a call to 
		 * this method should be made.
		 * 
		 * @param func pointer to user defined function to 
		 * 		handle ISR. An uint16_t argument is passed 
		 * 		containing the interrupt bits from RSTSTATUS 
		 */
		uint16_t IRQHandler(void);


	// Private methods
	private:

		/**
		 * @brief Send a command/data to ADE7753
		 * 
		 * @param data 8 bits of data to send
		 * 
		 * @return      
		 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
		 *         - ESP_OK                on success
		 * 
		*/
		esp_err_t send(uint8_t data);

		/**
		 * @brief Receive data from ADE7753
		 * 
		 * @return 8 bit data from ADE7753
		 * 
		*/
		uint8_t receive(void);

		/**
		 * @brief Read 8 bits from the device at specified register
		 * 
		 * @param reg register direction
		 * 
		 * @return 8 bit contents of register
		 *
		 */
		uint8_t read8(uint8_t reg);

		/**
		 * @brief Read 16 bits from the device at specified register
		 * 
		 * @param reg register direction
		 * 
		 * @return 16 bit contents of register
		 *
		 */
		uint16_t read16(uint8_t reg);

		/**
		 * @brief Read 24 bits from the device at specified register
		 * 
		 * @param reg register direction
		 * 
		 * @return 24 bit contents of register
		 *
		 */
		uint32_t read24(uint8_t reg);

		/**
		 * @brief Write 8 bits to the device at specified register
		 * 
		 * @param reg char containing register direction
		 * 
		 * @param data char, 8 bits of data to send
		 *
		 * @return 
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t write8(uint8_t reg, uint8_t data);

		/**
		 * @brief Write 16 bits to the device at specified register
		 * 
		 * @param reg register direction
		 * 
		 * @param data 16 bits of data to send
		 *
		 * @return 
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t write16(uint8_t reg, uint16_t data);

		/** 
		 * @brief Enable chip by setting LOW on Chip Select Pin
		 * 
		 * @return
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG GPIO number error
		 */
		esp_err_t enableChip();

		/**
		 * @brief Disable chip by setting HIGH Chip Select pin
		 * 
		 * @return
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG GPIO number error
		 */
		esp_err_t disableChip();

		// SPI Object Handler
		spi_device_handle_t _SPI;

		// SPI default pins 
		gpio_num_t _DOUT = DEF_DOUT;
		gpio_num_t _DIN = DEF_DIN;
		gpio_num_t _SCLK = DEF_SCLK;
		gpio_num_t _CS = DEF_CS;

		// SPI default frequency
		int _spiFreq = DEF_SPI_FREQ;

		uint8_t _readingsNum = 2;
		float _vconst = 1;
		float _iconst = 1;

};

#endif