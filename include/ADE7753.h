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

#define MAXWAVEFORMDATA 10000
#define MAXSAMPLECYCLES 10

// Chip Select ADE7753 se le suma luego 8 para compatibilizar con la funcion digitalWrite()
#define WRITE       0x80			// Valor del addres para la funcion Write. // TODO: No_arduino
#define CLKIN       3579545         // ADE7753 freq. (4MHz max)
#define PERIODO     50				// Frecuencia de Red

// Default Pins
#define DEF_DOUT GPIO_NUM_12
#define DEF_DIN GPIO_NUM_13
#define DEF_SCLK GPIO_NUM_14
#define DEF_CS GPIO_NUM_15
#define DEF_SPI_FREQ 8E6

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

#define DISHPF_BIT      (0x01 << 0)
#define DISLPF2_BIT     (0x01 << 1)
#define DISCF_BIT       (0x01 << 2)
#define DISSAG_BIT      (0x01 << 3)
#define ASUSPEND_BIT    (0x01 << 4)
#define TEMPSEL_BIT     (0x01 << 5)
#define SWRST_BIT       (0x01 << 6)
#define CYCMODE_BIT     (0x01 << 7)
#define DISCH1_BIT      (0x01 << 8)
#define DISCH2_BIT      (0x01 << 9)
#define SWAP_BIT	    (0x01 << 10)
#define DTRT_BIT    	(0x01 << 12 | 0x01 << 11)
#define WAVSEL_BIT  	(0x01 << 14 | 0x01 << 13)
#define POAM_BIT        (0x01 << 15)

#define DTRT_27_9		(0x00 << 11)
#define DTRT_14			(0x01 << 11)
#define DTRT_7			(0x02 << 11)
#define DTRT_3_5		(0x03 << 11)

#define WAVESEL_ACTIVE_POWER_SIGNAL ((0x00 << 0 | 0x00 << 1) << 13)
#define WAVESEL_RESERVED 	((0x00 << 1 | 0x01 << 0) << 13)
#define WAVESEL_CH1			((0x01 << 1 | 0x00 << 0) << 13)
#define WAVESEL_CH2 		((0x01 << 1 | 0x01 << 0) << 13)

// INTERRUPT REGISTERS (0x0A - 0x0C)

#define AEHF_BIT	    (0x01 << 0)
#define SAG_BIT		    (0x01 << 1)
#define CYCEND_BIT	    (0x01 << 2)
#define WSMP_BIT	    (0x01 << 3)
#define ZX_BIT		    (0x01 << 4)
#define TEMPC_BIT	    (0x01 << 5)
#define RESET_BIT	    (0x01 << 6)
#define AEOF_BIT	    (0x01 << 7)
#define PKV_BIT		    (0x01 << 8)
#define PKI_BIT		    (0x01 << 9)
#define VAEHF_BIT	    (0x01 << 10)
#define VAEOF_BIT	    (0x01 << 11)
#define ZXTO_BIT	    (0x01 << 12)
#define PPOS_BIT	    (0x01 << 13)
#define PNEG_BIT	    (0x01 << 14)

// CH1OS/CH2OS REGISTER (0x0D - 0x0E)

#define INTEGRATOR_BIT  (0x01 << 7)

// GAIN REGISTER (0x0F)

#define PGA1_BIT		(0x01 << 2 | 0x01 << 1 | 0x01 << 0)
#define SCALE_BIT		(0x01 << 4 | 0x01 << 3)
#define PGA2_BIT		(0x01 << 7 | 0x01 << 6 | 0x01 << 5)

#define GAIN_1      	(0x00 << 0)
#define GAIN_2      	(0x01 << 0)
#define GAIN_4      	(0x01 << 1)
#define GAIN_8      	(0x01 << 1 | 0x01 << 0)
#define GAIN_16     	(0x01 << 2)
#define INTEGRATOR_ON   1
#define INTEGRATOR_OFF  0
#define FULLSCALESELECT_0_5V    0
#define FULLSCALESELECT_0_25V   1
#define FULLSCALESELECT_0_125V  2


class WaveformSample { // TODO-> comment.

	public:

		/** 
		 * @brief Class constructor
		*/
		WaveformSample(uint8_t rate, uint8_t cyclesToSample);

		/** 
		 * @brief Class destructor
		*/
		~WaveformSample();

		uint16_t getCurrentSample();

		uint8_t getDataAvailable();

		void setDataAvailable();

		uint8_t getCyclesToSample();

		void setCyclesToSample(uint8_t Cycles);

		uint8_t getCurrentCycle();

		void setCurrentCycle(uint8_t Cycle);

		uint32_t *getData();

		void putData( uint32_t Data);

	private:

		uint8_t _dataAvailable = 0;
		uint8_t _cyclesToSample;
		uint8_t _currentCycle = 0;
		uint16_t _currentSample = 0;
		uint16_t _totalSamples;
		uint32_t *_dataPtr = NULL;



	friend class ADE7753;
};



class ADE7753 {

	// Public methods
	public:

		/** 
		 * @brief Class constructor
		*/
		ADE7753();

		/** 
		 * @brief Class destructor
		*/
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


		/**
		 * @brief
		 * 
		 * @param Bit on Mode register.
		 * 
		 * @return 
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t setMode(uint16_t mode);


		/**
		 * @brief
		 * 
		 * @param Bit on Mode register.
		 * 
		 * @return 
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t clearMode(uint16_t mode);

		/**
		 * @brief Get MODE register values from ADE7753
		 * 
		 * @return data from the register (unsigned 16 bits)
		 */
		uint16_t getMode(void);


		/**
		 * @brief
		 * 
		 * @param scale
		 * @param PGA2
		 * @param PGA1
		 * 
		 * @return
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t setGain(uint8_t scale, uint8_t PGA2, uint8_t PGA1);


		/** 
		 * @brief Get the values of PGA1, PGA2, and SCALE 
		 * from the GAIN register.
		 * 
		 * @return data from GAIN register
		*/
		uint8_t getGain(void);


		/**
		 * @brief Activate the digital integrator on Channel 1
		 * 
		 * @param integrator 1 for enable and 0 for disable
		 * 
		 * @return
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_ARG   if parameter is invalid
		 */
		esp_err_t setIntegrator(uint8_t integrator);


		/** 
		 * @brief Get interrupt enable bit from CH1OS register
		 * 
		 * @return 1 (enabled) or 0 (disabled)
		*/
		uint8_t getIntegrator(void);


		/**
		 * @brief Reads the STATUS register
		 * 
		 * When an interrupt event occurs in the ADE7753, the 
		 * corresponding flag in the interrupt status register is set 
		 * to logic high
		 * 
		 * @return 16 bit unsigned data containing the STATUS value 
		 */
		uint16_t getStatus(void);


		/**
		 * @brief Reads and Resets the STATUS register 
		 * 
		 * @return 16 bit unsigned data containing the STATUS value 
		 */
		uint16_t getResetStatus(void);


		/**
		 * @brief Returns the sampled waveform data from either Channel 1, 
		 * Channel 2, or the active power signal. The data source and the 
		 * length of the waveform registers are selected by data Bits 14 
		 * and 13 in the mode register
		 * 
		 * @return 24 bits signed data
		 */
		int32_t getWaveform(void);


		/**
		 * @brief Channel 2 RMS Value (Voltage Channel). 
		 * The update rate of the Channel 2 rms measurement is CLKIN/4.
		 * 
		 * @return  24 bits unsigned data
		 */
		uint32_t getVRMS(void);


		/**
		 * @brief Channel 1 RMS Value (Current Channel). 
		 * The update rate of the Channel 1 rms measurement is CLKIN/4.
		 * 
		 * @return 24 bits unsigned data
		 */
		uint32_t getIRMS(void);


		/**
		 * @brief Period of the Channel 2 (Voltage Channel) Input Estimated 
		 * by Zero-Crossing Processing. The MSB of this register is always zero.
		 * 
		 * @return 16 bit unsigned data
		 */
		uint16_t getPeriod(void);


		/**
		 * @brief Returns the result of the last temperature measurement
		 * 
		 * @return 8 bit signed data
		 */
		int8_t getTemp(void);


		float getFrecuency(void);
		void setLineCyc(uint16_t d);
		
		
		void setZeroCrossingTimeout(uint16_t d);


		/**
		 * @brief Zero-Crossing Timeout. If no zero crossings are detected
		 * on Channel 2 within a time period specified by this 12-bit register,
		 * the interrupt request line (IRQ) is activated
		 * 
		 * @return int with the data (12 bits unsigned).
		 */
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
		uint8_t setPotLine(uint16_t cycle);
		uint32_t getWatt(void);
		uint32_t getVar(void);


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
		 * @brief
		 * 
		 * @param  
		 */
		void clearInterrupt(uint16_t reg);


		/**
		 * @brief  When an interrupt event occurs in the ADE7753, 
		 * the corresponding flag in the interrupt status register is set to logic high.
		 * 
		 * When the MCU services the interrupt, it must first carry out a read from the 
		 * interrupt status register to determine the source of the interrupt.
		 * 
		 * @return masked IRQ register
		 */
		uint16_t getMaskInterrupt(void);

		/**
		 * @brief Sets chip in waveform sampling mode
		 * 
		 * @param
		 * 	   - channel: 0=voltage 1=current
		 *     - numberOfCycles: ZX sampling number
		 * 
		 * @return
		 *     - ESP_OK Success
		 *     - ESP_ERR_INVALID_STATE Waveform already in sample state
		 */

		esp_err_t sampleWaveform(uint8_t channel, uint8_t numberOfCycles=5, uint8_t sampleRate = 0);

		uint8_t waveformSampleAvailable();
		void stopSampling();
		void ZXISR();

		uint8_t getVrmsStatus();
		uint8_t getIrmsStatus();

	// Private methods
	private:

		/**
		 * @brief Send/receive SPI data to the ADE7753
		 * 
		 * @param len transaction bit length
		 * 
		 * @param tx_buffer pointer to a transmit buffer, should have len bits
		 * 
		 * @param rx_buffer pointer to a receive buffer, should have len bits
		 * 
		 * @return      
		 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
		 *         - ESP_OK                on success
		 * 
		 */
		esp_err_t transaction(size_t len, void *tx_buffer, void *rx_buffer);

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
		/**
		 * 	Device activity flags
		 * 
		 * isVrms: active vrms measure.
		 * isIrms: active irms measure.
		 * _myWaveformPtr!=NULL~: active waveform sampling.
		 * 
		 * 	Device error status flags TODO-> Check is these are necessary.
		 * 
		 * isZXTimeout: ZX timeout has happened.
		 * isSag: Undervoltage.
		 * isVpeak: Voltage peak detected.
		 * isIpeak: Current peak detected.
		 *	
		 **/

		uint8_t _isVrms = 0;
		uint8_t _isIrms = 0;
		WaveformSample *_myWaveformPtr;

		// SPI default frequency
		int _spiFreq = DEF_SPI_FREQ;




};

#endif
