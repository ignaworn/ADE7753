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


//bits

/*MODE REGISTER (0x09)
The ADE7753 functionality is configured by writing to the mode register. Table 14 describes the functionality of each bit in the register.

Bit Location	Bit Mnemonic	Default Value 		Description
0				DISHPF			0					HPF (high-pass filter) in Channel 1 is disabled when this bit is set.
1				DISLPF2			0					LPF (low-pass filter) after the multiplier (LPF2) is disabled when this bit is set.
2				DISCF			1					Frequency output CF is disabled when this bit is set.
3				DISSAG			1					Line voltage sag detection is disabled when this bit is set.
4				ASUSPEND		0					By setting this bit to Logic 1, both ADE7753 A/D converters can be turned off.
													In normal operation, this bit should be left at Logic 0.
													All digital functionality can be stopped by suspending the clock signal at CLKIN pin.
5				TEMPSEL			0					Temperature conversion starts when this bit is set to 1.
													This bit is automatically reset to 0 when the temperature conversion is finished.
6				SWRST			0					Software Chip Reset. A data transfer should not take place to the
													ADE7753 for at least 18 �s after a software reset.
7				CYCMODE			0					Setting this bit to Logic 1 places the chip into line cycle energy accumulation mode.
8				DISCH1			0					ADC 1 (Channel 1) inputs are internally shorted together.
9				DISCH2			0					ADC 2 (Channel 2) inputs are internally shorted together.
10				SWAP			0					By setting this bit to Logic 1 the analog inputs V2P and V2N are connected to ADC 1
													and the analog inputs V1P and V1N are connected to ADC 2.
11 to 12		DTRT			0					These bits are used to select the waveform register update rate.
													DTRT1	DTRT0	Update Rate
													0		0		27.9 kSPS (CLKIN/128)
													0		1		14 kSPS (CLKIN/256)
													1		0		7 kSPS (CLKIN/512)
													1		1		3.5 kSPS (CLKIN/1024)
13 to 14		WAVSEL			0					These bits are used to select the source of the sampled data for the waveform register.
													WAVSEL[1:0]		Source
													0	0			24 bits active power signal (output of LPF2)
													0	1			Reserved
													1	0			24 bits Channel 1
													1	1			24 bits Channel 2
15				POAM			0					Writing Logic 1 to this bit allows only positive active power to be accumulated in the ADE7753.
*/

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

/*
IRQEN:      INTERRUPT ENABLE REGISTER (0x0A)
STATUSR:    INTERRUPT STATUS REGISTER (0x0B)
RSTSTATUS:  RESET INTERRUPT STATUS REGISTER (0x0C) 

The status register is used by the MCU to determine the source of an interrupt request (IRQ).
When an interrupt event occurs in the ADE7753, the corresponding flag in the interrupt status register is set to logic high.
If the enable bit for this flag is Logic 1 in the interrupt enable register, the IRQ logic output goes active low.
When the MCU services the interrupt, it must first carry out a read from the interrupt status register to determine the source of the interrupt.
*/

#define AEHF	    0x01 << 0       // Indicates that an interrupt occurred because the active energy register, AENERGY, is more than half full.
#define SAG		    0x01 << 1       // Indicates that an interrupt was caused by a SAG on the line voltage.
#define CYCEND	    0x01 << 2       // Indicates the end of energy accumulation over an integer number of half line cycles as defined by the content of the LINECYC register�see the Line Cycle Energy Accumulation Mode section.
#define WSMP	    0x01 << 3       // Indicates that new data is present in the waveform register.
#define ZX		    0x01 << 4       // This status bit is set to Logic 0 on the rising and falling edge of the the voltage waveform. See the Zero-Crossing Detection section.
#define TEMPC	    0x01 << 5       // Indicates that a temperature conversion result is available in the temperature register.
#define RESET	    0x01 << 6       // Indicates the end of a reset (for both software or hardware reset). The corresponding enable bit has no function in the interrupt enable register, i.e., this status bit is set at the end of a reset, but it cannot be enabled to cause an interrupt.
#define AEOF	    0x01 << 7       // Indicates that the active energy register has overflowed.
#define PKV		    0x01 << 8       // Indicates that waveform sample from Channel 2 has exceeded the VPKLVL value.
#define PKI		    0x01 << 9       // Indicates that waveform sample from Channel 1 has exceeded the IPKLVL value.
#define VAEHF	    0x01 << 10      // Indicates that an interrupt occurred because the active energy register, VAENERGY, is more than half full.
#define VAEOF	    0x01 << 11      // Indicates that the apparent energy register has overflowed.
#define ZXTO	    0x01 << 12      // Indicates that an interrupt was caused by a missing zero crossing on the line voltage for the specified number of line cycles�see the Zero-Crossing Timeout section.
#define PPOS	    0x01 << 13      // Indicates that the power has gone from negative to positive.
#define PNEG	    0x01 << 14      // Indicates that the power has gone from positive to negative.

/*
CH1OS:      CH1OS REGISTER (0x0D)
CH2OS:      CH1OS REGISTER (0x0E)
The CHxOS register is an 8-bit, read/write enabled register.
The MSB of this register is used to switch on/off the digital integrator in Channel x,
and Bits 0 to 5 indicates the amount of the offset correction in Channel x.

Bit Location	Bit Mnemonic		Description
5 to 0			OFFSET				The six LSBs of the CH1OS register control the amount of dc offset correction in Channel x ADC.
									The 6-bit offset correction is sign and magnitude coded.
									Bits 0 to 4 indicate the magnitude of the offset correction.
									Bit 5 shows the sign of the offset correction.
									A 0 in Bit 5 means the offset correction is positive and a 1 indicates the offset correction is negative.
6				Not Used			This bit is unused.
7				INTEGRATOR			This bit is used to activate the digital integrator on Channel x.
									The digital integrator is switched on by setting this bit.
									This bit is set to be 0 on default.
*/

#define INTEGRATOR  0x01 << 7

/*
GAIN:       GAIN REGISTER (0x0F)
The PGA configuration of the ADE7753 is defined by writing to the GAIN register.

Bit Location		Bit Mnemonic		Default Value		Description
0 to 2				PGA1				0					Current GAIN
															PGA1[2:0]			Description
															0	0	0			x1
															0	0	1			x2
															0	1	0			x4
															0	1	1			x8
															1	0	0			x16
3 to 4				SCALE				0					Current input full-scale select
															SCALE[1:0]			Description
															0	0				0.5v
															0	1				0.25v
															1	0				0.125v
															1	1				Reserved
5 to 7				PGA2				0					Voltage GAIN
															PGA2[2:0]			Description
															0	0	0			x1
															0	0	1			x2
															0	1	0			x4
															0	1	1			x8
															1	0	0			x16
*/


// Constants

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
		* @brief SPI Pin configuration
		* 
		* @param DOUT, Master In Serial Out (MISO) Pin
		* @param DIN, Master Out Serial In (MOSI) Pin
		* @param SCLK, Serial Clock (CLK) Pin
		* @param CS, Chip Select (CS) Pin
		* @param spiFreq, SPI Data transfer frequency
		* 
		*/
        void configSPI(gpio_num_t DOUT, gpio_num_t DIN, gpio_num_t SCLK, gpio_num_t CS, int spiFreq);
        void setSPI(void);
        void closeSPI(void);

    //----------------------------------------------------------------------------
    // Modos y configs
    //----------------------------------------------------------------------------
        
		/**
		 * @brief Die Revision Register.
		 *
		 * @return: revision number of the silicon.
		 * 
		 */
		uint8_t getVersion();

        void setMode(uint16_t m);
        uint16_t getMode();
        void gainSetup(uint8_t integrator, uint8_t scale, uint8_t PGA2, uint8_t PGA1);
        uint16_t getInterrupts(void);
        void setInterrupts(uint16_t i);
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
        uint32_t getVa(void);
        void setIntPin(uint8_t interruptPin);
        void setVconst(float vconst);
        void setIconst(float iconst);
        void setReadingsNum(uint8_t readingsNum);
        void setInterruptFunction( void *(function));
        void doInterrupts(void);


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
		 */
		void write8(uint8_t reg, uint8_t data);
		
		/**
		 * @brief Write 16 bits to the device at specified register
		 * 
		 * @param reg register direction
		 * 
		 * @param data 16 bits of data to send
		 *
		 */
        void write16(uint8_t reg, uint16_t data);

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
        gpio_num_t _DOUT = GPIO_NUM_12;
        gpio_num_t _DIN = GPIO_NUM_13;
        gpio_num_t _SCLK = GPIO_NUM_14;
        gpio_num_t _CS = GPIO_NUM_15;
		
		// SPI default frequency (4 MHz is the max)
        int _spiFreq = 4*1000*1000;

        uint8_t _readingsNum = 2;
        float _vconst = 1;
        float _iconst = 1;
        uint8_t _interruptPin = 0;

};

#endif