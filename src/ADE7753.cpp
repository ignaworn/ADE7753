/* ADE7753.cpp
====================================================================================================
v0.2
fng
By: Ezequiel Pedace & Ignacio Worn
Created:     7 Dic 2012
Last update: 02 Feb 2019

ADE7753 library for esp32 w/ esp-idf framework
*/

#include "stdio.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "ADE7753.h"

// TODO: Cleanup includes
#include <string.h>


/**
 * @brief Class constructor 
 */
ADE7753::ADE7753(void) {

}


/** 
 * @brief Class destructor
 */
ADE7753::~ADE7753(void) {

}



void ADE7753::configSPI(gpio_num_t DOUT, gpio_num_t DIN, gpio_num_t SCLK, gpio_num_t CS, int spiFreq) {
	// SPI Pin configuration
	_DOUT = DOUT;
	_DIN = DIN;
	_SCLK = SCLK;
	_CS = CS;
    _spiFreq = spiFreq;
}


/**
 * @brief Start SPI module and configure ADE7753 device
 */
void ADE7753::setSPI(void) {
    // DOCS: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html
    
	// gpio_set_direction(_AFECS, GPIO_MODE_OUTPUT); // Configure SPI Chip Select PIN as Output
    // gpio_set_level(_AEFCS, 1); // Set SPI Chip Select PIN in HIGH to disable ADE7753 by default

    // Error handler for esp callbacks
    esp_err_t ret;
    
    // SPI BUS configuration structure
    spi_bus_config_t buscfg = {
		// GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used. 
        .mosi_io_num = _DIN,

		// GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used. 
        .miso_io_num = _DOUT,

		// GPIO pin for Spi CLocK signal, or -1 if not used. 
        .sclk_io_num = _SCLK,

		// GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used. 
        .quadwp_io_num = -1,
        
		// GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used. 
		.quadhd_io_num = -1,

		// Maximum transfer size, in bytes. Defaults to 4094 if 0. 
        .max_transfer_sz = 0, // TODO: Calculate proper value.

        // Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
        .flags = (uint32_t) 0,

        // Interrupt flag for the bus to set the priority, and IRAM attribute
        .intr_flags = 0,
    };

    // Initialize the SPI bus using HSPI host and use DMA channel 1
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	/**
	 * ESP_ERR_INVALID_ARG if configuration is invalid
	 * ESP_ERR_INVALID_STATE if host already is in use
	 * ESP_ERR_NO_MEM if out of memory
	 * ESP_OK on success 
	 */
    ESP_ERROR_CHECK(ret);

    // SPI Device Interface configuration structure.
    // See `spi_device_interface_config_t` in
    // https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/spi_master.h
    spi_device_interface_config_t ade7753_devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,

        // SPI mode (0-3)
        // Clock polarity and phase. See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
        .mode = 2,

        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0, // TODO: Fix this value

		// Clock speed, divisors of 80MHz, in Hz. See ``SPI_MASTER_FREQ_*``.
        .clock_speed_hz = _spiFreq,
        .input_delay_ns = 0,
        
        // SPI Chip Select pin
        .spics_io_num = _CS,

        .flags = (uint32_t) 0,

        // Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
        .queue_size = 7, // TODO: Check this value
        
        // Callback to be called before and after a transmission is made.
        .pre_cb = 0,  
        .post_cb = 0,

    };

    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &ade7753_devcfg, &_SPI);
	/**
	 * ESP_ERR_INVALID_ARG if parameter is invalid
	 * ESP_ERR_NOT_FOUND if host doesn’t have any free CS slots
	 * ESP_ERR_NO_MEM if out of memory
	 * ESP_OK on success 
	*/
    ESP_ERROR_CHECK(ret);

// TODO: Separate methods for bus initialization and device attachment
}


void ADE7753::closeSPI(void) {
    // Error handler for esp callbacks
    esp_err_t ret;
    
    // Remove the ADE7753 from the SPI bus
    ret = spi_bus_remove_device(_SPI);
    /**
     *         - ESP_ERR_INVALID_ARG   if parameter is invalid
     *         - ESP_ERR_INVALID_STATE if device already is freed
     *         - ESP_OK                on success
     */
    ESP_ERROR_CHECK(ret);

	// SPI.setDataMode(SPI_MODE0); // TODO: No_arduino
	ets_delay_us(10); // TODO: Check this delay. Is it necessary?

// TODO: Separate methods to dettach spi device and remove SPI bus.
}


/*****************************
 *
 * Private Functions
 *
 *****************************/

esp_err_t ADE7753::enableChip() {
    // Set Chip Select on LOW to enable the ADE7753
    // TODO: Check if this is correct
    return gpio_set_level(_CS, 0);
}


esp_err_t ADE7753::disableChip() {
    // Set Chip Select on HIGH to disable the ADE7753
    // TODO: Check if this is correct
    return gpio_set_level(_CS, 1);
}


esp_err_t ADE7753::send(uint8_t data) {
    
    // Error handler for esp callbacks
    esp_err_t ret;

    // SPI Transaction structure. See `struct spi_transaction_t`
    // https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/spi_master.h
    spi_transaction_t t = {
        // Bitwise OR of SPI_TRANS_* flags
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = (uint16_t) 0,
        .addr = (uint64_t) 0,
        
        // Total data length, in bits
        .length = (size_t) 8,
        .rxlength = (size_t) 0,
        .user = (void*) 0, // User-defined variable. Can be used to store eg transaction ID

        // Pointer to transmit buffer, or NULL for no MOSI phase
        // If SPI_USE_TXDATA is set, data set here is sent directly from this variable.
        {.tx_data = {data, 0, 0, 0}},
        /**
         * Sometimes, the amount of data is very small making it less than optimal allocating 
         * a separate buffer for it. If the data to be transferred is 32 bits or less, it can 
         * be stored in the transaction struct itself. For transmitted data, use the tx_data 
         * member for this and set the SPI_USE_TXDATA flag on the transmission. For received 
         * data, use rx_data and set SPI_USE_RXDATA. In both cases, do not touch the tx_buffer 
         * or rx_buffer members, because they use the same memory locations as tx_data and rx_data.
        */

        // Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.
        // If SPI_USE_RXDATA is set, data is received directly to this variable
        // When rx_buffer is NULL (and SPI_USE_RXDATA) is not set) the read phase is skipped.
        {.rx_buffer = (void*) 0},
    };

   // Send the command to the ADE7753
    ret = spi_device_transmit(_SPI, &t);  
    /**
     *         - ESP_ERR_INVALID_ARG   if parameter is invalid
     *         - ESP_OK                on success
    */
    ESP_ERROR_CHECK(ret);

    return ret; // TODO: Move delay to device post cmd and replace `ret` with the `spi_device_transmit` return.

}


uint8_t ADE7753::receive() {
    // Allocate memory for return value
    uint8_t data;
    
    // Error handler for esp callbacks
    esp_err_t ret;

    // SPI Transaction structure. See `struct spi_transaction_t`
    // https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/spi_master.h
    spi_transaction_t t = {
        // Bitwise OR of SPI_TRANS_* flags
        .flags = (uint32_t) 0,
        .cmd = (uint16_t) 0,
        .addr = (uint64_t) 0,
        
        // Total data length, in bits
        /**
         * With the ADE7753 in communications mode (i.e., CS logic low), an 8-bit 
         * write to the communications register first takes place. The MSB of this 
         * byte transfer is a 0, indicating that the next data transfer operation 
         * is a read. The LSBs of this byte contain the address of the register 
         * that is to be read
        */
        .length = (size_t) 8,
        .rxlength = (size_t) 0,
        .user = (void*) 0, // User-defined variable. Can be used to store eg transaction ID
        
        // Pointer to transmit buffer, or NULL for no MOSI phase
        // If SPI_USE_TXDATA is set, data set here is sent directly from this variable.
        // When tx_buffer is NULL (and SPI_USE_TXDATA) is not set) the write phase is skipped.
        {.tx_buffer = (const void *) 0},

        // Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.
        {.rx_buffer = &data},
    };

    // Read the response
    ret = spi_device_transmit(_SPI, &t);  
    /**
     *         - ESP_ERR_INVALID_ARG   if parameter is invalid
     *         - ESP_OK                on success
    */
    ESP_ERROR_CHECK(ret);
    
    return data;
}


uint8_t ADE7753::read8(uint8_t reg) {

    // Mask the register for a read operation
    reg &= ~(0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Enable ADE7753 communication mode
    enableChip();

    // Send the read command
    send(reg);

    // Wait at least 4us after a write operation (write to comm register) to ensure propper operation.
    ets_delay_us(5);

    // Receive data
    uint8_t data = receive();

    // Disable ADE7753 communication mode 
    disableChip();

    // Return the received data
    return data; 
}


uint16_t ADE7753::read16(uint8_t reg) {
    
    // Mask the register for a read operation
    reg &= ~(0x01 << 7);

    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Enable ADE7753 communication mode
    enableChip();

    // Send the read command
    send(reg);
    
    // Wait at least 4us after a write operation (write to comm register) to ensure propper operation.
    ets_delay_us(5);

    // Receive MSB byte of data
    uint16_t data = (uint16_t) (receive() << 8);

    // Receive LSB byte of data
    data |= (uint16_t) receive();

    // Disable ADE7753 communication mode 
    disableChip();

    // Return the received data
    return data; 
}


uint32_t ADE7753::read24(uint8_t reg) {
    
    // Mask the register for a read operation
    reg &= ~(0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Enable ADE7753 communication mode
    enableChip();

    // Send the read command
    send(reg);
        
    // Wait at least 4us after a write operation (write to comm register) to ensure propper operation.
    ets_delay_us(5);

    // Receive MSB byte of data
    uint32_t data = (uint32_t) (receive() << 16);

    // Receive second byte of data
    data |= (uint32_t) (receive() << 8);

    // Receive third LSB byte of data
    data |= (uint32_t) receive();

    // Disable ADE7753 communication mode 
    disableChip();

    // Return the received data
    return data; 
}


void ADE7753::write8(uint8_t reg, uint8_t data) {

    // Mask the register for a write operation
    reg |= (0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Enable ADE7753 communication mode
    enableChip();

    // Send the write command
    send(reg);

    // Send the data
    send(data);

    // Disable ADE7753 communication mode 
    disableChip();

}


void ADE7753::write16(uint8_t reg, uint16_t data) {

    // Mask the register for a write operation
    reg |= (0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Split the data into 2 bytes
    uint8_t dataMSB = (uint8_t) data >> 8;
    uint8_t dataLSB = (uint8_t) data & 0x0F;
    
    // Enable ADE7753 communication mode
    enableChip();

    // Send the write command
    send(reg);

    // Send the MSB data
    send(dataMSB);

    // Send the LSB data
    send(dataLSB);

    // Disable ADE7753 communication mode 
    disableChip();
}


/*****************************
 *
 *     public functions
 *
 *****************************/


uint8_t ADE7753::getVersion(void) {
    return read8(DIEREV);
}


/**=== setMode / getMode ===
MODE REGISTER (0x09)
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
													ADE7753 for at least 18 µs after a software reset.
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
void ADE7753::setMode(uint16_t m) {
    write16(MODE, m);
}
uint16_t ADE7753::getMode() {
    return read16(MODE);
}

/** === gainSetup ===

GAIN REGISTER (0x0F)
The PGA configuration of the ADE7753 is defined by writing to the GAIN register.
Table 18 summarizes the functionality of each bit in the GAIN register.

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

void ADE7753::gainSetup(uint8_t integrator, uint8_t scale, uint8_t PGA2, uint8_t PGA1) {
uint8_t pgas = (PGA2<<5) | (scale<<3) | (PGA1);
write8(GAIN,pgas);//write GAIN register, format is |3 bits PGA2 gain|2 bits full scale|3 bits PGA1 gain
uint8_t ch1os = (integrator<<7);
write8(CH1OS,ch1os);
}

/**	getStatus()/resetStatus()/getInterrupts()/setInterrupts(int i)
INTERRUPT STATUS REGISTER (0x0B), RESET INTERRUPT STATUS REGISTER (0x0C), INTERRUPT ENABLE REGISTER (0x0A)
The status register is used by the MCU to determine the source of an interrupt request (IRQ).
When an interrupt event occurs in the ADE7753, the corresponding flag in the interrupt status register is set to logic high.
If the enable bit for this flag is Logic 1 in the interrupt enable register, the IRQ logic output goes active low.
When the MCU services the interrupt, it must first carry out a read from the interrupt status register to determine the source of the interrupt.


Bit Location	Interrupt Flag		Description
0				AEHF				Indicates that an interrupt occurred because the active energy register, AENERGY, is more than half full.
1				SAG					Indicates that an interrupt was caused by a SAG on the line voltage.
2				CYCEND				Indicates the end of energy accumulation over an integer number of half line cycles as defined by
									the content of the LINECYC register—see the Line Cycle Energy Accumulation Mode section.
3				WSMP				Indicates that new data is present in the waveform register.
4				ZX					This status bit is set to Logic 0 on the rising and falling edge of the the voltage waveform.
									See the Zero-Crossing Detection section.
5				TEMP				Indicates that a temperature conversion result is available in the temperature register.
6				RESET				Indicates the end of a reset (for both software or hardware reset).
									The corresponding enable bit has no function in the interrupt enable register, i.e.,
									this status bit is set at the end of a reset, but it cannot be enabled to cause an interrupt.
7				AEOF				Indicates that the active energy register has overflowed.
8				PKV					Indicates that waveform sample from Channel 2 has exceeded the VPKLVL value.
9				PKI					Indicates that waveform sample from Channel 1 has exceeded the IPKLVL value.
A				VAEHF				Indicates that an interrupt occurred because the active energy register, VAENERGY, is more than half full.
B				VAEOF				Indicates that the apparent energy register has overflowed.
C				ZXTO				Indicates that an interrupt was caused by a missing zero crossing on the line voltage for the
									specified number of line cycles—see the Zero-Crossing Timeout section.
D				PPOS				Indicates that the power has gone from negative to positive.
E				PNEG				Indicates that the power has gone from positive to negative.
F				RESERVED			Reserved.
*/



uint16_t ADE7753::getStatus(void) {
    return read16(STATUSR);
}


uint16_t ADE7753::resetStatus(void) {
    return read16(RSTSTATUS);
}

/** === getIRMS ===
* Channel 2 RMS Value (Current Channel).
* The update rate of the Channel 2 rms measurement is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with the data (24 bits unsigned).
*/
uint32_t ADE7753::getIRMS(void) {
	int64_t lastupdate = 0;
	uint8_t t_of = 0;
	resetStatus(); // Clear all interrupts
	lastupdate = esp_timer_get_time();
	while( !(getStatus() & ZX) ) {   // wait Zero-Crossing
        if ((esp_timer_get_time() - lastupdate) > 20) {
            t_of = 1;
            break;
        }
	}
	if (t_of) {
	    return 0;
	}
    else{
	    return read24(IRMS);
	}
}

/** === getVRMS ===
* Channel 2 RMS Value (Voltage Channel).
* The update rate of the Channel 2 rms measurement is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with the data (24 bits unsigned).
*/
uint32_t ADE7753::getVRMS(void) {

	int64_t lastupdate = 0;
	uint8_t t_of = 0;
	resetStatus(); // Clear all interrupts
	lastupdate = esp_timer_get_time();

	while( !(getStatus() & ZX) ) { // wait Zero-Crossing
        if( (esp_timer_get_time()-lastupdate) > 20 ) {
            t_of = 1;
            break;
        }
	}
	if(t_of) {
		return 0;
	}
    else {
		return read24(VRMS);
	}
}

/** === vrms ===
* Returns the mean of last 100 readings of RMS voltage. Also supress first reading to avoid
* corrupted data.
* rms measurement update rate is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with RMS voltage value
*/
float ADE7753::vrms() {
	uint8_t i = 0;
	uint32_t v = 0;
	if( getVRMS() ) { //Ignore first reading to avoid garbage
        for( i=0; i < _readingsNum+1; ++i ) {
            v+=getVRMS();
        }
	    return float(v/_readingsNum)/_vconst;
	}
    else{
	    return 0;
	}
}

/** === irms ===
* Returns the mean of last 100 readings of RMS current. Also supress first reading to avoid
* corrupted data.
* rms measurement update rate is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with RMS current value in hundreds of [mA], ie. 6709=67[mA]
*/
float ADE7753::irms() {
	uint8_t n=0;
	uint32_t i=0;
    
    //Ignore first reading to avoid garbage
	if( getIRMS() ) { 
        for(n=0;n<_readingsNum+1;++n) {
            i+=getIRMS();
        }
	    return float(i/_readingsNum)/_iconst;
	}
    else {
	    return 0;
	}
}

/**
 * Period of the Channel 2 (Voltage Channel) Input Estimated by Zero-Crossing Processing. The MSB of this register is always zero.
 * @param none
 * @return int with the data (16 bits unsigned).
 */
uint16_t ADE7753::getPeriod(void) {
    int64_t lastupdate = 0;
    uint8_t t_of = 0;
    resetStatus(); // Clear all interrupts
    lastupdate = esp_timer_get_time();
    while( !(getStatus()&ZX) ) {   // wait Zero-Crossing
        if ( (esp_timer_get_time() - lastupdate) > 20 ) {
            t_of = 1;
            break;
        }
    }
    if (t_of) {
        return 0;
    }
    else {
        return read16(PERIOD);
    }
}

/**
 * Line Cycle Energy Accumulation Mode Line-Cycle Register.
 * This 16-bit register is used during line cycle energy accumulation  mode
 * to set the number of half line cycles for energy accumulation
 * 
 * @return int with the data (16 bits unsigned).
 */
void ADE7753::setLineCyc(uint16_t d) {
    write16(LINECYC,d);
}

/**
 * Zero-Crossing Timeout. If no zero crossings are detected
 * on Channel 2 within a time period specified by this 12-bit register,
 * the interrupt request line (IRQ) is activated
 * 
 * @return int with the data (12 bits unsigned).
 */
void ADE7753::setZeroCrossingTimeout(uint16_t d) {
    write16(ZXTOUT,d);
}
uint16_t ADE7753::getZeroCrossingTimeout() {
    return read16(ZXTOUT);
}

/**
 * Sag Line Cycle Register. This 8-bit register specifies the number of
 * consecutive line cycles the signal on Channel 2 must be below SAGLVL
 * before the SAG output is activated.
 * @param none
 * @return char with the data (8 bits unsigned).
 */
uint8_t ADE7753::getSagCycles() {
    return read8(SAGCYC);
}
void ADE7753::setSagCycles(uint8_t d) {
    write8(SAGCYC,d);
}

/**
 * Sag Voltage Level. An 8-bit write to this register determines at what peak
 * signal level on Channel 2 the SAG pin becomes active. The signal must remain
 * low for the number of cycles specified in the SAGCYC register before the SAG pin is activated
 * @param none
 * @return char with the data (8 bits unsigned).
 */
uint8_t ADE7753::getSagVoltageLevel() {
    return read8(SAGLVL);
}
void ADE7753::setSagVoltageLevel(uint8_t d) {
    write8(SAGLVL,d);
}

/**
 * Channel 1 Peak Level Threshold (Current Channel). This register sets the levelof the current
 * peak detection. If the Channel 1 input exceeds this level, the PKI flag in the status register is set.
 * @param none
 * @return char with the data (8 bits unsigned).
 */
uint8_t ADE7753::getIPeakLevel() {
    return read8(IPKLVL);
}
void ADE7753::setIPeakLevel(uint8_t d) {
    write8(IPKLVL,d);
}

/**
 * Channel 2 Peak Level Threshold (Voltage Channel). This register sets the level of the
 * voltage peak detection. If the Channel 2 input exceeds this level,
 * the PKV flag in the status register is set.
 * @param none
 * @return char with the data (8bits unsigned).
 */
uint8_t ADE7753::getVPeakLevel() {
    return read8(VPKLVL);
}
void ADE7753::setVPeakLevel(uint8_t d) {
    write8(VPKLVL,d);
}

/**
 * Same as Channel 1 Peak Register except that the register contents are reset to 0 after read.
 * @param none
 * @return long with the data (24 bits 24 bits unsigned).
 */
uint32_t ADE7753::getIpeakReset(void) {
    return read24(RSTIPEAK);
}

/**
 * Same as Channel 2 Peak Register except that the register contents are reset to 0 after a read.
 * @param none
 * @return long with the data (24 bits  unsigned).
 */
uint32_t ADE7753::getVpeakReset(void) {
    return read24(RSTVPEAK);
}

/** === setPotLine(Phase) ===
Setea las condiciones para Line accumulation.
Luego espera la interrupccion y devuelve 1 cuando ocurre.
Si no ocurre por mas de 1,5 segundos devuelve un 0.
**/
uint8_t ADE7753::setPotLine(uint16_t Ciclos) {
    int64_t lastupdate = 0;
    uint8_t t_of = 0;
    uint16_t m = 0;
    m = m | DISCF | DISSAG | CYCMODE;
    setMode(m);
    resetStatus();
    setLineCyc(Ciclos);
    lastupdate = esp_timer_get_time();
    // wait to terminar de acumular
    while( !(getStatus() & CYCEND) ) { // wait for the selected interrupt to occur
        if ((esp_timer_get_time() - lastupdate) > (Ciclos*20)) {
            t_of = 1;
            break;
        }
    }
    if(t_of) {
        return 0;
    }
    resetStatus();
    lastupdate = esp_timer_get_time();
    // wait to terminar de acumular
    while(!(getStatus() & CYCEND)) { // wait for the selected interrupt to occur
		if ((esp_timer_get_time() - lastupdate) > (Ciclos*20)) {
		    t_of = 1;
		    break;
		}
	}
	if(t_of) {
	    return 0;
	}
    else{
	    return 1;
	}
}

/*
* getWatt()
* Returns active power required.
* Call this fuction befor setPotLine() to generate values.
* @param none
* @return uint32_t data (24 bits unsigned).
*/

uint32_t ADE7753::getWatt(void) {
    return read24(LAENERGY);
}


/*
* getVar()
* Returns reactive power required.
* Call this fuction before setPotLine() to generate values.
* @param none
* @return uint32_t data (24 bits unsigned).
*/

uint32_t ADE7753::getVar(void) {
    return read24(LVARENERGY);
}

/**
 * 
 * @brief Returns total power required.
 * 
 * @warning Call this fuction befor setPotLine() to generate values.
 * 
 * @return data (24 bits unsigned).
 */
uint32_t ADE7753::getVa(void) {
    return read24(LVAENERGY);
}

void ADE7753::setIntPin(uint8_t interruptPin) {
    _interruptPin = interruptPin;
}

void ADE7753::setVconst(float vconst) { // cant be 0
    if (vconst) {
        _vconst = vconst;
    }
}

void ADE7753::setIconst(float iconst) {
    if (iconst) {
        _iconst = iconst;
    }
}

void ADE7753::setReadingsNum(uint8_t readingsNum) {
    _readingsNum = readingsNum;
}

void ADE7753::setIntPin(gpio_num_t interruptPin) {
    _interruptPin = interruptPin;
}

esp_err_t ADE7753::setInterruptFunction( gpio_isr_t func ) {
    
    // Error variable
    esp_err_t ret;

    // Set pin as INPUT
    ret = gpio_set_direction(_interruptPin, GPIO_MODE_INPUT);
    /**
     * ESP_OK Success
     * ESP_ERR_INVALID_ARG GPIO error 
    */
    ESP_ERROR_CHECK(ret);
    
    // Disable pull-up and pull-down
    ret =  gpio_set_pull_mode(_interruptPin, GPIO_FLOATING);
    /**
     * ESP_OK Success
     * ESP_ERR_INVALID_ARG : Parameter error  
    */
    ESP_ERROR_CHECK(ret);

    // Set interrupt trigger type to falling edge
    ret = gpio_set_intr_type(_interruptPin, GPIO_INTR_NEGEDGE);
    /**
     * ESP_OK Success
     * ESP_ERR_INVALID_ARG Parameter error 
    */
    ESP_ERROR_CHECK(ret);
    
    // Enable interrupt 
    ret = gpio_intr_enable(_interruptPin);
    /**
     * ESP_OK Success
     * ESP_ERR_INVALID_ARG Parameter error 
    */
    ESP_ERROR_CHECK(ret);

    // Create ISR service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE); // TODO: Check `esp_intr_alloc.h` for correct value
    /**
     * ESP_OK Success
     * ESP_ERR_NO_MEM No memory to install this service
     * ESP_ERR_INVALID_STATE ISR service already installed.
     * ESP_ERR_NOT_FOUND No free interrupt found with the specified flags
     * ESP_ERR_INVALID_ARG GPIO error 
    */
    ESP_ERROR_CHECK(ret);

    // Attach ISR to the GPIO pin
    ret = gpio_isr_handler_add(_interruptPin, interruptFunction, (void*) func);
    /**
     * ESP_OK Success
     * ESP_ERR_INVALID_STATE Wrong state, the ISR service has not been initialized.
     * ESP_ERR_INVALID_ARG Parameter error 
     */
    ESP_ERROR_CHECK(ret);

    // Return ESP_OK
    return ret;
}

void ADE7753::setInterrupt(uint16_t reg) {
    // Save the IRQEN in memory to speed up interrupt source detection
    _irqen = reg;

    // Write the Interrupt Enable Register
    write16(IRQEN, reg);

    }

uint16_t ADE7753::getInterrupt(void) {
    // Update IRQ enable cache
    _irqen = read16(IRQEN);

    // Return the Interrupt Enable Register
    return _irqen;
}


void IRAM_ATTR ADE7753::interruptFunction( void* func ) {
// TODO: Is IRAM_ATTR required? (RAM)

    // Handle the IRQ 
    uint16_t status = read16(RSTSTATUS); 
    
    // Mask the irq status with the enabled irq's
    status &= _irqen;

    // Do lib stuff


    // Call external function
    //func( status );

}
