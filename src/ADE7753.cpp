/* ADE7753.cpp
====================================================================================================
v0.2
fng
By: Ezequiel Pedace & Ignacio Worn
Created:     7 Dic 2012
Last update: 02 Feb 2019

ADE7753 library for esp32 w/ esp-idf framework
*/

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "ADE7753.h"


ADE7753::ADE7753(void) {

}


ADE7753::~ADE7753(void) {

}


void ADE7753::configSPI(gpio_num_t DOUT = DEF_DOUT, gpio_num_t DIN = DEF_DIN, gpio_num_t SCLK = DEF_SCLK, gpio_num_t CS = DEF_SCLK, int spiFreq = DEF_SPI_FREQ) {
    // DOCS: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html

	// gpio_set_direction(_AFECS, GPIO_MODE_OUTPUT); // Configure SPI Chip Select PIN as Output
    // gpio_set_level(_AEFCS, 1); // Set SPI Chip Select PIN in HIGH to disable ADE7753 by default

	_DOUT = DOUT;
	_DIN = DIN;
	_SCLK = SCLK;
	_CS = CS;
    _spiFreq = spiFreq;

    // Error handler for esp callbacks
    esp_err_t ret;
    
    // Configure ChipSelect pin
    gpio_config_t gpio_CS = {
        .pin_bit_mask = (1ULL << _CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK( gpio_config(&gpio_CS) );
    disableChip();
    
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

    // Initialize the SPI bus using HSPI host without DMA.
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
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
        .spics_io_num = -1,

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
    
    // TODO: Add some check ESP_OK to return data?
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


esp_err_t ADE7753::write8(uint8_t reg, uint8_t data) {

    // Error handler
    esp_err_t ret;

    // Mask the register for a write operation
    reg |= (0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Enable ADE7753 communication mode
    ret = enableChip();

    // Send the write command
    ret = send(reg);

    // Send the data
    ret = send(data);

    // Disable ADE7753 communication mode 
    ret = disableChip();

    return ret;
}


esp_err_t ADE7753::write16(uint8_t reg, uint16_t data) {

    // Error handler
    esp_err_t ret;

    // Mask the register for a write operation
    reg |= (0x01 << 7);
    
    // Clear the reserved bit
    reg &= ~(0x01 << 6);

    // Split the data into 2 bytes
    uint8_t dataMSB = (uint8_t) data >> 8;
    uint8_t dataLSB = (uint8_t) data & 0x0F;
    
    // Enable ADE7753 communication mode
    ret = enableChip();

    // Send the write command
    ret = send(reg);

    // Send the MSB data
    ret = send(dataMSB);

    // Send the LSB data
    ret = send(dataLSB);

    // Disable ADE7753 communication mode 
    ret = disableChip();

    return ret;
}


/*****************************
 *
 *     public functions
 *
 *****************************/


uint8_t ADE7753::getVersion(void) {
    return read8(DIEREV);
}


esp_err_t ADE7753::setMode(uint16_t mode) {
    // Check arguments
    if ( (mode & WAVSEL_BIT) == WAVESEL_RESERVED ) {
        return ESP_ERR_INVALID_ARG;
    }
    // TODO-> check next lines make sense
    uint16_t tempMode = 0xFFFF & mode; // all ones but zeroes on mode zero bits.
    tempMode &= getMode(); // put zero on mode zero bits.
    tempMode |= mode; //now mode zero bits will be zero always and ones one.

    /**
     * DISHPF_BIT      (0x01 << 0)
     * DISLPF2_BIT     (0x01 << 1)
     * DISCF_BIT       (0x01 << 2)
     * DISSAG_BIT      (0x01 << 3)
     * ASUSPEND_BIT    (0x01 << 4)
     * TEMPSEL_BIT     (0x01 << 5)
     * SWRST_BIT       (0x01 << 6)
     * CYCMODE_BIT     (0x01 << 7)
     * DISCH1_BIT      (0x01 << 8)
     * DISCH2_BIT      (0x01 << 9)
     * SWAP_BIT	       (0x01 << 10)
     * DTRT_BIT    	   (0x01 << 12 | 0x01 << 11)
     * WAVSEL_BIT  	   (0x01 << 14 | 0x01 << 13)
     * POAM_BIT        (0x01 << 15)

     * DTRT_27_9	   (0x00 << 11)
     * DTRT_14		   (0x01 << 11)
     * DTRT_7		   (0x02 << 11)
     * DTRT_3_5		   (0x03 << 11)

     * WAVESEL_ACTIVE_POWER_SIGNAL  ((0x00 << 0 | 0x00 << 1) << 13)
     * WAVESEL_RESERVED 	        ((0x00 << 1 | 0x01 << 0) << 13)
     * WAVESEL_CH1	        		((0x01 << 1 | 0x00 << 0) << 13)
     * WAVESEL_CH2 	            	((0x01 << 1 | 0x01 << 0) << 13)
    **/

    // TODO:  Check if bit 6 is 1
    //      (SWRST) A data transfer should not take place to the ADE7753 for at least  18 μs after a software reset. 
    return write16(MODE, tempMode);
}

esp_err_t ADE7753::clearMode(uint16_t mode) {
    // Check arguments
    if ( (mode & WAVSEL_BIT) == WAVESEL_RESERVED ) {
        return ESP_ERR_INVALID_ARG;
    }
    // TODO-> check next lines make sense
    uint16_t tempMode = 0xFFFF & mode; // all ones but zeroes on mode zero bits.
    tempMode &= getMode(); // put zero on mode zero bits.
    tempMode |= mode; //now mode zero bits will be zero always and ones one.
    /**
     * DISHPF_BIT      (0x01 << 0)
     * DISLPF2_BIT     (0x01 << 1)
     * DISCF_BIT       (0x01 << 2)
     * DISSAG_BIT      (0x01 << 3)
     * ASUSPEND_BIT    (0x01 << 4)
     * TEMPSEL_BIT     (0x01 << 5)
     * SWRST_BIT       (0x01 << 6)
     * CYCMODE_BIT     (0x01 << 7)
     * DISCH1_BIT      (0x01 << 8)
     * DISCH2_BIT      (0x01 << 9)
     * SWAP_BIT	       (0x01 << 10)
     * DTRT_BIT    	   (0x01 << 12 | 0x01 << 11)
     * WAVSEL_BIT  	   (0x01 << 14 | 0x01 << 13)
     * POAM_BIT        (0x01 << 15)

     * DTRT_27_9	   (0x00 << 11)
     * DTRT_14		   (0x01 << 11)
     * DTRT_7		   (0x02 << 11)
     * DTRT_3_5		   (0x03 << 11)

     * WAVESEL_ACTIVE_POWER_SIGNAL  ((0x00 << 0 | 0x00 << 1) << 13)
     * WAVESEL_RESERVED 	        ((0x00 << 1 | 0x01 << 0) << 13)
     * WAVESEL_CH1	        		((0x01 << 1 | 0x00 << 0) << 13)
     * WAVESEL_CH2 	            	((0x01 << 1 | 0x01 << 0) << 13)
    **/

    // TODO:  Check if bit 6 is 1
    //      (SWRST) A data transfer should not take place to the ADE7753 for at least  18 μs after a software reset. 
    return write16(MODE,getMode() & !tempMode);
}


uint16_t ADE7753::getMode() {
    return read16(MODE);
}


esp_err_t ADE7753::setGain(uint8_t scale = 0, uint8_t PGA2 = 0, uint8_t PGA1 = 0) {
    // Check arguments
    if ( ( scale > (0x01 << 1)) | (PGA1 > (0x01 << 2)) | (PGA2 > (0x01 << 2)) ) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pgas = (PGA2 << 5) | (scale << 3) | (PGA1);
    
    // Write GAIN register, format is |3 bits PGA2 gain|2 bits full scale|3 bits PGA1 gain
    return write8(GAIN, pgas);
    
}
    

uint8_t ADE7753::getGain(void) {
    return read8(GAIN);
}


esp_err_t ADE7753::setIntegrator(uint8_t integrator = 1) {
    
    // Check arguments
    if ( integrator > 0x01 ) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write bit 7 of CH1OS register
    return write8(CH1OS, integrator << 7);
}


uint8_t ADE7753::getIntegrator(void) {
    return (read8(CH1OS) >> 7);
}


uint16_t ADE7753::getStatus(void) {
    return read16(STATUSR);
}


uint16_t ADE7753::getResetStatus(void) {
    return read16(RSTSTATUS);
}


int32_t ADE7753::getWaveform(void) {
    return (int32_t) read24(WAVEFORM);
        }


uint32_t ADE7753::getIRMS(void) {
	    return read24(IRMS);
	}


uint32_t ADE7753::getVRMS(void) {
		return read24(VRMS);
	}


uint16_t ADE7753::getPeriod(void) {
        return read16(PERIOD);
    }


int8_t ADE7753::getTemp(void) {
    return (int8_t) read8(TEMP);
}

/**
 * Line Cycle Energy Accumulation Mode Line-Cycle Register.
 * This 16-bit register is used during line cycle energy accumulation  mode
 * to set the number of half line cycles for energy accumulation
 * 
 * @return int with the data (16 bits unsigned).
 */
void ADE7753::setLineCyc(uint16_t d) {
    write16(LINECYC, d);
}


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
uint8_t ADE7753::setPotLine(uint16_t cycle) {
    // int64_t lastupdate = 0;
    // uint8_t t_of = 0;

    setMode(  getMode() | DISCF_BIT | DISSAG_BIT | CYCMODE_BIT );
    setInterrupt( getInterrupt() | CYCEND_BIT ); 
    setLineCyc(cycle);
    // getResetStatus(); 
    // lastupdate = esp_timer_get_time();
    // // wait to terminar de acumular
    // while( !(getStatus() & CYCEND_BIT) ) { // wait for the selected interrupt to occur
    //     if ((esp_timer_get_time() - lastupdate) > (cycle*20)) {
    //         t_of = 1;
    //         break;
    //     }
    // }
    // if(t_of) {
    //     return 0;
    // }
    // getResetStatus();
    // lastupdate = esp_timer_get_time();
    // // wait to terminar de acumular
    // while(!(getStatus() & CYCEND_BIT)) { // wait for the selected interrupt to occur
	// 	if ((esp_timer_get_time() - lastupdate) > (cycle*20)) {
	// 	    t_of = 1;
	// 	    break;
	// 	}
	// }
	// if(t_of) {
        
	//     return 0;
	// }
    // else{
	//     return 1;
	// }
    return 0;
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


void ADE7753::setInterrupt(uint16_t reg) {
    /* Posible reg value:
    AEHF_BIT	    (0x01 << 0)
    SAG_BIT		    (0x01 << 1)
    CYCEND_BIT	    (0x01 << 2)
    WSMP_BIT	    (0x01 << 3)
    ZX_BIT		    (0x01 << 4)
    TEMPC_BIT	    (0x01 << 5)
    RESET_BIT	    (0x01 << 6)
    AEOF_BIT	    (0x01 << 7)
    PKV_BIT		    (0x01 << 8)
    PKI_BIT		    (0x01 << 9)
    VAEHF_BIT	    (0x01 << 10)
    VAEOF_BIT	    (0x01 << 11)
    ZXTO_BIT	    (0x01 << 12)
    PPOS_BIT	    (0x01 << 13)
    PNEG_BIT	    (0x01 << 14)
    */

    // Write the Interrupt Enable Register
    write16(IRQEN,getInterrupt() | reg);
}

void ADE7753::clearInterrupt(uint16_t reg) {
    /* Posible reg value:
    AEHF_BIT	    (0x01 << 0)
    SAG_BIT		    (0x01 << 1)
    CYCEND_BIT	    (0x01 << 2)
    WSMP_BIT	    (0x01 << 3)
    ZX_BIT		    (0x01 << 4)
    TEMPC_BIT	    (0x01 << 5)
    RESET_BIT	    (0x01 << 6)
    AEOF_BIT	    (0x01 << 7)
    PKV_BIT		    (0x01 << 8)
    PKI_BIT		    (0x01 << 9)
    VAEHF_BIT	    (0x01 << 10)
    VAEOF_BIT	    (0x01 << 11)
    ZXTO_BIT	    (0x01 << 12)
    PPOS_BIT	    (0x01 << 13)
    PNEG_BIT	    (0x01 << 14)
    */

    // Write the Interrupt Enable Register
    write16(IRQEN,getInterrupt() & !reg);
}



uint16_t ADE7753::getInterrupt(void) {

    // Return the Interrupt Enable Register
    return read16(IRQEN);
}


uint16_t ADE7753::getMaskInterrupt( void ) {
    
    // Mask the IRQ status with the IRQ enabled bits
    return ( getResetStatus() & getInterrupt() );
}

uint8_t ADE7753::getVrmsStatus(){
    return _isVrms;
}

uint8_t ADE7753::getIrmsStatus(){
    return _isIrms;
}

uint8_t ADE7753::getIrmsWaveformStatus(){
    return _iWaveform.isActive;
}

uint8_t ADE7753::getVrmsWaveformStatus(){
    return _vWaveform.isActive;
}

esp_err_t ADE7753::sampleWaveform(uint8_t channel, uint8_t numberOfCycles, uint8_t sampleRate){
    if (numberOfCycles > MAXSAMPLECYCLES){
        numberOfCycles = MAXSAMPLECYCLES;
    }
    if (numberOfCycles < 1){
        numberOfCycles = 1;
    }
    if (_iWaveform.isActive | _vWaveform.isActive){
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t tempMode = getMode() & 0x87FF; // first we mask out previous waveselect value.
    if(channel){ // cuurent sampling mode ch1
        _iWaveform.isActive = 1;
        _iWaveform.currentCycle = 0;
        _iWaveform.currentSample = 0;
        tempMode|=WAVESEL_CH1;
    }else{ // voltage sampling mode ch2
        _vWaveform.isActive = 1;
        _vWaveform.currentCycle = 0;
        _vWaveform.currentSample = 0;
        tempMode|=WAVESEL_CH2;
    }
    switch(sampleRate){
        case 0:         tempMode|=DTRT_3_5;
                        break;
        case 1:         tempMode|=DTRT_7;
                        break;
        case 2:         tempMode|=DTRT_14;
                        break;
        case 3:         tempMode|=DTRT_27_9;
                        break;
        default:        tempMode|=DTRT_27_9;
                        break;
    }
    setMode(tempMode);
    setInterrupt(WSMP_BIT);
}

void ADE7753::waveformSampleAvailable(){
    if (_iWaveform.isActive){
        _iWaveform.data[_iWaveform.currentSample] = getWaveform();
        _iWaveform.currentSample++;
    }else{
        _vWaveform.data[_vWaveform.currentSample] = getWaveform();
        _vWaveform.currentSample++;
    }
}
