# ADE7753

Library for ESP-IDF framework, interfaces with ADE7753 single phase energy meter IC.

## Disclaimer

This is a work in progress, code needs further testing, cleanup and optimization. 

## Reads

*Voltage  
*Current  
*Frequency  
*Energy  
*Real power  
*Reactive power  
*Apparent power  
*IC temperature  

## Registers

### MODE REGISTER (0x09)

The ADE7753 functionality is configured by writing to the mode register. Table 14 describes the functionality of each bit in the register.

| Bit Location |  Bit Mnemonic |  Default Value | Description                                                                                                                                                                                                                                                                   |
|--------------|---------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0            |  DISHPF       | 0              | HPF (high-pass filter) in Channel 1 is disabled when this bit is set.                                                                                                                                                                                                         |
| 1            |  DISLPF2      | 0              | LPF (low-pass filter) after the multiplier (LPF2) is disabled when this bit is set.                                                                                                                                                                                           |
| 2            |  DISCF        | 1              | Frequency output CF is disabled when this bit is set.                                                                                                                                                                                                                         |
| 3            |  DISSAG       | 1              | Line voltage sag detection is disabled when this bit is set.                                                                                                                                                                                                                  |
| 4            |  ASUSPEND     | 0              | By setting this bit to Logic 1, both ADE7753 A/D converters can be turned off.  In normal operation, this bit should be left at Logic 0.  All digital functionality can be stopped by suspending the clock signal at CLKIN pin.                                               |
| 5            |  TEMPSEL      | 0              | Temperature conversion starts when this bit is set to 1.  This bit is automatically reset to 0 when the temperature conversion is finished.                                                                                                                                   |
| 6            |  SWRST        | 0              | Software Chip Reset. A data transfer should not take place to the ADE7753 for at least 18 us after a software reset.                                                                                                                                                          |
| 7            |  CYCMODE      | 0              | Setting this bit to Logic 1 places the chip into line cycle energy accumulation mode.                                                                                                                                                                                         |
| 8            |  DISCH1       | 0              | ADC 1 (Channel 1) inputs are internally shorted together.                                                                                                                                                                                                                     |
| 9            |  DISCH2       | 0              | ADC 2 (Channel 2) inputs are internally shorted together.                                                                                                                                                                                                                     |
| 10           |  SWAP         | 0              | By setting this bit to Logic 1 the analog inputs V2P and V2N are connected to ADC 1 and the analog inputs V1P and V1N are connected to ADC 2.                                                                                                                                 |
| 11 to 12     |  DTRT         | 0              | These bits are used to select the waveform register update rate.  DTRT1   DTRT0   Update Rate  0       0       27.9 kSPS (CLKIN/128)  0       1       14 kSPS (CLKIN/256)  1       0       7 kSPS (CLKIN/512)  1       1       3.5 kSPS (CLKIN/1024)                          |
| 13 to 14     |  WAVSEL       | 0              | These bits are used to select the source of the sampled data for the waveform register.  WAVSEL[1:0]     Source  0   0           24 bits active power signal (output of LPF2)  0   1           Reserved  1   0           24 bits Channel 1  1   1           24 bits Channel 2 |
| 15           |  POAM         | 0              | Writing Logic 1 to this bit allows only positive active power to be accumulated in the ADE7753.                                                                                                                                                                               |

For more detailed description, see page 55 from [ADE7753 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ade7753.pdf).

### INTERRUPT REGISTER (0x0A - 0x0C)

The status register is used by the MCU to determine the source of an interrupt request (IRQ).
When an interrupt event occurs in the ADE7753, the corresponding flag in the interrupt status register is set to logic high.
If the enable bit for this flag is Logic 1 in the interrupt enable register, the IRQ logic output goes active low.
When the MCU services the interrupt, it must first carry out a read from the interrupt status register to determine the source of the interrupt.

| Bit Location | Interrupt Flag | Description                                                                                                                                                                                                                                                  |
|--------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0            | AEHF           | Indicates that an interrupt occurred because the active energy register,   AENERGY, is more than half full.                                                                                                                                                  |
| 1            | SAG            | Indicates that an interrupt was caused by a SAG on the line voltage.                                                                                                                                                                                         |
| 2            | CYCEND         | Indicates the end of energy accumulation over an integer number of half   line cycles as defined by the content of the LINECYC register—see the Line   Cycle Energy Accumulation Mode section.                                                               |
| 3            | WSMP           | Indicates that new data is present in the waveform register.                                                                                                                                                                                                 |
| 4            | ZX             | This status bit is set to Logic 0 on the rising and falling edge of the   the voltage waveform. See the Zero-Crossing Detection section.                                                                                                                     |
| 5            | TEMP           | Indicates that a temperature conversion result is available in the   temperature register.                                                                                                                                                                   |
| 6            | RESET          | Indicates the end of a reset (for both software or hardware reset). The   corresponding enable bit has no function in the interrupt enable register,   i.e., this status bit is set at the end of a reset, but it cannot be enabled   to cause an interrupt. |
| 7            | AEOF           | Indicates that the active energy register has overflowed.                                                                                                                                                                                                    |
| 8            | PKV            | Indicates that waveform sample from Channel 2 has exceeded the VPKLVL   value.                                                                                                                                                                               |
| 9            | PKI            | Indicates that waveform sample from Channel 1 has exceeded the IPKLVL   value.                                                                                                                                                                               |
| A            | VAEHF          | Indicates that an interrupt occurred because the active energy register,   VAENERGY, is more than half full.                                                                                                                                                 |
| B            | VAEOF          | Indicates that the apparent energy register has overflowed.                                                                                                                                                                                                  |
| C            | ZXTO           | Indicates that an interrupt was caused by a missing zero crossing on the   line voltage for the specified number of line cycles—see the Zero-Crossing   Timeout section.                                                                                     |
| D            | PPOS           | Indicates that the power has gone from negative to positive.                                                                                                                                                                                                 |
| E            | PNEG           | Indicates that the power has gone from positive to negative.                                                                                                                                                                                                 |
| F            | RESERVED       | Reserved.                                                                                                                                                                                                                                                    |

For more detailed description, see page 57 from [ADE7753 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ade7753.pdf).

### CH1OS/CH2OS REGISTER (0x0D, 0x0E)

The CHxOS register is an 8-bit, read/write enabled register.
The MSB of this register is used to switch on/off the digital integrator in Channel x,
and Bits 0 to 5 indicates the amount of the offset correction in Channel x.

| Bit Location | Bit Mnemonic | Description |
|--------------|--------------|-------------|
| 5 to 0       | OFFSET       | The six LSBs of the CH1OS register control the amount of dc offset correction in Channel x ADC. The 6-bit offset correction is sign and magnitude coded. Bits 0 to 4 indicate the magnitude of the offset correction. Bit 5 shows the sign of the offset correction. A 0 in Bit 5 means the offset correction is positive and a 1 indicates the offset correction is negative. |
| 6            | Not Used     | This bit is unused. |
| 7            | INTEGRATOR   | This bit is used to activate the digital integrator on Channel x. The digital integrator is switched on by setting this bit. This bit is set to be 0 on default. |

For more detailed description, see page 56 from [ADE7753 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ade7753.pdf).

### GAIN REGISTER (0x0F)

The PGA configuration of the ADE7753 is defined by writing to the GAIN register.
Table 18 summarizes the functionality of each bit in the GAIN register.

| Bit Location | Bit Mnemonic | Default Value | Description                     |
|--------------|--------------|---------------|---------------------------------|
| 0 to 2       | PGA1         | 0             | Current GAIN                    |
| 3 to 4       | SCALE        | 0             | Current input full-scale select |
| 5 to 7       | PGA2         | 0             | Voltage GAIN                    |

For more detailed description, see figure 32, page 16 from [ADE7753 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ade7753.pdf).
