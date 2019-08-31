#ifndef _ADS1120_H_
#define _ADS1120_H_

#include <SPI.h>

#define ADS1120_SPEED       2500000

#ifndef PAUSE_SHORT
#define PAUSE_SHORT __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
#endif

enum FLAGS {
    CONFIG_FOR_LOAD_CELL,
    CONFIG_FOR_POT
};

enum commands{
    com_reset = 0b00000110,
    com_start = 0b00001000, 
    com_powerdown = 0b00000010,
    com_rdata = 0b00010000,
    com_rreg = 0b00100000, //apply OR mask to lower nibble
    com_wreg = 0b01000000, //apply OR mask to lower nibble
};

//nibble for mux
enum adc_mux_input{
    diff_0_1 = 0x0,
    diff_1_2 = 0x3,
    diff_2_3 = 0x5,
    single_0 = 0x8,
    single_1 = 0x9,
    single_2 = 0xA,
    single_3 = 0xB
};

class ADS1120{
  
  public:

    ADS1120(int cs_pin);
    void setup(uint8_t config);
    bool read_channel(uint8_t mode, uint16_t *output);
    void set_mux_input(uint8_t mode);
    void set_gain(uint8_t gain);

  private:
    SPISettings ADC_SPI_settings;
    static constexpr int miso_pin = 12; 
    uint8_t cur_mux_input;  
    int cs_pin;
    void _set_single_shot(void);
    void _set_continuous_conv(void);
 
    /*
 * Reads registers from the chip
 * @param start first register to read
 * @param regs number of registers to be read
 * @param *buf pointer to a uint8_t array of length regs 
 * @return true if read was successful
 * Note: only the lower two bits of start and regs may be set, other  bits must be cleared 
 */
    inline bool _read_reg(uint8_t start, uint8_t regs, uint8_t *buf){
        SPI.beginTransaction(ADC_SPI_settings);
        digitalWrite(cs_pin,LOW);
        PAUSE_SHORT;
        uint8_t command = com_rreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            buf[i] = SPI.transfer(0);
        }
        
        digitalWrite(cs_pin,HIGH);

        SPI.endTransaction();
         
        return true; 
    } 


    /*
 * Writes registers to the chip
 * @param start first register to write to 
 * @param regs number of registers to be written to 
 * @param *buf pointer to a uint8_t array of length regs 
 * @return true if write was successful
 * Note: only the lower two bits of start and regs are read
 */
    inline bool _write_reg(uint8_t start, uint8_t regs, uint8_t *buf){
        SPI.beginTransaction(ADC_SPI_settings);
        digitalWrite(cs_pin,LOW);
        PAUSE_SHORT;

        uint8_t command = com_wreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            SPI.transfer(buf[i]);
        }
        
        digitalWrite(cs_pin,HIGH);
        SPI.endTransaction();
        return true; 
    }
};

#endif