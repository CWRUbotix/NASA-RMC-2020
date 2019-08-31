#include "ADS1120.h"


ADS1120::ADS1120(int cs){
    this->cs_pin = cs;
    this->ADC_SPI_settings = SPISettings(ADS1120_SPEED, MSBFIRST, SPI_MODE1);
    this->cur_mux_input = single_0;
}

void ADS1120::setup(uint8_t config){
	bool write_ok;
	uint8_t data[1] = {};
	// CONFIG REG 0
	if(config == CONFIG_FOR_POT){ 	// BYPASS THE PGA OR NOT
		bool read_ok = this->_read_reg(0, 1, data);
		data[0] |= 1; 				// sets bypass PGA to true
		write_ok = this->_write_reg(0, 1, data);
	}

	// CONFIG REG 1
	data[0] = (1 << 2); 			// SET CONTINUOUS CONVERSION MODE
	data[0] |= B01000000; 			// SET 180 SPS in TURBO MODE
	data[0] |= B00010000; 			// SET TURBO MODE
	write_ok = this->_write_reg(1, 1, data);

	// CONFIG REG 2
	data[0] = B10000000; 			//chooses AIN0 and AIN3 as vref

	if(config == CONFIG_FOR_LOAD_CELL){
		data[0] |= (1 << 3); 		// ENABLES LOW SIDE POWER SWITCH
	}
	write_ok = this->_write_reg(2, 1, data);

	// CONFIG REG 3
	data[0] = (1 << 1); 			// sets DRDYM to 1 (useful for single shot mode)
	write_ok = this->_write_reg(3, 1, data);

	digitalWrite(this->cs_pin,LOW);
    PAUSE_SHORT;
    SPI.beginTransaction(this->ADC_SPI_settings);
    SPI.transfer(com_start);
    SPI.endTransaction();
	digitalWrite(this->cs_pin,HIGH);

}

void ADS1120::_set_single_shot(){
	bool ok;
	uint8_t data[1] = {};
	ok = this->_read_reg(1, 1, data);
	data[0] &= ~(1 << 2);
	ok = this->_write_reg(1, 1, data);
}

void ADS1120::_set_continuous_conv(){
	bool ok;
	uint8_t data[1] = {};
	ok = this->_read_reg(1, 1, data);
	data[0] |= (1 << 2);
	ok = this->_write_reg(1, 1, data);
}


//set the gain to 2^n
//keep n at 7 or lower
void ADS1120::set_gain(uint8_t n){
    uint8_t config_reg = 0;
    _read_reg(0,1,&config_reg);
    config_reg &= 0b11110001;
    config_reg |= (n<<1);
    _write_reg(0,1,&config_reg);
    return; 
}

void ADS1120::set_mux_input(uint8_t mode){
    uint8_t config_reg = 0;
    _read_reg(0,1,&config_reg);
    config_reg &= 0b00001111;
    config_reg |= ( ((uint8_t)mode) <<4);   
    _write_reg(0,1,&config_reg);
    this->cur_mux_input = mode;
    return;
}

bool ADS1120::read_channel(uint8_t mode, uint16_t *output){

    //if the request matchtes the current mux mode then read out a value
    if(mode == this->cur_mux_input){
        digitalWrite(this->cs_pin, LOW);
        PAUSE_SHORT;
        SPI.beginTransaction(this->ADC_SPI_settings);
        uint8_t data_msb = SPI.transfer(com_rdata);
        data_msb 		 = SPI.transfer(0x00);
        uint8_t data_lsb = SPI.transfer(0x00);
        SPI.endTransaction();
        digitalWrite(this->cs_pin,HIGH);
        *output = data_msb;
        *output = *output << 8;
        *output += data_lsb;
        return true;
    //if the request does not match the current mux mode then change modes and read
    }else{
        uint8_t prev_mux_input = this->cur_mux_input;
        set_mux_input(mode);
        _set_single_shot();
        
        uint32_t count = 0;
        bool got_data = false;
        
        digitalWrite(this->cs_pin,LOW);
        PAUSE_SHORT;
        SPI.beginTransaction(this->ADC_SPI_settings);
        SPI.transfer(com_start);

        while(count<500 && !got_data){
            count++;
            got_data = !digitalRead(miso_pin);
            delayMicroseconds(12);
        }

        if(!got_data) return false;
        uint8_t data_msb = SPI.transfer(0x00);
        // data_msb 		 = SPI.transfer(0x00);
        uint8_t data_lsb = SPI.transfer(0x00);
        SPI.endTransaction();
        digitalWrite(this->cs_pin,HIGH);
        *output = data_msb;
        *output = *output << 8;
        *output += data_lsb;
        this->_set_continuous_conv();
        set_mux_input(prev_mux_input);
        return true;
 
    }
           
}
