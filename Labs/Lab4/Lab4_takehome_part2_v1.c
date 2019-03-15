/*
Assume GPIO19 of TMS320F28335 is connected to chip select(CS) pin of MCP23S08 chip
Assume the address of MCP23S08 chip A1 A0 are both ground (00)

Connection: MOSI to (MCP23S08) SI/DIN
			Clock to Clock
			GPIO to CS(Chip Select)
GPIO16:SPISIMOA
GPIO17:SPISOMIA
GPIO18:SPICLKA
GPIO19, GPIO48, GPIO49, GPIO58:General Purpose I/O pin
chip select each chip when polling
	pull low to select, pull back high after done
*/
#include <tistdtypes.h>
#include <coecsl.h>
#include "28335_spi.h"
#include "user_includes.h"
long SPIbyte1,SPIbyte2,SPIbyte3,SPIbyte4,SPIbyte5;
long SPIenc1_reading = 0;
long SPIenc2_reading = 0;
long SPIenc3_reading = 0;
long SPIenc4_reading = 0;
int SPIenc_state = 0;
int	SPIenc_state_errors = 0;
long MSP_reading = 0;

unsigned int dac1data = 0;
unsigned int dac2data = 0;
/*
initialize the MCP23S08 chip depending on what parameters are passed. Poll on the RXFFST status bits
to determine when the SPI transmit and receive have completed before returning from this function. 

write 1 byte data of IODIRvalue to Direction register of MCP23S08 through SPI by writing into transmit
FIFO
*/
void Init_MCP23S08 (unsigned int IODIRvalue, unsigned int IPOLvalue, unsigned int 
GPPUvalue, unsigned int IOCONvalue, unsigned int OLATvalue){
	//write 1 byte data of IODIRvalue to Direction register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x00)<<8;  //Address of IODIR register: 00h
	SpiaRegs.SPITXBUF = ((unsigned)IODIRvalue)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
	
	//write 1 byte data of IPOLvalue to Polarity register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x01)<<8;  //Address of IPOL register: 01h
	SpiaRegs.SPITXBUF = ((unsigned)IPOLvalue)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
	
	//write 1 byte data of GPPUvalue to GPPU register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x06)<<8;  //Address of GPPU register:06h
	SpiaRegs.SPITXBUF = ((unsigned)GPPUvalue)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
	
	//write 1 byte data of IOCONvalue to Polarity register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x05)<<8;  //Address of IOCON register: 05h
	SpiaRegs.SPITXBUF = ((unsigned)IOCONvalue)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
	
	//write 1 byte data of OLATvalue to OLAT register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x0A)<<8;  //Address of OLAT register: 0Ah
	SpiaRegs.SPITXBUF = ((unsigned)OLATvalue)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
}


/*
Only the least significant 8 bits are sent to the MCP23S08’s
OLAT register. Only the bits setup as outputs will be changed by writing to the OLAT register. Poll on
the RXFFST status bits to determine when the SPI transmit and receive have completed before returning
from this function.

write only to OLAT register
*/
void SetPortLatch(unsigned int byte){
	//write 1 byte data of OLATvalue to OLAT register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x40)<<8;  // WR MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x0A)<<8;  //Address of OLAT register: 0Ah
	SpiaRegs.SPITXBUF = ((unsigned)byte)<<8;
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
}


/*
Read and return the 8 bit value of the GPIO register. Poll on the
RXFFST status bits to determine when the SPI transmit and receive have completed before returning
from this function
*/
unsigned int ReadPort(void){
	//write 1 byte data of OLATvalue to OLAT register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x41)<<8;  // RD MODE A1 A0: 0 0 
	SpiaRegs.SPITXBUF = ((unsigned)0x09)<<8;  //Address of GPIO register: 09h
	SpiaRegs.SPITXBUF = ((unsigned)0x00)<<8;  //send random data
	//wait for transmission/recption to happen
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {}
	//pull chip select back high, only 1 chip
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	//read data, 3 bytes
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;
	SPIbyte3 = SpiaRegs.SPIRXBUF;
	
	return SPIbyte3;
}

