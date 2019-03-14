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


/*
read the 8 bit GPIO register after the four LS7366 chips have been read. In other words you are going to add to the LS7366 code you
studied in Section #1 of this take home assignment in order to read the MCP23S08’s GPIO register. Here you will
be using the SPI’s RX interrupt so you will not have to poll on the RXFFST status bits to determine when the SPI
transmit and receive have finished
*/
void SPI_RXint(void) {

	//pull all chip select high
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	GpioDataRegs.GPASET.bit.GPIO19 = 1;

	switch (SPIenc_state) {
		case 1:
			SPIbyte1 = SpiaRegs.SPIRXBUF;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 2;
			GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 2:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc1_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 3;
			GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 3:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc2_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 4;
			GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 4:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc3_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 5;
			GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 5:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc4_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;//MCP23S08 sends back 3 bytes of data, i.e., 3 interrupt
			SPIenc_state = 8;//read the 8 bit GPIO register after the four LS7366 chips have been read
			GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;//chip select MCP23S08, pull low to select
			SpiaRegs.SPITXBUF = ((unsigned)0x41)<<8;  // RD MODE A1 A0: 0 0 
			SpiaRegs.SPITXBUF = ((unsigned)0x09)<<8;  //Address of GPIO register: 09h
			SpiaRegs.SPITXBUF = ((unsigned)0x00)<<8;  //send random data

			break;
			
		case 8:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF; 
			MSP_reading = SPIbyte3;//output data of MCP23S08

			SWI_post(&SWI_control);

			break;	
			
		case 6:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIenc_state = 7;

			// Output to DAC Ch2
			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
			SPIenc_state = 7;
			GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
			SpiaRegs.SPITXBUF = ((unsigned)0x12)<<8;
			SpiaRegs.SPITXBUF = (int)(dac2data << 4);
			SpiaRegs.SPITXBUF = ((int)(dac2data))<<12;

			break;
		case 7:
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte1 = SpiaRegs.SPIRXBUF;


			SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
			SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

			SPIenc_state = 0;

			break;
		default:
			SPIbyte1 = SpiaRegs.SPIRXBUF;  // Should never get in here.  
			SPIbyte2 = SpiaRegs.SPIRXBUF; 
			SPIbyte3 = SpiaRegs.SPIRXBUF;
			SPIbyte4 = SpiaRegs.SPIRXBUF;
			SPIbyte5 = SpiaRegs.SPIRXBUF;
			SPIenc_state_errors++;
			break;
	}

	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE

}