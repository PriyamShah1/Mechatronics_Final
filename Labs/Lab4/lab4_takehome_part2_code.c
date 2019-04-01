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
/* Here we define several variables. The SPIbyteX are for reading the 5 byte values from the slave chips.
The SPIencX_reading are used to store the encoder values pulled from the slave chips. SPIenc_state is used
to determine the state for the state machine in the SPI_RXint ISR. MSP_Reading is used to store data received
from the MSP in case 5.*/
long SPIbyte1,SPIbyte2,SPIbyte3,SPIbyte4,SPIbyte5;
long SPIenc1_reading = 0;
long SPIenc2_reading = 0;
long SPIenc3_reading = 0;
long SPIenc4_reading = 0;
int SPIenc_state = 0;
int SPIenc_state_errors = 0;
long MSP_reading = 0;

unsigned int dac1data = 0;
unsigned int dac2data = 0;
/*
Function: initialize the MCP23S08 chip depending on what parameters are passed. Poll on the RXFFST status bits
to determine when the SPI transmit and receive have completed before returning from this function. 

The function initialize the registers to desired values.
*/
void Init_MCP23S08 (unsigned int IODIRvalue, unsigned int IPOLvalue, unsigned int 
GPPUvalue, unsigned int IOCONvalue, unsigned int OLATvalue){
	
	EALLOW;
		
		/* Setting the GPAMUX bits 15 and 14 of our 4 pins connected to the slave chips to 0, 
		thus setting the pins to General Purpose Input/Output mode. GPADIR sets the GPIO pins
		to act as output, so to transmit data to the master chip. GPASET controls the slave select,
		setting this value to 1 un-select it stops us from giving it commands. */
		GpioCtrlRegs.GPAMUX1.bit.GPIO19 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO48 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO49 = 0;
		GpioCtrlRegs.GPAMUX2.bit.GPIO58 = 0;

		GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO48 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO49 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO58 = 1;

		GpioDataRegs.GPASET.bit.GPIO19 = 1;
		GpioDataRegs.GPASET.bit.GPIO48 = 1;
		GpioDataRegs.GPASET.bit.GPIO49 = 1;
		GpioDataRegs.GPASET.bit.GPIO58 = 1;

	EDIS;

	InitSpiaGpio();
	
	/*Setting SPI config control register. SWReset sets the SPI ready to receive/transmit next char.
	Setting Pol and Phase so that data data is recorded on the rising edge, default low. 
	Then we control the length of chars sent over SPI to 8 bits. We then configure the SPI as a master,
	also enabling it for transmission. The SPI interrupt flag is also enabled. Finally we configure 
	the baud rate for the SPI connection. 
	*/
	SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; 
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

	SpiaRegs.SPICCR.bit.SPICHAR = 7;   // set to transmitt 8 bits

	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
	SpiaRegs.SPICTL.bit.TALK = 1;

	SpiaRegs.SPICTL.bit.SPIINTENA = 0;

	SpiaRegs.SPISTS.all=0x00E0;

	SpiaRegs.SPIBRR = 39;   // divide by 40 2.5 Mhz
	
	/* The SPIFFTX register controls the First In First Out Transmit Buffer. SPIRST set to 1 allows
	the SPI FIFO to resume normal transmit and receive operation. SPIFFENA set to 1 enables the FIFO,
	so if disabled the FIFO would not be used. TXFIFO resets the FIFO pointer to 0, clearing any past data. 
	The TXFFINTCLR set to 1 clears the interrupt flag for the Transmit buffer. The same is done bellow for
	the Receive buffer registers to prepare for communication. */
	SpiaRegs.SPIFFTX.bit.SPIRST = 1;
	SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
	SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
	SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;

	SpiaRegs.SPIFFCT.all=0x00;

	SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
	
	/* Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
	0x40 enable the write mode. Note the unsigned int is left shifted 8 bit so that we only send least
	significant 8 bits. After entering the writing mode, sending 0x00 selects the IODIR register. Next, we 
	transmit the IODIRvalue to the SPITXBUF to select desired I/O direction. We must place a while loop to
	stay here until all set of instructions are done, in this case 3. After this, we set the GPIO19 pins back
	to 1, un-selecting the pin since we are done giving it instructions. At the end we read the 3 bytes of data 
	from the FIFO Receive buffer to clear the buffer. */
	
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
	
	/* Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
	0x40 enable the write mode. Note the unsigned int is left shifted 8 bit so that we only send least
	significant 8 bits. After entering the writing mode, sending 0x01 selects the IPOL register. Next, we 
	transmit the IPOLvalue to the SPITXBUF to select desired input polarity. We must place a while loop to
	stay here until all set of instructions are done, in this case 3. After this, we set the GPIO19 pins back
	to 1, un-selecting the pin since we are done giving it instructions. At the end we read the 3 bytes of data 
	from the FIFO Receive buffer to clear the buffer. */
	
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
	
	/* Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
	0x40 enable the write mode. Note the unsigned int is left shifted 8 bit so that we only send least
	significant 8 bits. After entering the writing mode, sending 0x06 selects the GPPU register. Next, we 
	transmit the GPPUvalue to the SPITXBUF to select desired pull up resistor configuration. We must place a while loop to
	stay here until all set of instructions are done, in this case 3. After this, we set the GPIO19 pins back
	to 1, un-selecting the pin since we are done giving it instructions. At the end we read the 3 bytes of data 
	from the FIFO Receive buffer to clear the buffer. */
	
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
	
	/* Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
	0x40 enable the write mode. Note the unsigned int is left shifted 8 bit so that we only send least
	significant 8 bits. After entering the writing mode, sending 0x05 selects the IOCON register. Next, we 
	transmit the IOCONvalue to the SPITXBUF to select desired configuration for the device. We must place a while loop to
	stay here until all set of instructions are done, in this case 3. After this, we set the GPIO19 pins back
	to 1, un-selecting the pin since we are done giving it instructions. At the end we read the 3 bytes of data 
	from the FIFO Receive buffer to clear the buffer. */
	
	//write 1 byte data of IOCONvalue to ICON register
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
	
	/* Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
	0x40 enable the write mode. Note the unsigned int is left shifted 8 bit so that we only send least
	significant 8 bits. After entering the writing mode, sending 0x0A selects the OLAT register. Next, we 
	transmit the OLATvalue to the SPITXBUF to select desired output latches that modify the pins configured as 
	outputs. We must place a while loop to stay here until all set of instructions are done, in this case 3. After
	this, we set the GPIO19 pins back to 1, un-selecting the pin since we are done giving it instructions. At the
	end we read the 3 bytes of data from the FIFO Receive buffer to clear the buffer. */
	
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
Function: Only the least significant 8 bits are sent to the MCP23S08’s
OLAT register. Only the bits setup as outputs will be changed by writing to the OLAT register. Poll on
the RXFFST status bits to determine when the SPI transmit and receive have completed before returning
from this function.

The function set the port latch register to desired value.
Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
0x40 enable the write mode. After entering the writing mode, sending 0x0A selects the OLAT register. Next,
we transmit the byte to the SPITXBUF to select desired output latches that modify the pins configured as 
outputs. We must place a while loop to stay here until all set of instructions are done, in this case 3. After
this, we set the GPIO19 pins back to 1, un-selecting the pin since we are done giving it instructions. At the
end we read the 3 bytes of data from the FIFO Receive buffer to clear the buffer. 
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
Fucntion: Read and return the 8 bit value of the GPIO register. Poll on the
RXFFST status bits to determine when the SPI transmit and receive have completed before returning
from this function

The function read the port value by polling.
Clearing the GPIO19 enables us to write instructions to the IR register. Setting the SPITXBUF to 
0x41 enable the read mode. After entering the reading mode, sending 0x09 selects the GPIO register. Next,
we transmit the random data, i.e. 00, to the SPITXBUF since the chips requires 3 bytes of data every time.
We must place a while loop to stay here until all set of instructions are done, in this case 3. After
this, we set the GPIO19 pins back to 1, un-selecting the pin since we are done giving it instructions. At the
end we read the 3 bytes of data from the FIFO Receive buffer, and return the 8 bit value of the GPIO register 
stored in the last byte of received data.
*/
unsigned int ReadPort(void){
	//write 1 byte data of OLATvalue to OLAT register
	//pull chip select low for chip23S08
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	//send 3 bytes of data across
	SpiaRegs.SPITXBUF = ((unsigned)0x41)<<8;  // RD MODE A1 A0: 0 1 
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

	/* Setting all the GPIOX pins slave select 1, un-selecting them, so that we can call the 
	specific one needed in each state case in the state machine bellow, as we read one at a time. */
	GpioDataRegs.GPASET.bit.GPIO19 = 1;
	GpioDataRegs.GPASET.bit.GPIO48 = 1;
	GpioDataRegs.GPASET.bit.GPIO49 = 1;
	GpioDataRegs.GPASET.bit.GPIO58 = 1;

	switch (SPIenc_state) {
		case 1:
			/*First we read the first byte from the receive buffer, this is random data. Then we define
			for the interrupt flag of the receive buffer to be set after 5 bytes have been received. 
			We also progress the state of the machine to read the data from the next GPIOX pins. 
			Then we clear the GPIO48,49, and 58 register bits. When these are are pulled low, the DEMUX
			pulls the first output low also, allowing us to receive data from it. Then we send set 
			of instructions to the transmit buffer, select output register, then clear it. 
			We must send bytes in order to be able to receive at the same time, so we send all 0s. 
			*/
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 2;
			GpioDataRegs.GPASET.bit.GPIO48 = 1;
			GpioDataRegs.GPASET.bit.GPIO49 = 1;
			GpioDataRegs.GPASET.bit.GPIO58 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 2:
			/*First we read the first byte of random data from the receiver buffer, followed by the next 4 bytes.
			The transmit buffer value is masked before saving to our variable so that we only save the LSB.
			We must read the bytes individually since the SPI works as a FIFO register. We then concatenate all the 
			bytes (without the random data) into our own variable for encoder 1. We set the interrupt flag for the 
			receiver buffer to be activated after 5 bytes are received. Then progress the state to read the next encoder
			value in the next time around. We then clear the GPIO49 and 58 registers. When these are are pulled low, 
			the DEMUX pulls the second output low also, allowing us to receive data from it. Finally we send a set of
			instructions to clear the output register followed by 0s to fill in the remaining 4 bytes.
			*/
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc1_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 3;
			GpioDataRegs.GPASET.bit.GPIO49 = 1;
			GpioDataRegs.GPASET.bit.GPIO58 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 3:
			/*First we read the first byte of random data from the receiver buffer, followed by the next 4 bytes.
			The transmit buffer value is masked before saving to our variable so that we only save the LSB.
			We must read the bytes individually since the SPI works as a FIFO register. We then concatenate all the 
			bytes (without the random data) into our own variable for encoder 1. We set the interrupt flag for the 
			receiver buffer to be activated after 5 bytes are received. Then progress the state to read the next encoder
			value in the next time around. We then clear the GPIO48 and 58 registers. When these are are pulled low, 
			the DEMUX pulls the third output low also, allowing us to receive data from it. Finally we send a set of
			instructions to clear the output register followed by 0s to fill in the remaining 4 bytes.
			*/
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc2_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 4;
			GpioDataRegs.GPASET.bit.GPIO48 = 1;
			GpioDataRegs.GPASET.bit.GPIO58 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 4:
			/*First we read the first byte of random data from the receiver buffer, followed by the next 4 bytes.
			The transmit buffer value is masked before saving to our variable so that we only save the LSB.
			We must read the bytes individually since the SPI works as a FIFO register. We then concatenate all the 
			bytes (without the random data) into our own variable for encoder 1. We set the interrupt flag for the 
			receiver buffer to be activated after 5 bytes are received. Then progress the state to read the next encoder
			value in the next time around. We then clear the GPIO58 register. When these are are pulled low, 
			the DEMUX pulls the fourth output low also, allowing us to receive data from it. Finally we send a set of
			instructions to clear the output register followed by 0s to fill in the remaining 4 bytes.
			*/
			SPIbyte1 = SpiaRegs.SPIRXBUF;
			SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
			SPIenc3_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
			SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
			SPIenc_state = 5;
			GpioDataRegs.GPASET.bit.GPIO58 = 1;
			SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;
			SpiaRegs.SPITXBUF = 0;

			break;
		case 5:
			/*First we read the first byte of random data from the receiver buffer, followed by the next 4 bytes.
			The transmit buffer value is masked before saving to our variable so that we only save the LSB.
			We must read the bytes individually since the SPI works as a FIFO register. We then concatenate all the 
			bytes (without the random data) into our own variable for encoder 1. We set the interrupt flag for the 
			receiver buffer to be activated after 5 bytes are received. Then progress the state to read the next encoder
			value in the next time around. We then clear the GPIO19 register. When this is pulled low, 
			it allows us to receive data from the MCP. Finally we send a set of
			instructions to clear the output register followed by 0s to fill in the remaining 4 bytes.
			*/
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
			//reading in values from last device, then posting SWI to resume previous code
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
