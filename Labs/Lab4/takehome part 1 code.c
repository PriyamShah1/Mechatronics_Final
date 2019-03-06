
#include <tistdtypes.h>
#include <coecsl.h>
#include "28335_spi.h"
#include "user_includes.h"
/* Here we define several variables. The SPIbyteX are for reading the 5 byte values from the slave chips.
The SPIencX_reading are used to store the encoder values pulled from the slave chips. SPIenc_state is used
to determine the state for the state machine in the SPI_RXint ISR. */
long SPIbyte1,SPIbyte2,SPIbyte3,SPIbyte4,SPIbyte5;
long SPIenc1_reading = 0;
long SPIenc2_reading = 0;
long SPIenc3_reading = 0;
long SPIenc4_reading = 0;
int SPIenc_state = 0;
int	SPIenc_state_errors = 0;

unsigned int dac1data = 0;
unsigned int dac2data = 0;

/* In this function, we will be initiating the registers for the GPIOX pins, as well as the transfer
and receiver buffers, used to communicate with the slave chips. */
void init_SPI(void){
	/*****************************************************/
	/*********  SPI  *************************************/
	EALLOW;
		
		/* Setting the GPAMUX bits 15 and 14 of our 4 pins connected to the slave chips to 0, 
		thus setting the pins to General Purpose Input/Output mode. GPADIR sets the GPIO pins
		to act as output, so to transmit data to the master chip. GPASET controls the slave select,
		setting this value to 1 un-select it stops us from giving it commands. */
		GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
		GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;

		GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;

		GpioDataRegs.GPASET.bit.GPIO9 = 1;
		GpioDataRegs.GPASET.bit.GPIO10 = 1;
		GpioDataRegs.GPASET.bit.GPIO11 = 1;
		GpioDataRegs.GPASET.bit.GPIO22 = 1;

	EDIS;

	InitSpiaGpio();

	EALLOW;
	// SS for DAC7564
		GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;  // use GPIO19 for SS
		GpioDataRegs.GPASET.bit.GPIO19 = 1;  // enabled
		GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  // GPIO19 as output
	EDIS;

	//NEED MORE COMMENTS
	SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
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

	/* Clearing the GPIOX enables us to write instructions to the IR register. Setting the SPITXBUF
	defines the bits in the Instruciton Register to select the CNTR register, then we clear it for all
	4 chips, and continue this until all set of instructions are done, in this case 1. After this, we 
	set the GPIOX pins back to 1, un-selecting them since we are done giving it instructions. At the end
	we write the first 8 bits in the FIFO Receive buffer to our variable SPIbyte1. */
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x20)<<8;  // CLR COUNT all four chips
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 1) {}
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;

	/* Clearing the GPIOX enables us to write instructions to the IR register. Setting the SPITXBUF
	defines the bits in the Instruciton Register to select the Mode Register 0, write into it for all
	4 chips. Then inside the MDR0, we set the filter clock division to 2, disable the index, select free
	running count mode, and declare x4 quadrature count mode. We must place a while loop to stay here
	until all set of instructions are done, in this case 2. After this, we set the GPIOX pins back to 1, 
	un-selecting them since we are done giving it instructions. At the end we write the first 16 bits 
	in the FIFO Receive buffer to our variables SPIbyte1 and SPIbyte2. */
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x88)<<8;  // WR to MDR0
	SpiaRegs.SPITXBUF = ((unsigned)0x83)<<8;
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;

	/* Clearing the GPIOX enables us to write instructions to the IR register. Setting the SPITXBUF
	defines the bits in the Instruciton Register to select the Mode Register 1, write into it for all
	4 chips. Then inside the MDR1, we do not use the flags so setting those bits to 0, then we enable
	counting in bit 2, set counter mode to 4-byte. We must place a while loop to stay here
	until all set of instructions are done, in this case 2. After this, we set the GPIOX pins back to 1, 
	un-selecting them since we are done giving it instructions. At the end we write the first 16 bits 
	in the FIFO Receive buffer to our variables SPIbyte1 and SPIbyte2. */
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0x90)<<8;  // WR MDR1
	SpiaRegs.SPITXBUF = 0x00<<8;
	while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;
	SPIbyte1 = SpiaRegs.SPIRXBUF;
	SPIbyte2 = SpiaRegs.SPIRXBUF;

	//NEED MORE COMMENTS
	SpiaRegs.SPICTL.bit.SPIINTENA = 1;
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

/*********  SPI  *************************************/
/*****************************************************/

	// SPI
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt

}

// interrupt function when data is received over the GPIOX pins from the encoder chips
void SPI_RXint(void) {

	/* Setting all the GPIOX pins slave select 1, un-selecting them, so that we can call the 
	specific one needed in each state case in the state machine bellow, as we read one at a time. */
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO22 = 1;

	GpioDataRegs.GPASET.bit.GPIO19 = 1;

	switch (SPIenc_state) {
		case 1:
			/*First we read the first byte from the receive buffer, this is random data. Then we define
			for the interrupt flag of the receive buffer to be set after 5 bytes have been received. 
			We also progress the state of the machine to read the data from the next GPIOX pins. 
			Then we clear the GPIO9 bit so that we can interact with that specific chip. 
			Then we 
			*/
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

void start_SPI(void) {

	/* RXFFIL sets the interrupt flag to be set after 1 byte is transmitted over the FIFO buffer.
	Clearing dir of GPIOX pins, enables them to transmit data from Slave to Master. When the first byte
	of data is transmitted, the interrupt service routine of SPI_RXint is called. */
	SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	SpiaRegs.SPITXBUF = ((unsigned)0xE8)<<8; // Latch All ENCs
	SPIenc_state = 1;
}

void writeDAC7564(float dac1,float dac2) {

	int rawdac1,rawdac2;

	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for DAC7564
	SpiaRegs.SPICTL.bit.CLK_PHASE = 0;

	rawdac1 = (int) (dac1 * 1638 + 0.5); // The 0.5 is used to round to the nearest integer
 	rawdac2 = (int) (dac2 * 1638 + 0.5); // The 0.5 is used to round to the nearest integer


   	if (rawdac1 < 0) rawdac1 = 0;
   	if (rawdac1 > 4095) rawdac1 = 4095;
   	if (rawdac2 < 0) rawdac2 = 0;
   	if (rawdac2 > 4095) rawdac2 = 4095;

	dac1data = rawdac1;
	dac2data = rawdac2;

	// Output to DAC Ch1
	SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
	SPIenc_state = 6;
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
	SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
	SpiaRegs.SPITXBUF = ((unsigned)0x10)<<8;
	SpiaRegs.SPITXBUF = (int)(dac1data << 4);
	SpiaRegs.SPITXBUF = ((int)(dac1data))<<12;

}