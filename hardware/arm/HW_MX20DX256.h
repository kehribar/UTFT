/**
 *
 * This file is a modified version of the HW_Teensy3.h created by Paul Stoffregen.
 *
 * Teensy 3.x pin definitions created by Dawnmist
 * http://forum.pjrc.com/threads/18002-Teensy-3-0-driving-an-SSD1289-with-utft?p=34719&viewfull=1#post34719
 *
 * This file only supports the B and D ports as defined by Dawnmist for 8-bit and 16-bit display modules.
 * Serial display modules are also supported.
 * 
 * NOTE: This file has only been tested on a Teensy 3.1
 *
**/

#define SPI_SR_TXCTR 0x0000f000

uint32_t ctar0;
uint32_t ctar1;

void updatectars() 
{
	uint32_t mcr = SPI0_MCR;
	if(mcr & SPI_MCR_MDIS) 
	{
		SPI0_CTAR0 = ctar0;
		SPI0_CTAR1 = ctar1;
	} 
	else 
	{
		SPI0_MCR = mcr | SPI_MCR_MDIS | SPI_MCR_HALT;
		SPI0_CTAR0 = ctar0;
		SPI0_CTAR1 = ctar1;
		SPI0_MCR = mcr;
	}
}

void init_raw()
{
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
	CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
	CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);

	// SPI_CTAR_DBR, SPI_CTAR_BR(0), SPI_CTAR_BR(1)
	ctar0 = SPI_CTAR_DBR;
	ctar1 = ctar0;
	ctar0 |= SPI_CTAR_FMSZ(7);
	ctar1 |= SPI_CTAR_FMSZ(15);
	SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F);
	SPI0_MCR |= SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF;
	updatectars();
}

void init(uint8_t speed) 
{
	init_raw();
	// Default 1/2 speed
	uint32_t ctar = SPI_CTAR_DBR;
	switch(speed) 
	{
		case 1: // 1/4
			ctar = 0;
			break;
	
		case 2: // 1/8
			ctar = SPI_CTAR_BR(1);
			break;

		case 3: // 1/12
			ctar = SPI_CTAR_BR(2);
			break;

		case 4: // 1/16
			ctar = SPI_CTAR_BR(3);
			break;

		case 5: // 1/32
			ctar = SPI_CTAR_PBR(1) | SPI_CTAR_BR(4);
			break;

		case 6: // 1/64
			ctar = SPI_CTAR_PBR(1) | SPI_CTAR_BR(5);
			break;

		case 7: //1/128
			ctar = SPI_CTAR_PBR(1) | SPI_CTAR_BR(6);
			// fall thru
		default:
		// default 1/2 speed, this is the maximum.
		break;
	}
	ctar0 = ctar | SPI_CTAR_FMSZ(7);
	ctar1 = ctar | SPI_CTAR_FMSZ(15);
	updatectars();
}

// *** Hardware specific functions ***
void UTFT::_hw_special_init()
{
	/* Maximum speed */
	init(0); 
}

void UTFT::LCD_Writ_Bus(char VH,char VL, byte mode)
{
	switch (mode)
	{
	case 1:
		if (display_serial_mode==SERIAL_4PIN)
		{
			if (VH==1)
				sbi(P_SDA, B_SDA);
			else
				cbi(P_SDA, B_SDA);
			pulse_low(P_SCL, B_SCL);
		}
		else
		{
			if (VH==1)
				sbi(P_RS, B_RS);
			else
				cbi(P_RS, B_RS);
		}

		/* Hardware SPI */
		SPI0_PUSHR = VL;						
		while((SPI0_SR & (SPI_SR_TXCTR)));		
	
		break;
	case 8:
		*(volatile uint8_t *)(&GPIOD_PDOR) = VH;
		pulse_low(P_WR, B_WR);
		*(volatile uint8_t *)(&GPIOD_PDOR) = VL;
		pulse_low(P_WR, B_WR);
		break;
	case 16:
		*(volatile uint8_t *)(&GPIOD_PDOR) = VH;
  		GPIOB_PCOR = 0x000F000F;							// clear data lines B0-3,B16-19
        GPIOB_PSOR = (0x0F & VL) | ((VL >> 4) << 16);  		// set data lines 0-3,16-19 if set in cl
		pulse_low(P_WR, B_WR);
		break;
	}
}

void UTFT::_set_direction_registers(byte mode)
{
	GPIOD_PDDR |= 0xFF;
	PORTD_PCR0  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR1  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR2  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR3  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR4  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR5  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR6  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR7  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);

	if (mode == 16)
    {
		GPIOB_PDDR |= 0x000F000F;
		PORTB_PCR0  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR1  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR2  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR3  = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR16 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR17 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR18 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		PORTB_PCR19 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    }
}
void UTFT::_fast_fill_16(int ch, int cl, long pix)
{
	long blocks;

	*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
  	GPIOB_PCOR = 0x000F000F;						// clear data lines B0-3,B16-19
    GPIOB_PSOR = (0x0F & cl) | ((cl >> 4) << 16);  	// set data lines 0-3,16-19 if set in cl

	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16); i++)
		{
			pulse_low(P_WR, B_WR);
		}
}

void UTFT::_fast_fill_8(int ch, long pix)
{
	long blocks;

	*(volatile uint8_t *)(&GPIOD_PDOR) = ch;

	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16); i++)
		{
			pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		}
}
