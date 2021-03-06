#include <stdint.h>
#include <stdbool.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];

/*
 *      Borrowed and adapted from https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/
 */
static const char character[0x60][6] = {
    { 0x00,0x00,0x00,0x00,0x00,0x00 } , /*SPC */
    { 0x00,0x00,0x5F,0x00,0x00,0x00 } , /* !  */
    { 0x04,0x03,0x04,0x03,0x00,0x00 } , /* "  */
    { 0x28,0x7E,0x14,0x3F,0x0A,0x00 } , /* #  */
    { 0x24,0x2A,0x7F,0x2A,0x12,0x00 } , /* $  */
    { 0x23,0x13,0x08,0x64,0x62,0x00 } , /* %  */
    { 0x30,0x4E,0x59,0x26,0x50,0x00 } , /* &  */
    { 0x00,0x00,0x02,0x01,0x00,0x00 } , /* '  */
    { 0x00,0x00,0x1C,0x22,0x41,0x00 } , /* (  */
    { 0x41,0x22,0x1C,0x00,0x00,0x00 } , /* )  */
    { 0x22,0x14,0x08,0x14,0x22,0x00 } , /* *  */
    { 0x08,0x08,0x3E,0x08,0x08,0x00 } , /* +  */
    { 0x50,0x30,0x00,0x00,0x00,0x00 } , /* ,  */
    { 0x08,0x08,0x08,0x08,0x08,0x00 } , /* -  */
    { 0x60,0x60,0x00,0x00,0x00,0x00 } , /* .  */
    { 0x20,0x10,0x08,0x04,0x02,0x00 } , /* /  */
    { 0x3E,0x51,0x49,0x45,0x3E,0x00 } , /* 0  */
    { 0x00,0x42,0x7F,0x40,0x00,0x00 } , /* 1  */
    { 0x62,0x51,0x49,0x49,0x46,0x00 } , /* 2  */
    { 0x22,0x41,0x49,0x49,0x36,0x00 } , /* 3  */
    { 0x18,0x14,0x12,0x7F,0x10,0x00 } , /* 4  */
    { 0x2F,0x45,0x45,0x45,0x39,0x00 } , /* 5  */
    { 0x3E,0x49,0x49,0x49,0x32,0x00 } , /* 6  */
    { 0x01,0x61,0x19,0x05,0x03,0x00 } , /* 7  */
    { 0x36,0x49,0x49,0x49,0x36,0x00 } , /* 8  */
    { 0x26,0x49,0x49,0x49,0x3E,0x00 } , /* 9  */
    { 0x00,0x36,0x36,0x00,0x00,0x00 } , /* :  */
    { 0x00,0x56,0x36,0x00,0x00,0x00 } , /* ;  */
    { 0x00,0x08,0x14,0x22,0x41,0x00 } , /* <  */
    { 0x14,0x14,0x14,0x14,0x14,0x00 } , /* =  */
    { 0x41,0x22,0x14,0x08,0x00,0x00 } , /* >  */
    { 0x02,0x01,0x59,0x09,0x06,0x00 } , /* ?  */
    { 0x3E,0x41,0x5D,0x55,0x2E,0x00 } , /* @  */
    { 0x60,0x1C,0x13,0x1C,0x60,0x00 } , /* A  */
    { 0x7F,0x49,0x49,0x49,0x36,0x00 } , /* B  */
    { 0x3E,0x41,0x41,0x41,0x22,0x00 } , /* C  */
    { 0x7F,0x41,0x41,0x22,0x1C,0x00 } , /* D  */
    { 0x7F,0x49,0x49,0x49,0x41,0x00 } , /* E  */
    { 0x7F,0x09,0x09,0x09,0x01,0x00 } , /* F  */
    { 0x1C,0x22,0x41,0x49,0x3A,0x00 } , /* G  */
    { 0x7F,0x08,0x08,0x08,0x7F,0x00 } , /* H  */
    { 0x00,0x41,0x7F,0x41,0x00,0x00 } , /* I  */
    { 0x20,0x40,0x40,0x40,0x3F,0x00 } , /* J  */
    { 0x7F,0x08,0x14,0x22,0x41,0x00 } , /* K  */
    { 0x7F,0x40,0x40,0x40,0x00,0x00 } , /* L  */
    { 0x7F,0x04,0x18,0x04,0x7F,0x00 } , /* M  */
    { 0x7F,0x04,0x08,0x10,0x7F,0x00 } , /* N  */
    { 0x3E,0x41,0x41,0x41,0x3E,0x00 } , /* O  */
    { 0x7F,0x09,0x09,0x09,0x06,0x00 } , /* P  */
    { 0x3E,0x41,0x51,0x21,0x5E,0x00 } , /* Q  */
    { 0x7F,0x09,0x19,0x29,0x46,0x00 } , /* R  */
    { 0x26,0x49,0x49,0x49,0x32,0x00 } , /* S  */
    { 0x01,0x01,0x7F,0x01,0x01,0x00 } , /* T  */
    { 0x3F,0x40,0x40,0x40,0x3F,0x00 } , /* U  */
    { 0x03,0x1C,0x60,0x1C,0x03,0x00 } , /* V  */
    { 0x0F,0x70,0x0F,0x70,0x0F,0x00 } , /* W  */
    { 0x41,0x36,0x08,0x36,0x41,0x00 } , /* X  */
    { 0x01,0x06,0x78,0x02,0x01,0x00 } , /* Y  */
    { 0x61,0x51,0x49,0x45,0x43,0x00 } , /* Z  */
    { 0x00,0x00,0x7F,0x41,0x41,0x00 } , /* [  */
    { 0x15,0x16,0x7C,0x16,0x11,0x00 } , /* \  */
    { 0x41,0x41,0x7F,0x00,0x00,0x00 } , /* ]  */
    { 0x00,0x02,0x01,0x02,0x00,0x00 } , /* ^  */
    { 0x40,0x40,0x40,0x40,0x40,0x00 } , /* _  */
    { 0x00,0x01,0x02,0x00,0x00,0x00 } , /* `  */
    { 0x00,0x20,0x54,0x54,0x78,0x00 } , /* a  */
    { 0x00,0x7F,0x44,0x44,0x38,0x00 } , /* b  */
    { 0x00,0x38,0x44,0x44,0x28,0x00 } , /* c  */
    { 0x00,0x38,0x44,0x44,0x7F,0x00 } , /* d  */
    { 0x00,0x38,0x54,0x54,0x18,0x00 } , /* e  */
    { 0x00,0x04,0x3E,0x05,0x01,0x00 } , /* f  */
    { 0x00,0x08,0x54,0x54,0x3C,0x00 } , /* g  */
    { 0x00,0x7F,0x04,0x04,0x78,0x00 } , /* h  */
    { 0x00,0x00,0x7D,0x00,0x00,0x00 } , /* i  */
    { 0x00,0x40,0x40,0x3D,0x00,0x00 } , /* j  */
    { 0x00,0x7F,0x10,0x28,0x44,0x00 } , /* k  */
    { 0x00,0x01,0x7F,0x00,0x00,0x00 } , /* l  */
    { 0x7C,0x04,0x7C,0x04,0x78,0x00 } , /* m  */
    { 0x00,0x7C,0x04,0x04,0x78,0x00 } , /* n  */
    { 0x00,0x38,0x44,0x44,0x38,0x00 } , /* o  */
    { 0x00,0x7C,0x14,0x14,0x08,0x00 } , /* p  */
    { 0x00,0x08,0x14,0x14,0x7C,0x00 } , /* q  */
    { 0x00,0x7C,0x08,0x04,0x04,0x00 } , /* r  */
    { 0x00,0x48,0x54,0x54,0x24,0x00 } , /* s  */
    { 0x00,0x04,0x3E,0x44,0x40,0x00 } , /* t  */
    { 0x00,0x3C,0x40,0x40,0x7C,0x00 } , /* u  */
    { 0x00,0x7C,0x20,0x10,0x0C,0x00 } , /* v  */
    { 0x1C,0x60,0x1C,0x60,0x1C,0x00 } , /* w  */
    { 0x00,0x6C,0x10,0x10,0x6C,0x00 } , /* x  */
    { 0x00,0x4C,0x50,0x30,0x1C,0x00 } , /* y  */
    { 0x00,0x44,0x64,0x54,0x4C,0x00 } , /* z  */
};

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOA, 2),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static int
writeCommandMulti(uint8_t *commandByte, uint8_t count)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);
	for(int i=0; i<count; i++) {
		payloadBytes[0] = commandByte[i];
		status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
						NULL		/* spi_master_user_config_t */,
						(const uint8_t * restrict)&payloadBytes[0],
						(uint8_t * restrict)&inBuffer[0],
						1		/* transfer size */,
						100		/* timeout in microseconds (unlike I2C which is ms) */);
	}
	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static int
writeData(uint16_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinDC);
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
	payloadBytes[0] = (commandByte >> 8);
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	payloadBytes[0] = (commandByte);
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);
	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static void FontSizeConvert()
{
    switch( chr_size ) {
        case WIDE:
            lpx=2;
            lpy=1;
            break;
        case HIGH:
            lpx=1;
            lpy=2;
            break;
        case WH  :
            lpx=2;
            lpy=2;
            break;
        case WHx36:
            lpx=6;
            lpy=6;
            break;
        case NORMAL:
        default:
            lpx=1;
            lpy=1;
            break;
    }
}

void
devSSD1331init(int new_temp, int new_hum, int new_IAQ_score)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");

	/*
	 * 	Fill the screen with green rectangle
	 */
/*
	writeCommand(0x22);    //draw rectangle
	writeCommand(0x00);    //column start address
	writeCommand(0x00);    //row start address
	writeCommand(0x5F);    //column end address
        writeCommand(0x3F);    //row end address
	writeCommand(0x00);    //color c of the line
	writeCommand(0xFF);    //color b of the line
	writeCommand(0x00);    //color a of the line
	writeCommand(0x00);    //color c of the fill area
	writeCommand(0xFF);    //color b of the fill area
	writeCommand(0x00);    //color a of the fill area
*/	
	
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");

	/*
	 * 	Draw out the screen with temperature, humidity and IAQ readings, 
	 * 	with an indication of smiley face or exclamation mark depending on 
	 * 	the state of the system
	 */
	chr_size = HIGH;
        FontSizeConvert();
        locate(3,10);
        writeString("T:");

        locate(17,10);
        uint16_t value1 = 1;
        display(17,10,new_temp,value1);

        locate(33,10);
        writeString("degC");

        locate(3,30);
        writeString("H:");

        locate(17,30);
        uint16_t value2 = 1;
        display(17,30,new_hum,value2);

        locate(33,30);
        writeString("%rH");

        locate(3,50);
        writeString("IAQ:");

        locate(30,50);
        uint16_t value3 = 1;
        display(17,50,new_IAQ_score,value3);

        if((new_temp >= 20) & (new_temp <= 25) & (new_hum >= 40) & (new_hum <= 60) & (new_IAQ_score <50))
        {
        devSSD1331DrawGreenFace();
        }

        else if((new_temp >= 20) & (new_temp <= 25) & (new_hum >= 40) & (new_hum <= 60) & (50 <= new_IAQ_score) & (new_IAQ_score < 100))
        {
        devSSD1331DrawYellowFace();
        }

        else
        {
        locate(80,25);
        writeString("!");
        }
}

/*
 *	Draw a rectangle or line from (x0, y0) to (x1, y1) of colour (C, B, A)
 */
void 	
drawRectLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t C, uint8_t B, uint8_t A, bool rect)
{
        if (rect == 1)
	{
	writeCommand(0x22); 	// Draw Rectangle
	} 
	else 
	{
	writeCommand(0x21);	// Draw Line
	}
        writeCommand(x0);      // Start column address
        writeCommand(y0);      // Start row address
        writeCommand(x1);      // End column address
        writeCommand(y1);      // End row address
	writeCommand(C);       // Set outline colour C
        writeCommand(B);       // Set outline colour B
        writeCommand(A);       // Set outline colour A
        writeCommand(C);       // Set fill colour C
        writeCommand(B);       // Set fill colour B
        writeCommand(A);       // Set fill colour A
}

void	
devSSD1331DrawGreenFace()
{
        drawRectLine(0x4B, 0x1D, 0x4C, 0x20, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x53, 0x1D, 0x54, 0x20, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x4E, 0x2A, 0x51, 0x2B, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x4C, 0x29, 0x4D, 0x2A, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x52, 0x29, 0x53, 0x2a, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x4B, 0x28, 0x4C, 0x29, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x53, 0x28, 0x54, 0x29, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x4A, 0x27, 0x4B, 0x28, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x54, 0x27, 0x55, 0x28, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x49, 0x26, 0x4A, 0x26, 0x00, 0xFF, 0x00, 1);
        drawRectLine(0x55, 0x26, 0x56, 0x26, 0x00, 0xFF, 0x00, 1);
}

void
devSSD1331DrawYellowFace()
{
        drawRectLine(0x4B, 0x1D, 0x4C, 0x20, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x53, 0x1D, 0x54, 0x20, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x4E, 0x2A, 0x51, 0x2B, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x4C, 0x29, 0x4D, 0x2A, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x52, 0x29, 0x53, 0x2a, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x4B, 0x28, 0x4C, 0x29, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x53, 0x28, 0x54, 0x29, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x4A, 0x27, 0x4B, 0x28, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x54, 0x27, 0x55, 0x28, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x49, 0x26, 0x4A, 0x26, 0xFF, 0xFF, 0x00, 1);
        drawRectLine(0x55, 0x26, 0x56, 0x26, 0xFF, 0xFF, 0x00, 1);
}

/*
 *	Borrowed and adapted from https://os.mbed.com/users/star297/code/ssd1331/docs/tip/ssd1331_8h_source.html
 */
void 
writeChar(int value)
{
	uint8_t chMode = 0;
	if(value == '\n') 
	{
	char_x = 0;
	char_y = char_y + Y_height;
	}
	if ((value < 31) || (value > 127)) return;   // test char range
	if (char_x + X_width > width) 
	{
	char_x = 0;
	char_y = char_y + Y_height;
	if (char_y >= height - Y_height) 
	{
	char_y = 0;
	}
	}
	int i,j,w,k,l,xw;
	unsigned char Temp=0;
	j = 0; i = 0;
	w = X_width;
	xw = X_width;

	for(i=0; i<xw; i++) 
	{
		for ( l=0; l<lpx; l++) 
		{
			Temp = character[value-32][i];
			for(j=Y_height-1; j>=0; j--) 
			{
				for (k=0; k<lpy; k++) 
				{
					chMode = Temp & 0x80? 1 : 0;
					pixel(char_x+(i*lpx)+l, char_y+(((j+1)*lpy)-1)-k,chMode);
				}
				Temp = Temp << 1;
			}
		}
	}
	char_x += (w*lpx);
}

void 
locate(uint8_t column, uint8_t row)
{
    char_x  = column;
    char_y = row;
}

/*
 *	Borrowed and adapted from https://os.mbed.com/users/star297/code/ssd1331/docs/tip/ssd1331_8h_source.html
 */
void 
pixel(uint8_t x,uint8_t y, char colour)
{
	if (colour)
	{
		unsigned char cmd[7]= {kSSD1331CommandSETCOLUMN,0x00,0x00,kSSD1331CommandSETROW,0x00,0x00};

		if ((x>width)||(y>height)) return ;

		cmd[1] = x;
		cmd[2] = x;
		cmd[4] = y;
		cmd[5] = y;
		writeCommandMulti(cmd,6);

		uint16_t white = 0xffff;

		writeData(white);
	}
	else
		return;
}

/*
 *	Borrowed and adapted from https://electropeak.com/learn/the-beginners-guide-to-display-text-image-animation-on-oled-display-by-arduino/
 */
void 
writeString(const char *pString)
{
    while (*pString != '\0')
    {       
	int charAscii = (int)*pString;
        writeChar(charAscii);
        pString++;
    }
}

void 
writeInt(int* pString, int size)
{
    for (int i=0; i<size;i++)
	{
	int charAscii = pString[i]+48;
        writeChar(charAscii);
    }
}

void 
display(int x, int y, uint16_t val, uint16_t prevVal)
{
	if (val != prevVal)
	{
		unsigned int digitsCurrent=countDigits(val);
		unsigned int digitsOld=countDigits(prevVal);
		if (digitsCurrent != digitsOld) // If the length of the numbers are different, rewrite the whole string
		{
			int splitCurrent1[6];
			splitInt(splitCurrent1,val);
			locate(x,y);
			clearScreen(char_x, y ,char_x+(X_width*lpx),y+Y_height*lpy);
			writeInt(splitCurrent1,digitsCurrent);
		}
		else
		{
			int splitCurrent[6];
			int splitPrev[6];
			splitInt(splitCurrent, val);
			splitInt(splitPrev, prevVal);

			for (unsigned int j=0;j<digitsCurrent; j++)
			{
				if (splitCurrent[j]!= splitPrev[j])
				{
					locate(x,y);
					char_x += (j)*(X_width*lpx);
					clearScreen(char_x, y ,char_x+(X_width*lpx),y+Y_height*lpy);

					int charAscii = splitCurrent[j]+48;
					writeChar(charAscii);
				}
			}
		}
	}
	if (val==0)
	{
		locate(x,y);
		int charAscii = val + 48;
		writeChar(charAscii);
	}
	displayedNumber = val;
}

void 
clearScreen(uint8_t x_start, uint8_t y_start,uint8_t x_end,uint8_t y_end)
{
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(x_start);
	writeCommand(y_start);
	writeCommand(x_end);
	writeCommand(y_end);
}

uint16_t countDigits(uint16_t i)
{
	uint16_t digits=1;
	while (i/=10) digits++;
	return digits;
}

void 
splitInt(int *arr, int num)
{
	int temp,factor=1;
	int counter =0;
	temp=num;

	while(temp)
	{
		temp=temp/10;
		factor = factor*10;
	}

	while(factor>1)
	{
		factor = factor/10;
		arr[counter]= num/factor;
		num = num % factor;
		counter++;

	}
}
