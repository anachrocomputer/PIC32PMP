/* ili9486test --- ILI9486 parallel interfaced LCD test     2020-10-21 */
/* Copyright (c) 2020 John Honniball. All rights reserved              */

/*
 * Created: 2020-10-21 22:28
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// PIC32MX360F512L Configuration Bit Settings
 
// 'C' source line config statements
 
// DEVCFG3
// USERID = No Setting
 
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)
#pragma config UPLLIDIV = DIV_4         // 
#pragma config UPLLEN = ON              // 
 
// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
 
// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // 
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
 
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
 
#include <xc.h>
#include <sys/attribs.h>

// Fonts: Inconsolata
// convert -resize 32x32\! -font Liberation-Mono +dither -pointsize 144 label:B B.xbm

// PIC32 with 128k RAM: PIC32MX795F512L-80I/PT

#include "P1030550_tiny.h"

#define LED1        LATAbits.LATA6
#define LED2        LATBbits.LATB9

#define GRAT_HT  (256)
#define GRAT_WD  (256)

#define GRATICULE (32)

typedef uint16_t iliColr;

// 16-bit 5-6-5 RGB colour definitions
#define ILI9486_BLACK   (0x0000u)
#define ILI9486_BLUE    (0x001Fu)
#define ILI9486_RED     (0xF800u)
#define ILI9486_GREEN   (0x07E0u)
#define ILI9486_CYAN    (0x07FFu)
#define ILI9486_MAGENTA (0xF81Fu)
#define ILI9486_YELLOW  (0xFFE0u)
#define ILI9486_WHITE   (0xFFFFu)

#define ILI9486_ORANGE  0xFC00

// LCD controller commands
#define ILI9486_NOP         (0x00)  // No operation
#define ILI9486_RESET       (0x01)  // Soft Reset
#define ILI9486_READID      (0x04)  // Read display identification information
#define ILI9486_READNERRORS (0x05)  // Read number of errors on DSI
#define ILI9486_READSTATUS  (0x09)  // Read Display Status
#define ILI9486_SLEEP_IN    (0x10)  // Sleep IN
#define ILI9486_SLEEP_OUT   (0x11)  // Sleep OUT
#define ILI9486_DISPLAY_OFF (0x28)  // Display off
#define ILI9486_DISPLAY_ON  (0x29)  // Display on
#define ILI9486_COL_ADDR    (0x2a)  // Column Address Set
#define ILI9486_PAGE_ADDR   (0x2b)  // Page Address Set
#define ILI9486_MEMORY_WR   (0x2c)  // Memory Write
#define ILI9486_MEMORY_RD   (0x2e)  // Memory Read
#define ILI9486_PARTIALAREA (0x30)  // Partial Area
#define ILI9486_MEMACCTRL   (0x36)  // Memory Access Control
#define ILI9486_IFPIXELFMT  (0x3a)  // Interface Pixel Format
#define ILI9486_IFMODECTRL  (0xb0)  // Interface Mode Control
#define ILI9486_POWERCTRL1  (0xc0)  // Power Control 1
#define ILI9486_POWERCTRL2  (0xc1)  // Power Control 2
#define ILI9486_POWERCTRL3  (0xc2)  // Power Control 3
#define ILI9486_POWERCTRL4  (0xc3)  // Power Control 4
#define ILI9486_POWERCTRL5  (0xc4)  // Power Control 5
#define ILI9486_VCOMCTRL1   (0xc5)  // VCOM Control 1
#define ILI9486_PGAMCTRL    (0xe0)  // PGAMCTRL (Positive Gamma Control)
#define ILI9486_NGAMCTRL    (0xe1)  // NGAMCTRL (Negative Gamma Control)
#define ILI9486_DGAMCTRL1   (0xe2)  // DGAMCTRL1 (Digital Gamma Control 1)
#define ILI9486_DGAMCTRL2   (0xe3)  // DGAMCTRL2 (Digital Gamma Control 2)

#define RGB_ILI(r, g, b)  (((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3))

struct ColourScheme {
  iliColr trace1;
  iliColr trace2;
  iliColr xcursor1;
  iliColr xcursor2;
  iliColr ycursor1;
  iliColr ycursor2;
  iliColr trig;
  iliColr graticule;
  iliColr background;
};

const struct ColourScheme LCDColours = {
  .trace1 = ILI9486_YELLOW,
  .trace2 = ILI9486_CYAN,
  .xcursor1 = ILI9486_WHITE,
  .xcursor2 = ILI9486_WHITE,
  .ycursor1 = ILI9486_GREEN,
  .ycursor2 = ILI9486_GREEN,
  .trig = ILI9486_ORANGE,
  .graticule = RGB_ILI(128, 128, 128),
  .background = ILI9486_BLACK,
};

iliColr C1grat[GRAT_HT];
iliColr C2grat[GRAT_HT];
iliColr Bgrat[GRAT_HT];
iliColr Dgrat[GRAT_HT];
iliColr Tgrat[GRAT_HT];
iliColr Wgrat[GRAT_HT];
    
#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
static struct UART_BUFFER U3Buf;

volatile uint32_t MilliSeconds = 0;


/* dally --- CPU busy-loop for crude time delay */

static void dally(const int loops)
{
    volatile int dally;
    
    for (dally = 0; dally < loops; dally++)
        ;
}


/* delayms --- busy-wait delay for given number of milliseconds */

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}


void __ISR(_TIMER_1_VECTOR, ipl2AUTO) Timer1Handler(void) 
{
    MilliSeconds++;
    
    LATAINV = _LATA_LATA7_MASK; // Toggle RA7, P3 pin 42 (500Hz)
    
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
}


void __ISR(_UART_3_VECTOR, ipl1AUTO) UART3Handler(void) 
{
    if (IFS2bits.U3TXIF)
    {
        if (U3Buf.tx.head != U3Buf.tx.tail) // Is there anything to send?
        {
            const uint8_t tmptail = (U3Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
            
            U3Buf.tx.tail = tmptail;

            U3TXREG = U3Buf.tx.buf[tmptail];     // Transmit one byte
        }
        else
        {
            IEC2CLR = _IEC2_U3TXIE_MASK;         // Nothing left to send; disable Tx interrupt
        }
        
        IFS2CLR = _IFS2_U3TXIF_MASK;  // Clear UART3 Tx interrupt flag
    }
    
    if (IFS1bits.U3RXIF)
    {
        const uint8_t tmphead = (U3Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
        const uint8_t ch = U3RXREG;   // Read received byte from UART
        
        if (tmphead == U3Buf.rx.tail)   // Is receive buffer full?
        {
             // Buffer is full; discard new byte
        }
        else
        {
            U3Buf.rx.head = tmphead;
            U3Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
        }
        
        IFS1CLR = _IFS1_U3RXIF_MASK;  // Clear UART3 Rx interrupt flag
    }
    
    if (IFS1bits.U3EIF)
    {
        IFS1CLR = _IFS1_U3EIF_MASK;   // Clear UART3 error interrupt flag
    }
}


static void UART3_begin(const int baud)
{
    U3Buf.tx.head = 0;
    U3Buf.tx.tail = 0;
    U3Buf.rx.head = 0;
    U3Buf.rx.tail = 0;

    U3MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U3STAbits.UTXEN = 1;    // Enable Tx
    U3STAbits.URXEN = 1;    // Enable Rx
    
    U3BRG = (40000000 / (baud * 16)) - 1;
    
    IPC9bits.U3IP = 1;          // UART3 interrupt priority 1
    IPC9bits.U3IS = 0;          // UART3 interrupt sub-priority 0
    
    IFS2CLR = _IFS2_U3TXIF_MASK;  // Clear UART3 Tx interrupt flag
    IFS1CLR = _IFS1_U3RXIF_MASK;  // Clear UART3 Rx interrupt flag
    IFS1CLR = _IFS1_U3EIF_MASK;   // Clear UART3 error interrupt flag
    
    IEC1SET = _IEC1_U3RXIE_MASK;  // Enable UART3 Rx interrupt
    IEC1SET = _IEC1_U3EIE_MASK;   // Enable UART3 error interrupt
    
    U3MODESET = _U3MODE_ON_MASK;      // Enable USART3
}

uint8_t UART3RxByte(void)
{
    const uint8_t tmptail = (U3Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
    
    while (U3Buf.rx.head == U3Buf.rx.tail)  // Wait, if buffer is empty
        ;
    
    U3Buf.rx.tail = tmptail;
    
    return (U3Buf.rx.buf[tmptail]);
}


void UART3TxByte(const uint8_t data)
{
    const uint8_t tmphead = (U3Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
    
    while (tmphead == U3Buf.tx.tail)   // Wait, if buffer is full
        ;

    U3Buf.tx.buf[tmphead] = data;
    U3Buf.tx.head = tmphead;

    IEC2SET = _IEC2_U3TXIE_MASK;       // Enable UART3 Tx interrupt
}


bool UART3RxAvailable(void)
{
    return (U3Buf.rx.head != U3Buf.rx.tail);
}


void _mon_putc(const char ch)
{
    // See: https://microchipdeveloper.com/faq:81
    if (ch == '\n')
    {
        UART3TxByte('\r');
    }
    
    UART3TxByte(ch); // Connect stdout to UART3
}


/* PMP_begin --- initialise the Parallel Master Port as a 16-bit Z80-style bus */

static void PMP_begin(void)
{
    PMMODEbits.MODE16 = 1;  // 16 data bit mode
    PMMODEbits.WAITB = 0;   // Setup time wait states
    PMMODEbits.WAITM = 0;   // Read/write strobe width
    PMMODEbits.WAITE = 0;   // Hold time wait states
    PMMODEbits.IRQM = 0;    // No interrupts
    PMMODEbits.INCM = 0;    // No auto-increment
    PMMODEbits.MODE = 2;    // /WR and /RD mode (Z80 style bus)
    
    PMCONbits.ADRMUX = 0;   // Non-multiplexed address bus
    PMCONbits.PTRDEN = 1;   // Enable the PMRD pin (pin 82, P3 pin 32)
    PMCONbits.PTWREN = 1;   // Enable the PMWR pin (pin 81, P3 pin 31)
    PMCONbits.RDSP = 0;     // PMRD active-LOW
    PMCONbits.WRSP = 0;     // PMWR active-LOW
    PMCONbits.CSF = 2;      // PMCS1 and PMCS2 are Chip Select pins
    PMCONbits.CS1P = 0;     // PMCS1 active-LOW
    PMCONbits.CS2P = 0;     // PMCS2 active-LOW
    
    PMAENbits.PTEN = 0x03 | (1 << 14) | (1 << 15);
    
    PMADDRbits.CS1 = 0;     // PMCS1 inactive (pin 71, P3 pin 21)
    PMADDRbits.CS2 = 0;     // PMCS2 inactive (pin 70, P3 pin 20)
    
    PMCONbits.ON = 1;       // Enable the Parallel Master Port module
}


/* PPS_begin --- map Peripheral Pin Select to suit dev board */

static void PPS_begin(void)
{
    /* Configure USART3 */
    RPC1Rbits.RPC1R = 1;    // U3Tx on pin 6, RPC1, P2 pin 6
    U3RXRbits.U3RXR = 10;   // U3Rx on pin 9, RPC4, P2 pin 9 (5V tolerant)
    
    /* Configure SPI1 */
    // SCK1 on pin 70, RD10 clashes with PMP
    //SDI1Rbits.SDI1R = 0;   // SDI1 on RPD3
    //RPC13Rbits.RPC13R = 8; // SDO1 on RPC13
    
    /* Configure SPI2 */
    // SCK2 on pin 10, RG6 clashes with PMP
    //SDI2Rbits.SDI2R = 0;   // SDI2 on RPD3, pin 78
    //RPC13Rbits.RPC13R = 6; // SDO2 on RPC13, pin 73
    
    /* Configure SPI3 */
    // SCK3 on pin 39, RF13, P2 pin 39
    //SDI3Rbits.SDI3R = 0;   // SDI3 on RPD2, pin 77
    //RPG8Rbits.RPG8R = 14;  // SDO3 on RPG8, pin 12 clashes with PMP
    
    /* Configure SPI4 */
    // SCK4 on pin 48, RD15, P2 pin 48
}


/* iliCmd0 --- send a command byte with no parameter bytes */

void iliCmd0(const uint8_t cmd)
{
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = cmd;
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
}


/* iliCmd1 --- send a command byte followed by 1 parameter byte */

void iliCmd1(const uint8_t cmd, const uint8_t arg1)
{
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = cmd;
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 1;
    PMDIN = arg1;
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
}


/* iliCmd4 --- send a command byte followed by 4 parameter bytes */

void iliCmd4(const uint8_t cmd, const uint8_t arg1, const uint8_t arg2, const uint8_t arg3, const uint8_t arg4)
{
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = cmd;
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 1;
    PMDIN = arg1;
    
    while (PMMODEbits.BUSY)
        ;
    
    PMDIN = arg2;
    
    while (PMMODEbits.BUSY)
        ;
    
    PMDIN = arg3;
    
    while (PMMODEbits.BUSY)
        ;
    
    PMDIN = arg4;
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
}


/* iliCmd15 --- send a command byte followed by 15 parameter bytes */

void iliCmd15(const uint8_t cmd, const uint8_t arg[15])
{
    int i;
    
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = cmd;
    
    for (i = 0; i < 15; i++)
    {
        while (PMMODEbits.BUSY)
            ;
    
        PMADDRbits.ADDR = 1;
        PMDIN = arg[i];
    }
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
}


/* ili9486_begin --- initialise the 16-bit parallel LCD */

void ili9486_begin(void)
{
    static uint8_t pgamma[15] = {0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48,
                                 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00};
    static uint8_t ngamma[15] = {0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47,
                                 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00};
    static uint8_t dgamma[15] = {0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47,
                                 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00};
    LATDbits.LATD3 = 1;
    delayms(20);
    LATDbits.LATD3 = 0;     // Assert display RESET, active LOW
    delayms(20);
    LATDbits.LATD3 = 1;
    delayms(20);
    
    iliCmd1(ILI9486_IFMODECTRL, 0x00); // Interface Mode Control
    iliCmd0(ILI9486_SLEEP_OUT);        // Sleep OUT
    delayms(250);
    
    iliCmd1(ILI9486_IFPIXELFMT, 0x55); // Interface Pixel Format, 16 bits / pixel
    iliCmd1(ILI9486_POWERCTRL3, 0x44); // Power Control 3 (For Normal Mode)
    iliCmd4(ILI9486_VCOMCTRL1, 0x00, 0x00, 0x00, 0x00); // VCOM Control 1
    iliCmd15(ILI9486_PGAMCTRL, pgamma);   // PGAMCTRL (Positive Gamma Control)
    iliCmd15(ILI9486_NGAMCTRL, ngamma);   // NGAMCTRL (Negative Gamma Control)
    iliCmd15(ILI9486_DGAMCTRL1, dgamma);  // Digital Gamma Control 1
    iliCmd1(ILI9486_MEMACCTRL, 0x48);  // Memory Access Control, MX | BGR
    
    iliCmd0(ILI9486_DISPLAY_ON);       // Display ON
    PMDIN = 0xffff; // Keep data bus high to work around a hardware problem
    delayms(250);
}


/* ili9486_fill --- fill display memory with a given pixel */

void ili9486_fill(const uint16_t pixel, const uint32_t n)
{
    int i;
    
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = ILI9486_MEMORY_WR;
    
    while (PMMODEbits.BUSY)
        ;        
    
    PMADDRbits.ADDR = 1;
    
    for (i = 0; i < n; i++)
    {
        while (PMMODEbits.BUSY)
            ;

        PMDIN = pixel;
    }
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
    PMDIN = 0xffff; // Keep data bus high to work around a hardware problem
}


/* ili9486_write --- write pixels to the display memory, from a buffer */

void ili9486_write(const uint16_t buf[], const uint32_t n)
{
    int i;
    
    PMADDRbits.CS1 = 1;     // PMCS1 active
    
    while (PMMODEbits.BUSY)
        ;
    
    PMADDRbits.ADDR = 0;
    PMDIN = ILI9486_MEMORY_WR;
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.ADDR = 1;
    
    for (i = 0; i < n; i++)
    {
        while (PMMODEbits.BUSY)
            ;

        PMDIN = buf[i];
    }
    
    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
    PMDIN = 0xffff; // Keep data bus high to work around a hardware problem
}


/* ili9486_fillRect --- fill a rectangle at given co-ordinates with a pixel colour */

void ili9486_fillRect(const int x1, const int y1, const int x2, const int y2, const uint16_t colr)
{
    iliCmd4(ILI9486_COL_ADDR, x1 >> 8, x1, x2 >> 8, x2);
    iliCmd4(ILI9486_PAGE_ADDR, y1 >> 8, y1, y2 >> 8, y2);
    ili9486_fill(colr, (1 + x2 - x1) * (1 + y2 - y1));
}


/* ili9486_pixMap --- write pixels to the display memory, in a rectangle */

void ili9486_pixMap(const int x1, const int y1, const int wd, const int ht, const uint16_t *const pixMap)
{
    const int x2 = x1 + wd - 1;
    const int y2 = y1 + ht - 1;
    
    iliCmd4(ILI9486_COL_ADDR, x1 >> 8, x1, x2 >> 8, x2);
    iliCmd4(ILI9486_PAGE_ADDR, y1 >> 8, y1, y2 >> 8, y2);
    ili9486_write(pixMap, wd * ht);
}


/* TRIS_begin --- switch GPIO pins to input or output as required */

static void TRIS_begin(void)
{
    TRISAbits.TRISA7 = 0;   // RA7 pin 92, P3 pin 42 as output (timer toggle)
    TRISAbits.TRISA6 = 0;   // RA6 pin 91, P3 pin 41 as output (LED1)
    TRISBbits.TRISB9 = 0;   // RB9 pin 33, P2 pin 33 as output (LED2)
    
    TRISDbits.TRISD3 = 0;   // RD3 pin 78, P3 pin 28 as output (LCD RESET)
}

void generateGraticule(const struct ColourScheme *cp)
{
  int y;
  
  for (y = 0; y < GRAT_HT; y++) {
    Wgrat[y] = cp->graticule;
    
    if ((y % GRATICULE) == 0)
      Bgrat[y] = cp->graticule;
    else
      Bgrat[y] = cp->background;
    
    if ((y % 8) == 0)
      Dgrat[y] = cp->graticule;
    else
      Dgrat[y] = cp->background;
    
    if ((y == 1) || (y == ((GRAT_HT / 2) + 1)) || (y == ((GRAT_HT / 2) - 1)) || (y == (GRAT_HT - 2)) || ((y % GRATICULE) == 0))
      Tgrat[y] = cp->graticule;
    else
      Tgrat[y] = cp->background;
    
    if ((y % 2) == 0)
      C1grat[y] = cp->xcursor1;
    else
      C1grat[y] = cp->background;
    
    if ((y % 2) == 0)
      C2grat[y] = cp->xcursor2;
    else
      C2grat[y] = cp->background;
   }
}

void interpolateY(const uint8_t wave[], const int x, iliColr pixels[], const iliColr colr)
{
  int y1, y2;
  int y;
  
  y1 = wave[x];

  if (x < (GRAT_WD - 1))
    y2 = wave[x + 1];
  else
    y2 = y1;

  if ((y1 > 0) && (y1 < GRAT_HT)) {
    if (y2 < y1) {
      y = y1;
      y1 = y2;
      y2 = y;
    }
    
    for (y = y1; y <= y2; y++)
      pixels[y] = colr;
  }
}

void interpolateY2(const uint8_t wave[], const int x, const int yoff, iliColr pixels[], const iliColr colr)
{
  int y1, y2;
  int y;
  
  y1 = wave[x] + yoff;

  if (x < (GRAT_WD - 1))
    y2 = wave[x + 1] + yoff;
  else
    y2 = y1;

  if ((y1 > 0) && (y1 < GRAT_HT) && (y2 > 0) && (y2 < GRAT_HT)) {
    if (y2 < y1) {
      y = y1;
      y1 = y2;
      y2 = y;
    }
    
    for (y = y1; y <= y2; y++)
      pixels[y] = colr;
  }
}

void main(void)
{
    unsigned int before, after;
    int x;
    iliColr pixels[GRAT_HT];
    uint8_t wave1[GRAT_WD];
    uint8_t wave2[GRAT_WD];
    float theta1, delta1;
    float theta2, delta2;
    float offset1 = 128.0;
    float offset2 = 180.0;
    bool cursorMode = true;
    int xcursor1 = 34, xcursor2 = 241;
    int ycursor1 = 54, ycursor2 = 237;
    int yshift1 = 22;
    static uint16_t pixMap[64] = {0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f,  // blue
                                  0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f, 0x001f,  // blue
                                  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,  // green
                                  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,  // green
                                  0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800,  // red
                                  0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800,  // red
                                  0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000,  // black/white
                                  0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff}; // black/white
    
    /* Set up peripherals to match pin connections on PCB */
    PPS_begin();
    
    /* Configure tri-state registers */
    TRIS_begin();
    
    UART3_begin(9600);
    
    PMP_begin();
    
    /* Configure Timer 1 */
    T1CONbits.TCKPS = 0;        // Timer 1 prescale: 1
    
    TMR1 = 0x00;                // Clear Timer 1 counter
    PR1 = 39999;                // Interrupt every 40000 ticks (1ms)
    
    T1CONbits.ON = 1;           // Enable Timer 1
    
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
    
    IPC1bits.T1IP = 2;          // Timer 1 interrupt priority 2
    IPC1bits.T1IS = 1;          // Timer 1 interrupt sub-priority 1
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
    IEC0SET = _IEC0_T1IE_MASK;  // Enable Timer 1 interrupt
    
    __asm__("EI");              // Global interrupt enable
    
    puts("ILI9486TEST");
    
    ili9486_begin();
    
    generateGraticule(&LCDColours);
    
    // Synthesise two dummy waveforms
    delta1 = (M_PI * 4.0) / (float)GRAT_WD;
    delta2 = (M_PI * 8.0) / (float)GRAT_WD;

    for (x = 0; x < GRAT_WD; x++) {
        theta1 = (float)x * delta1;
        theta2 = (float)x * delta2;

        wave1[x] = (sin(theta1) * 50.0) + offset1;
        wave2[x] = (sin(theta2) * 50.0) + offset2;
    }
   
    
    ili9486_fill(ILI9486_WHITE, 480u * 320u);
    
    while (1)
    {
        LED1 = 0;
        LED2 = 1;
        ili9486_fillRect(0, 0, 319, 0, ILI9486_WHITE);
        
        delayms(500);
               
        LED1 = 1;
        LED2 = 1;
        ili9486_pixMap(8, 8, 8, 8, pixMap);
        ili9486_pixMap((320 - 64) / 2, 8, 64, 64, (const uint16_t *)Image);
        
        before = millis();
        ili9486_fillRect(0, 80, 319, 319 + 80, ILI9486_ORANGE);
        after = millis();
        
        ili9486_fillRect(32, 112, 255 + 32, 255 + 112, ILI9486_WHITE);
        ili9486_fillRect(64, 144, 191 + 64, 191 + 144, ILI9486_RED);
        ili9486_fillRect(96, 176, 127 + 96, 127 + 176, ILI9486_GREEN);
        ili9486_fillRect(128, 208, 63 + 128, 63 + 208, ILI9486_BLUE);
        printf("\n320x320 pixels took %dms\n", after - before);
                
        delayms(500);
        
        LED1 = 0;
        LED2 = 0;
        ili9486_fillRect(0, 0, 319, 0, ILI9486_MAGENTA);
        
        delayms(500);
        
        if (PORTAbits.RA1 == 0)
        {
            fputs("PRESSED!", stdout);
        }
        else
        {
            printf("%X", 0xdeadbeef);
        }
        
        LED1 = 1;
        LED2 = 0;
        
        before = millis();
        
        for (x = 0; x < GRAT_WD; x++)
        {
            // Draw the graticule, cursors or background
            if ((cursorMode == true) && (x == xcursor1))
              memcpy(pixels, C1grat, sizeof (pixels));
            else if ((cursorMode == true) && (x == xcursor2))
              memcpy(pixels, C2grat, sizeof (pixels));
            else if ((x == 1) || (x == ((GRAT_WD / 2) + 1)) || (x == ((GRAT_WD / 2) - 1)) || (x == (GRAT_WD - 2)))
              memcpy(pixels, Dgrat, sizeof (pixels));
            else if ((x % GRATICULE) == 0)
              memcpy(pixels, Wgrat, sizeof (pixels));
            else if ((x % 8) == 0)
              memcpy(pixels, Tgrat, sizeof (pixels));
            else
              memcpy(pixels, Bgrat, sizeof (pixels));
            
            // Add two Y-cursors
            if ((cursorMode == true) && ((x % 2) == 0)) {
              pixels[ycursor1] = LCDColours.ycursor1;
              pixels[ycursor2] = LCDColours.ycursor2;
            }
            
            // Add the two waveforms
            interpolateY2(wave1, x, yshift1, pixels, LCDColours.trace1);
            interpolateY(wave2, x, pixels, LCDColours.trace2);
        
            ili9486_pixMap(x + 32, 112, 1, GRAT_HT, pixels);
        }
        
        after = millis();
        printf("\n%dx%d scope display took %dms\n", GRAT_WD, GRAT_HT, after - before);
        
        ili9486_fillRect(0, 0, 319, 0, ILI9486_GREEN);
        
        delayms(500);
        
        LED1 = 1;
        LED2 = 0;
        ili9486_fillRect(0, 0, 319, 0, ILI9486_ORANGE);
        
        delayms(500);
    }
}
