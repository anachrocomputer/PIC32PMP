/* pmptest --- test Parallel Master Port on PIC32 dev board 2019-02-08 */
/* Copyright (c) 2019 John Honniball. All rights reserved              */

/*
 * Created: 2019-02-08 21:02
 */


// PIC32MX250F256L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (4x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


#define LED1        LATAbits.LATA6
#define LED2        LATBbits.LATB9


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


/* PMP_begin --- initialise the Parallel Master Port as an 8-bit Z80-style bus */

static void PMP_begin(void)
{
    PMMODEbits.MODE16 = 0;  // 8 data bit mode
    PMMODEbits.WAITB = 3;   // Setup time wait states
    PMMODEbits.WAITM = 15;  // Read/write strobe width
    PMMODEbits.WAITE = 3;   // Hold time wait states
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


/* PMP_write8 --- write eight characters to the pair of HMDL-2416 displays */

void PMP_write8(const char msg[])
{
    int i;
    
    // PMP write cycles: four to CS1 and four to CS2
    PMADDRbits.CS1 = 1;     // PMCS1 active
    PMADDRbits.CS2 = 0;     // PMCS2 inactive

    for (i = 0; i < 4; i++)
    {
        while (PMMODEbits.BUSY)
            ;

        PMADDRbits.ADDR = 3 - i;
        PMDIN = msg[i];
    }

    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
    PMADDRbits.CS2 = 1;     // PMCS2 active

    for (i = 0; i < 4; i++)
    {
        while (PMMODEbits.BUSY)
            ;

        PMADDRbits.ADDR = 3 - i;
        PMDIN = msg[i + 4];
    }

    while (PMMODEbits.BUSY)
            ;

    PMADDRbits.CS1 = 0;     // PMCS1 inactive
    PMADDRbits.CS2 = 0;     // PMCS2 inactive
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


/* TRIS_begin --- switch GPIO pins to input or output as required */

static void TRIS_begin(void)
{
    TRISAbits.TRISA7 = 0;   // RA7 pin 92, P3 pin 42 as output (timer toggle)
    TRISAbits.TRISA6 = 0;   // RA6 pin 91, P3 pin 41 as output (LED1)
    TRISBbits.TRISB9 = 0;   // RB9 pin 33, P2 pin 33 as output (LED2)
}


void main(void)
{
    uint8_t ch;
    int i;
    volatile int junk;
    char msg[16];
    bool flag = false;
    
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
    
    puts("PMPTEST");
    
    while (1)
    {
        LED1 = 0;
        LED2 = 1;
        
        // PMP read cycle
        PMADDR = 0;
        junk = PMDIN;
        
        delayms(500);
        
        if (UART3RxAvailable())
        {
            ch = UART3RxByte();
        }
        else
        {
            ch = '*';
        }
               
        LED1 = 1;
        LED2 = 1;
        
        if (flag)
        {
            strcpy(msg, "FARTARSE");
        }
        else
        {
            strcpy(msg, "ARSEFART");
        }
        
        flag = !flag;
        
        PMP_write8(msg);
                
        delayms(500);
        
        LED1 = 0;
        LED2 = 0;
        
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
        
        delayms(500);
        
        LED1 = 1;
        LED2 = 0;
        
        delayms(500);
    }
}

