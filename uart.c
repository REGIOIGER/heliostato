#include <msp430.h>
#include <math.h>

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

const char string[] = { "Hello World\r\n" };
unsigned int i; //Counter

float x,sx,cx;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT

    /* Use Calibration values for 1MHz Clock DCO*/
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    /* Configure Pin Muxing P1.1 RXD and P1.2 TXD */
    P1SEL = BIT1 | BIT2 ;
    P1SEL2 = BIT1 | BIT2;

    P2DIR |= 0xFF; // All P2.x outputs
    P2OUT &= 0x00; // All P2.x reset

    /* Place UCA0 in Reset to be configured */
    UCA0CTL1 = UCSWRST;

    /* Configure */
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 104; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1

    /* Take UCA0 out of reset */
    UCA0CTL1 &= ~UCSWRST;

    /* Enable USCI_A0 RX interrupt */
    IE2 |= UCA0RXIE;

    __bis_SR_register(LPM0_bits + GIE); // Enter LPM0, interrupts enabled

while(1){}
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
   P1OUT |= TXLED; 
     UCA0TXBUF = string[i++]; // TX next character 
    if (i == sizeof string - 1) // TX over? 
       UC0IE &= ~UCA0TXIE; // Disable USCI_A0 TX interrupt 
    P1OUT &= ~TXLED; } 
  
#pragma vector=USCIAB0RX_VECTOR 
__interrupt void USCI0RX_ISR(void) 
{ 
   P1OUT |= RXLED; 
    if (UCA0RXBUF == 'a') // 'a' received?
    { 
       i = 0; 
       UC0IE |= UCA0TXIE; // Enable USCI_A0 TX interrupt 
      UCA0TXBUF = string[i++];
        for(x = 0; x < 10; x++){
        sx = sin(x);
        cx = cos(x);
        }
 
    } 
    P1OUT &= ~RXLED;
}
