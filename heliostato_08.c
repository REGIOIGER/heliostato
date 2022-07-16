#include  <msp430g2553.h>   		// System define for the micro that I am using

#define RXD    BIT1 	    		//Check your launchpad rev to make sure this is the case. Set jumpers to hardware uart.
#define TXD    BIT2 	    		//TXD with respect to what your sending to the computer. Sent data will appear on this line
#define BUTTON BIT3
					//Puerto 0 para control de motores de paso
#define LED_0  BIT0			//LED rojo
#define mElDir BIT4			//Direccion del motor de elevacion
#define mElStp BIT5			//Paso en el motor de elevacion
#define mAzDir BIT3			//Direccion en el motor azimutal
#define mAzStp BIT5			//Paso en el motor azimutal

#define M1CL  13.7 			//cm //Motor 1 Elevacion Cateto Largo
#define M1CC   2.5 			//cm //Motor 1 Elevacion Cateto Corto
#define M2CL  15.0   			//cm //Motor 2 Azimutal Cateto Largo
#define M2CC   5.0   			//cm //Motor 2 Azimutal Cateto Corto
#define D01   11.8   			//cm //longitud inicial de elevacion (d)
#define D02   12.5 			//cm //longitud inicial azimutal (d) 

//#define STPM_OUT P1OUT //Stepper Motor port
//#define STPM_DIR P1DIR

					//Para uso del reloj
#define	ONEBYTEADDR	1
#define	TWOBYTEADDR	2
#define	WRITE		0		// ISR mode WRITE or READ
#define	READ		1
#define	NOACK		2
#define	DS3231_ADDR	0x68


#define PI   3.141592653589793

#define DR  57.29577951     		// degree / radian factor;
#define WH   0.017214206    		// earth's mean orbital angular speed in radians
#define WR   0.261799388    		// earth's speed of rotation relative to sun 
#define E2   0.0334         		// twice earth's orbital eccentricity
#define SN   0.17214206     		// 10 days from December solstice to New Year (Jan $
#define SP   0.206570472    		// 12 days from December solstice to perihelion

#define ST  -0.397948631    		//sin(CC); // sine of reverse tilt
#define CT   0.917407699    		//cos(CC); // cosine of reverse tilt


#define MCU_CLOCK	    1000000
#define PWM_FREQUENCY	    46	    	// In Hertz, ideally 50Hz.
#define SERVO_STEPS	    180	    	// Maximum amount of steps in degrees (180 is common)
#define SERVO_MIN	    650	    	// The minimum duty cycle for this servo
#define SERVO_MAX	    2700	// The maximum duty cycle

					//Para uso con servos

unsigned int PWM_Period	= (MCU_CLOCK / PWM_FREQUENCY);	// PWM Period
unsigned int PWM_Duty	= 0;	

					// Calculate the step value and define the current step, defaults to minimum.
unsigned int servo_stepval 	= ( (SERVO_MAX - SERVO_MIN) / SERVO_STEPS );
unsigned int servo_stepnow	= SERVO_MIN;		

void stpm_init();

					//Valores por default de fecha y hora en el reloj
unsigned char txdataDS3231[7] = {0x00, 0x08, 0x09, 0x07, 0x18, 0x02, 0x17};
unsigned char rxdata[7];

typedef struct {
	volatile unsigned char *data_buf;	// address of tx or rx data buffer
        volatile unsigned char buf_size;	// size of the buffer
        volatile unsigned char buf_index;	// index in the buffer
        volatile unsigned char addr_index;	// index of the byte address (0,1)
        volatile unsigned char isr_mode;	// Tx or Rx affects the interrupt logic
        volatile unsigned char addr_high_byte;	// High byte of the address to read/write to
        volatile unsigned char addr_low_byte;	// Low byte of the address to read/write to
        volatile unsigned char addr_type;	// two bytes like eeprom or 1 byte like RTC for example
    } i2c_t;

i2c_t i2c_packet;

void i2c_init(void);
void i2c_tx(unsigned char, unsigned char *, unsigned char, unsigned char, unsigned char, unsigned char);
void i2c_rx(unsigned char, unsigned char *, unsigned char, unsigned char, unsigned char, unsigned char);
void usdelay(unsigned int);

unsigned char bcdToDec(unsigned char val);
unsigned char decToBcd(unsigned char val);

						// Fucniones para comunicacion UART
void UART_OutUDec(unsigned long n);		// Transmite un entero largo
void UART_char(int tx_data);			// Transmite un caracter
void UART_TX(char * tx_data);            	// Function Prototype for TX
void UART_ConvertDistance(unsigned long n);	// TODO
void UART_OutDistance(unsigned long n);		// TODO

unsigned char String[10];

unsigned char buffer[8];
unsigned char buffer_index;
unsigned char flag_00 = 0;

//long int R;
//float x;
					// Funciones Matematicas
float rad2deg(float radians);
float deg2rad(float degrees);

float rad2len(float rads, float a, float b);

float sin(float value);
float cos(float value);
float tan(float x);
float atan(float value);
float sqrt(float x);

int floor(float x);

float ang(float x, float y); 
void P2C(float AZ, float EL, float *x, float *y, float *z);
void C2P(float x, float y, float z, float *AZ, float *EL);

						
void updateMotorEL(float newL);		// Actualiza motor de elevacion
void updateMotorAZ(float newL);		// Actualiza motor azimutal

float CC = -23.45 / DR; 		// reverse angle of earth's axial tilt in radians

unsigned char agEL,agAZ;

unsigned char hora, minuto;		// Para modo demostracion

					//ANGULOS DEL OBJETIVO
int az = 200;
int el = 12;

float l1 = D01; //0.0;			// TODO: debe ser inicializada correctamente
float l2 = D02; //0.0;			// TODO: debe ser inicializada correctamente

char F = 0;				// establecer reloj F = 1, establecer angulos del heliostato F = 2 (se hace desde la consola UART)

					// Funcion principal del Heliostato

void helios(int Mth, int Day, int Hr, int Mn, int tAZ, int tEL);

void main(void)
{
WDTCTL = WDTPW + WDTHOLD;         	// Stop Watch dog timer

BCSCTL1 = CALBC1_1MHZ;            	// Set DCO to 1 MHz
DCOCTL = CALDCO_1MHZ;

P1SEL = RXD + TXD;                	// Select TX and RX functionality for P1.1 & P1.2
P1SEL2 = RXD + TXD;               	//

UCA0CTL1 |= UCSSEL_2;             	// Have USCI use System Master Clock: AKA core clk 1MHz

UCA0BR0 = 104;                    	// 1MHz 9600, see user manual
UCA0BR1 = 0;                      	//

UCA0MCTL = UCBRS0;                	// Modulation UCBRSx = 1
UCA0CTL1 &= ~UCSWRST;             	// Start USCI state machine

IE2 |= UCA0RXIE;                  	// Enable USCI_A0 RX interrupt

i2c_init();			       	// Initialize I2C


TA1CCTL1 = OUTMOD_7;              	// TA1CCR1 reset/set
TA1CCTL2 = OUTMOD_7;              	// TA1CCR1 reset/set

TA1CTL	= TASSEL_2 + MC_1;        	// SMCLK, upmode
TA1CCR0	= PWM_Period-1;           	// PWM Period
TA1CCR1	= 650;//PWM_Duty;         	// TA1CCR1 PWM Duty Cycle
TA1CCR2	= 650;//PWM_Duty;
P2DIR	|= BIT1 + BIT4;           	// P2.1 P2.4 = output
P2SEL	|= BIT1 + BIT4;           	// P2.1 P2.4 = TA1 output


stpm_init();


//hora = 6;
//minuto = 0;





while(1)                          	// While 1 is equal to 1 (forever)
{

	                              	//if(!((P1IN & BUTTON)==BUTTON)) // Was button pressed?
{	
					// Obtener hora del reloj

i2c_rx(DS3231_ADDR, rxdata, 7,ONEBYTEADDR,0x00,0x00);//i2c RX 7 bytes from DS3231 starting @ address 00:00

					//Imprimir datos del reloj en UART
		UART_OutUDec(bcdToDec(rxdata[2]));
		UART_char(':');
		UART_OutUDec(bcdToDec(rxdata[1]));
		UART_char(':');
		UART_OutUDec(bcdToDec(rxdata[0]));
		UART_char(' ');
		UART_OutUDec(bcdToDec(rxdata[4]));
		UART_char(' ');
		UART_OutUDec(bcdToDec(rxdata[5]));
		UART_char(' ');
		UART_OutUDec(bcdToDec(rxdata[6]));
		UART_char(' ');
		UART_OutUDec(bcdToDec(rxdata[3]));
		UART_char('\r');
		UART_char('\n');

//usdelay(65535);




//helios(8, 27, 18, 2, az, el);// modo demo 27 de agosto 18:2
helios(bcdToDec(rxdata[5]), bcdToDec(rxdata[4]), bcdToDec(rxdata[2]), bcdToDec(rxdata[1]), az, el);

/*

minuto+=1;
if(hora > 19)hora = 6;
if(minuto > 59)
{
	minuto = 0;
        hora++;
}

*/

		UART_OutUDec(hora);
		UART_char(':');
		UART_OutUDec(minuto);
		UART_char('\r');
		UART_char('\n');
//usdelay(2048);
usdelay(65535);
usdelay(65535);
usdelay(65535);
usdelay(65535);
usdelay(65535);
usdelay(65535);
usdelay(65535);
usdelay(65535);

    	//usdelay(65535);usdelay(65535);usdelay(65535);
	//usdelay(65535);usdelay(65535);usdelay(65535);
	//usdelay(65535);usdelay(65535);usdelay(65535);
	//usdelay(65535);usdelay(65535);usdelay(65535);

//__delay_cycles(1000000);__delay_cycles(1000000);


					// Establecer hora y fecha mediante UART
if(F == 1)
{
i2c_tx(DS3231_ADDR, txdataDS3231, 7,ONEBYTEADDR,0x00,0x00);//i2c TX 7 bytes "HELLO WORLD" starting @ address 00
i2c_tx(DS3231_ADDR, txdataDS3231, 7,ONEBYTEADDR,0x00,0x00);//i2c TX 7 bytes "HELLO WORLD" starting @ address 00
F = 0;
}

      }
  }

}


void helios(int Mth, int Day, int Hr, int Mn, int TAZ, int TEL)
{
float A, B, SL, DD, CL, DC, LD, tAZ, tEL, sAZ, sEL, mAZ, mEL, tX, tY, tZ, sX, sY, sZ, LT, LG;
int TZN;

	//Observer's latitude (degrees North):
	LT = 21.090368;
	LT = LT * PI/180;
	
	//Observer's longitude (degrees East)
	LG = -101.658543;
	LG = LG * PI/180;
	
	//tAZ = 30; TODO: comentar bien que este es el angulo azimutal del objetivo
	tAZ = TAZ * PI/180;

	//tEL = 40; TODO: comentar bien que este es el angulo de elevacion del objetivo
	tEL = TEL * PI/180;

	TZN = -6;

	CL = PI / 2 - LT;	                                // co-latitude
	DD = (int)(30.6 * ((Mth + 9) % 12) + 58.5 + Day) % 365;	// day of year (D = 0 on Jan 1)
	A = WH * DD + SN;	                                // orbit angle since solstice at mean speed 
	B = A + E2 * sin(A - SP);	                        // angle with correction for eccentricity 
	CC = (A - atan(tan(B) / CT)) / PI;
	SL = PI * (CC - floor(CC + .5));                        // solar longitude relative to mean position 
	CC = ST * cos(B);
	DC =  atan(CC / sqrt(1 - CC * CC));			// solar declination (latitude) 
	LD = (Hr - TZN + Mn / 60.0) * WR + SL + LG;		// longitude difference 

	P2C(LD, DC, &sX, &sY, &sZ);				// polar axis (perpend'r to azimuth plane) 
	C2P(sY, sZ, sX, &sAZ, &sEL);				// horizontal axis 
	P2C(sAZ + CL, sEL, &sY, &sZ, &sX);			// rotate by co-latitude 
	
	// Sun's position
	C2P(sX, sY, sZ, &sAZ, &sEL);				// vertical axis 

        UART_TX("\r\nH:"); 
        UART_OutUDec((long int)Hr); 

        UART_TX(" M:"); 
        UART_OutUDec((long int)Mn); 
	UART_TX("\r\n"); 

	


	UART_TX("\r\nSun Azimuth: ");
	if(sAZ < 0){
		sAZ*=-1;
		UART_char('-');
		UART_OutUDec((long int)(1000*rad2deg(sAZ)));
		sAZ*=-1;
	} 
	else UART_OutUDec((long int)(1000*rad2deg(sAZ))); 

	UART_TX("\r\nSun Elevation: "); 
 	if(sEL < 0){
                sEL*=-1;
                UART_char('-');
                UART_OutUDec((long int)(1000*rad2deg(sEL)));
		sEL*=-1;
        } 
        else UART_OutUDec((long int)(1000*rad2deg(sEL)));

	
	// Mirror aim direction
	P2C(tAZ, tEL, &tX, &tY, &tZ); 



	C2P(sX + tX, sY + tY, sZ + tZ, &mAZ, &mEL);



//mEL = 1.570796327; //valores de prueba EL 90 grados, AZ 90 grados
//mAZ = 1.570796327;

	UART_TX("\r\nMirror Azimuth: "); 
	UART_OutUDec((long int)(1000*rad2deg(mAZ))); // multiplica por 1000 para representar el numero flotante como entero

	UART_TX("\r\nMirror Elevation: "); 
        UART_OutUDec((long int)(1000*rad2deg(mEL))); // multiplica por 1000 para representar el numero flotante como entero

	UART_TX("\r\n"); 
	UART_TX("\r\nMirror Azimuthal screw length: "); 
        UART_OutUDec(1000 * rad2len(mAZ, M2CL, M2CC));
	//UART_TX("\r\n"); 

	UART_TX("\r\nMirror Elevation screw length: "); 
        UART_OutUDec(1000 * rad2len(mEL, M1CL, M1CC));
	//UART_TX("\r\n"); 



	if(F == 2){
		// TODO: establecer limites una vez funcional el codigo

		//if(mEL > 0 && mEL < 1.570)
		updateMotorEL(rad2len(deg2rad(agEL), M1CL, M1CC )); //revisa limites en radianes de el angulo de elevacion
		//if(mAZ > 0 && mAZ < 1.570)
		updateMotorAZ(rad2len(deg2rad(agAZ), M2CL, M2CC )); //revisa limites en radianes de el angulo de azimutal
		F = 0;
	} else
	{

		//if(mEL > 0 && mEL < 1.570)
		updateMotorEL(rad2len(mEL, M1CL, M1CC )); //revisa limites en radianes de el angulo de elevacion
		//if(mAZ > 0 && mAZ < 1.570)
		updateMotorAZ(rad2len(mAZ, M2CL, M2CC )); //revisa limites en radianes de el angulo de azimutal
	}
	
	// Go to sEl
	
	//pruebas con los servos

	//TA1CCR1 = (unsigned int)(SERVO_MIN + (180 - rad2deg(mAZ))*servo_stepval);
	//TA1CCR2 = (unsigned int)(SERVO_MIN + (180 - rad2deg(mEL))*servo_stepval);

        //usdelay(65535);//usdelay(65535);usdelay(65535);usdelay(65535);
	//usdelay(65535);usdelay(65535);usdelay(65535);usdelay(65535);
	
}


						//Inicializa posicion de motores
void stpm_init()
{
	P1DIR &=~BUTTON;                  	// Ensure button is input (sets a 0 in P1DIR register at location BIT3)

	P1OUT |=  BUTTON;                 	// Enables pullup resistor on button
	P1REN |=  BUTTON;

	P1DIR |= (LED_0 + mElDir + mElStp); // Set P1.0 to output direction
	P1OUT &= ~(LED_0 + mElDir + mElStp);// Set the LED off

	P2DIR |= (mAzDir + mAzStp); 		//Pone P2.0 Y P2.2 con direccion de salida (1): |=
	P2OUT &= ~(mAzDir + mAzStp); 		//Pone 0: &= ~(...)

	//STPM_OUT |= (LED_0);

	//recorrer hacia atras hasta activar el limit switch, en este caso sera simulado por BUTTON
	//STPM_OUT |= (mElDir);
	P1OUT &= ~(LED_0);
while(1) //entra en un ciclo infinito hasta que se presiona el boton
{
						//usdelay(65535);usdelay(65535);usdelay(65535);

	P1OUT |= (mElStp);			//digitalWrite(stepPin, HIGH);
	usdelay(7);				//delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
	P1OUT &= ~(mElStp);			//digitalWrite(stepPin, LOW);
    
	usdelay(220);
    
//	P1OUT ^= (LED_0);			//digitalWrite(ledPin, !digitalRead(ledPin));

        if(!((P1IN & BUTTON)==BUTTON))  	// Was button pressed?
	{
		P1OUT |= LED_0; l1 = D01; //0.0;	// TODO: que longitud debe tener l1?
		break;
	}

}

usdelay(64000);

UART_char('^');
P1OUT &= ~(LED_0);
while(1) 					//entra en un ciclo infinito hasta que se presiona el boton
{
						//usdelay(65535);usdelay(65535);usdelay(65535);

	P2OUT |= (mAzStp);			//digitalWrite(stepPin, HIGH);
	usdelay(7);				//delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
	P2OUT &= ~(mAzStp);			//digitalWrite(stepPin, LOW);
    
	usdelay(220);
    
//	P1OUT ^= (LED_0);			//digitalWrite(ledPin, !digitalRead(ledPin));

        if(!((P1IN & BUTTON)==BUTTON))  	// Was button pressed? TODO: debe usarse otro limit switch para el control azimutal
	{
		P1OUT |= LED_0; l2 = D02; //0.0;	// TODO: que longitud debe tener l2?
		break;
	}

}

UART_char('@');

//STPM_OUT |= (mElDir);

/*
int n;

for(n = 0; n < 10; n++)
{

	//usdelay(65535);usdelay(65535);usdelay(65535);


    STPM_OUT |= (mElStp);	//digitalWrite(stepPin, HIGH);
    usdelay(7);		//delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
    STPM_OUT &= ~(mElStp);	//digitalWrite(stepPin, LOW);
    
    usdelay(820);
    
    STPM_OUT ^= (LED_0);	//digitalWrite(ledPin, !digitalRead(ledPin));
  

}
*/
	
}


void updateMotorEL(float newL)
{
int i,n;
float d = newL - l1;

if(d < 0)
{
	UART_TX("\r\nDistancia de ajuste en el tornillo de elevacion = - "); 
	UART_OutUDec((int)(1000 * - d));
	//UART_TX("\r\n");
}
else
{ 
	UART_TX("\r\nDistancia de ajuste en el tornillo de elevacion =  "); 
	UART_OutUDec((int)(1000 * d));
	//UART_TX("\r\n");
}



if(d > 0.1)
{
	P1OUT &= ~(LED_0);
	n = (int)(d * 9.75 * 48);	//vueltas por unidad de longitud
	UART_TX("\r\nNumero de pasos del motor de elevacion: "); 
	UART_OutUDec(n);
	P1OUT |= (mElDir);		//change direction of rotation
	for(i = 0; i < n; i++)
	{
    		P1OUT |= (mElStp);	//mElStp en HIGH	
    		usdelay(7);		
    		P1OUT &= ~(mElStp);	//mElStp en LOW	
    		usdelay(220);   	//duracion del pulso () aproximadamente 25 milisegundos   
//    		P1OUT ^= (LED_0);	//toggle LED_0	
  

	}
	l1 = newL;
	P1OUT |= LED_0;

}

if(d < -0.1)
{
	P1OUT &= ~(LED_0);
	n = (int)(-d * 9.75 * 48);			    //vueltas por unidad de longitud
	UART_TX("\r\nNumero de pasos del motor de elevacion: "); 
	UART_TX("-");	
	UART_OutUDec(n);
	P1OUT &= ~(mElDir);		    //change direction of rotation
	for(i = 0; i < n; i++)
	{
    		P1OUT |= (mElStp);	//mElStp en HIGH
    		usdelay(7);		
    		P1OUT &= ~(mElStp);	//mElStp en LOW
    		usdelay(220);   	//duracion del pulso () aproximadamente 25 milisegundos
//    		P1OUT ^= (LED_0);	//toggle LED_0
  

	}
	l1 = newL;
	P1OUT |= LED_0;
}

//UART_TX("\r\n");

}


void updateMotorAZ(float newL)
{
int i,n;
float d = newL - l2;



if(d < 0)
{
	UART_TX("\r\nDistancia de ajuste en el tornillo azimutal = - "); 
	UART_OutUDec((int)(1000 * - d));
	//UART_TX("\r\n");
}
else
{ 
	UART_TX("\r\nDistancia de ajuste en el tornillo azimutal =  "); 
	UART_OutUDec((int)(1000 * d));
	//UART_TX("\r\n");
}
		


if(d > 0.1)
{
	P1OUT &= ~(LED_0);
	n = (int)(d * 9.75 * 48);	//n=pasos
	UART_TX("\r\nNumero de pasos del motor azimutal: "); 
	UART_OutUDec(n);
	P2OUT |= (mAzDir);		//change direction of rotation
	for(i = 0; i < n; i++)
	{
    		P2OUT |= (mAzStp);	//mElStp en HIGH	
    		usdelay(7);		
    		P2OUT &= ~(mAzStp);	//mElStp en LOW	
    		usdelay(220);   	//duracion del pulso () aproximadamente 25 milisegundos   
		//P2OUT ^= (LED_0);	//toggle LED_0	
  

	}
	l2 = newL;
	P1OUT |= LED_0;
}

if(d < -0.1)
{
	P1OUT &= ~(LED_0);
	n =(int)(-d * 9.74 * 48);	//vueltas por unidad de longitud
	UART_TX("\r\nNumero de pasos del motor azimutal: "); 
	UART_TX("-");
	UART_OutUDec(n);
	P2OUT &= ~(mAzDir);		//change direction of rotation
	for(i = 0; i < n; i++)
	{
    		P2OUT |= (mAzStp);	//mElStp en HIGH
    		usdelay(7);		
    		P2OUT &= ~(mAzStp);	//mElStp en LOW
    		usdelay(220);   	//duracion del pulso () aproximadamente 25 milisegundos
    		//P2OUT ^= (LED_0);	//toggle LED_0
  

	}
	l2 = newL;
	P1OUT |= LED_0;
}

//UART_TX("\r\n");

}




void i2c_init(void)
{
	P1SEL |= BIT6 + BIT7;						//Set I2C pins
	P1SEL2|= BIT6 + BIT7;
	UCB0CTL1 |= UCSWRST;						//Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		//I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;				//Use SMCLK, keep SW reset
	UCB0BR0 = 10;								//fSCL = SMCLK/11 = ~100kHz
	UCB0BR1 = 0;
	UCB0CTL1 &= ~UCSWRST;						//Clear SW reset, resume operation
	IE2 |= UCB0TXIE;							//Enable TX interrupt
	IE2 |= UCB0RXIE;							//Enable RX interrupt
	UCB0I2CIE |= UCNACKIE;						//Need to enable the status change interrupt
	__enable_interrupt();						//Enable global interrupt
}



/*
 * This functions writes to any I2C device. It takes as parameters:
 * 	- The slave address (7 bits)
 * 	- Pointer to the data buffer to transmit
 * 	- Size of the buffer
 * 	- Size of the address location to start writing at. Being 2 bytes for some of the EEPROMs and single byte
 * 	  for other devices. Please note that the ISR of this function assumes that the address bytes are written high
 * 	  byte first, then low byte second.
 * 	  In case of single byte address device, only the high byte will be used to set the address.
 * 	- The high byte and low byte address for the location to start writing at.
 *
 * 	In case the start condition of the write operation is not ack (for example EEPROM busy with a previous write cycle),
 * 	the corresponding interrupts detects this condition, generates a stop signal, and a Timer A1 (not Timer A0)
 * 	is activated for 0.5 ms, then the trial for writing is repeated.
 *
 * 	Please note that this function does not cater for the EEPROM 128byte paging requirements. So, if you are going
 * 	to write more than 128 bytes to an EEPROM, you will need to write a higher level function to segment the
 * 	writing into consecutive 128byte chunks.
 *
 */



void i2c_tx(unsigned char slave_addr, unsigned char *txdata, unsigned char bufSize, 
unsigned char addr_size, unsigned char high_byte_addr, unsigned char low_byte_addr)
{
i2c_packet.isr_mode=WRITE;
i2c_packet.data_buf=txdata;
i2c_packet.buf_size=bufSize;
i2c_packet.buf_index=0;
i2c_packet.addr_type=addr_size;
i2c_packet.addr_high_byte=high_byte_addr;
i2c_packet.addr_low_byte=low_byte_addr;
i2c_packet.addr_index=0;
UCB0I2CSA = slave_addr;						//Slave Address

while (1) 
{
	UCB0CTL1 |= UCTR + UCTXSTT;				// I2C TX, start condition
	LPM0;									// Enter LPM0
	if (i2c_packet.isr_mode == NOACK){		// If no ack received, then sleep for 0.5ms and try again
		i2c_packet.isr_mode = WRITE;
		i2c_packet.addr_index=0;			// Reset the address index for the next write operation
		usdelay(500);
	} 
	else 
	{
		break;								// Successful write, then quit
	}

}
}



/*
* This functions reads from any I2C device. It takes as parameters:
* 	- The slave address (7 bits)
* 	- Pointer to the data buffer to fill with data read.
* 	- Size of the buffer
* 	- Size of the address location to start writing at. Being 2 bytes for some of the EEPROMs and single byte
* 	  for other devices. Please note that the ISR of this function assumes that the address bytes are written high
* 	  byte first, then low byte second.
* 	  In case of single byte address device, only the high byte will be used to set the address.
* 	- The high byte and low byte address for the location to start reading at.
*
*	The function starts with a write operation to specify the address at which the read operation with start
* 	In case the start condition of the write operation is not ack (for example EEPROM busy with a a previous write cycle),
* 	the corresponding interrupts detects this condition, generates a stop signal, and a Timer A1 (not Timer A0)
* 	is activated for 0.5 ms, then the trial for writing is repeated.
*
* 	Once the write address is successful, the functions switch to read mode, and fills the buffer provided
*
*/

void i2c_rx(unsigned char slave_addr, unsigned char *rxdata, unsigned char bufSize, unsigned char addr_size,
unsigned char high_byte_addr, unsigned char low_byte_addr)
{
i2c_packet.isr_mode=READ;					// The ISR will send the address bytes, then wake CPU.
i2c_packet.addr_type=addr_size;
i2c_packet.addr_high_byte=high_byte_addr;
i2c_packet.addr_low_byte=low_byte_addr;
i2c_packet.addr_index=0;
UCB0I2CSA = slave_addr;						// Slave Address

while (1)
{
	UCB0CTL1 |= UCTR + UCTXSTT;				// I2C TX, start condition
	LPM0;        							// Enter LPM0

	if (i2c_packet.isr_mode == NOACK)
	{										// If no ack received, then sleep for 0.5ms and try again
		i2c_packet.isr_mode = READ;
		i2c_packet.addr_index=0;			// Reset the address index for the next write operation
		usdelay(500);
	}
	else
	{
		break;								// Successful write, then quit
	}

}

											// We wrote already the address, so now read only data.
i2c_packet.addr_index=i2c_packet.addr_type;
i2c_packet.data_buf=rxdata;
i2c_packet.buf_size=bufSize;
i2c_packet.buf_index=0;
UCB0CTL1 &= ~UCTR;                      	// I2C RX
UCB0CTL1 |= UCTXSTT;                    	// I2C re-start condition
LPM0;										// Enter LPM0

       	   	   	   	   	   					// and remain until all data is received
}



											//interrupt(USCIAB0RX_VECTOR) state change to trap the no_Ack from slave case

#pragma vector = USCIAB0RX_VECTOR

__interrupt void USCIAB0RX_ISR(void)
{
unsigned char c;
c = UCA0RXBUF;

if(flag_00 == 1){
	UART_char(c);
	buffer[buffer_index++] = c;
	if(buffer_index == 7){
		buffer[7] = 0;
		UART_char('@');

		az = (buffer[1] - '0')*100 + (buffer[2] - '0')*10 + (buffer[3] - '0');
		el = (buffer[4] - '0')*100 + (buffer[5] - '0')*10 + (buffer[6] - '0');
		UART_TX(buffer);
		
		flag_00 = 0;
	}

}
else if(flag_00 == 2){
	UART_char(c);
	buffer[buffer_index++] = c;
	if(buffer_index == 14){
		buffer[14] = 0;


		UART_char('#');

// DS3231 seconds, minutes, hours, day, date, month, year
//example: C00041902150517
unsigned char sec = (buffer[0] - '0')*10 + (buffer[1] - '0');
unsigned char min = (buffer[2] - '0')*10 + (buffer[3] - '0');
unsigned char hou = (buffer[4] - '0')*10 + (buffer[5] - '0');
unsigned char day = (buffer[6] - '0')*10 + (buffer[7] - '0');
unsigned char dat = (buffer[8] - '0')*10 + (buffer[9] - '0');
unsigned char mon = (buffer[10] - '0')*10 + (buffer[11] - '0');
unsigned char yea = (buffer[12] - '0')*10 + (buffer[13] - '0');

txdataDS3231[0] = decToBcd(sec);
txdataDS3231[1] = decToBcd(min);
txdataDS3231[2] = decToBcd(hou);
txdataDS3231[3] = decToBcd(day);
txdataDS3231[4] = decToBcd(dat);
txdataDS3231[5] = decToBcd(mon);
txdataDS3231[6] = decToBcd(yea);

		UART_TX(buffer);

	
		F = 1; 			//flag for set the clock
		flag_00 = 0; 	//reset flag for command
	}

}

else if(flag_00 == 3)
{
	UART_char(c);
	buffer[buffer_index++] = c;
	if(buffer_index == 4){
		buffer[4] = 0;


		UART_char('&');
		//establece angulos del heliostato
		//example: S4560
		agEL = (buffer[0] - '0')*10 + (buffer[1] - '0');
		agAZ = (buffer[2] - '0')*10 + (buffer[3] - '0');
		UART_TX(buffer);

        	F = 2;		//lfag to set heliostat angles
		flag_00 = 0; 	//reset flag for command
	}
}

if (c == 'T') 			// 'T' received?
{

	flag_00 = 1;
	buffer_index = 0;
        UART_TX("\r\nHola Mundo!\r\n");
	UART_char(c);
	//UART_char('\r');
	//UART_char('\n');
	//UART_TX(buffer);
}

if (c == 'C') // 'C' received?
{

	flag_00 = 2;
	buffer_index = 0;
        UART_TX("\r\nEstableciendo hora y fecha en el reloj...\r\n");
	UART_char(c);
	//UART_char('\r');
	//UART_char('\n');
	//UART_TX(buffer);
}

if (c == 'S') // 'S' received?
{

	flag_00 = 3;
	buffer_index = 0;
        UART_TX("\r\nEstableciendo angulos de elevacion y azimutal...\r\n");
	UART_char(c);
	//UART_char('\r');
	//UART_char('\n');
	//UART_TX(buffer);
}



if(UCNACKIFG & UCB0STAT)
{
	UCB0STAT &= ~UCNACKIFG;			// Clear flag so that not to come here again
	i2c_packet.isr_mode=NOACK;		// The main function needs to act based on noack
	UCB0CTL1 |= UCTXSTP;               	// I2C stop condition


	LPM0_EXIT;				// Exit LPM0
}

}


/*
 * This interrupt is called each time the UCSI_B module is either ready to get a new byte in UCB0TXBUF to send to the I2C device, or
 * a new byte is read into UCB0RXBUF and we should pick it up.
 * The interrupt is called as both UCB0TXIE and UCB0RXIE are enabled. To stop this interrupt being called indefinitely, the corresponding
 * interrupt flag should be cleared.
 * These flags are automatically clearly by the USCI_B module if the UCB0XXBUF is access. However, if we are to do something different than reading
 * or writing a byte to/from the UCB0XXBUF, we need to clear the corresponding flag by ourselves or the ISR will be called for ever,
 * and the whole program will hang.
 */

//interrupt(USCIAB0TX_VECTOR) USCIAB0TX_ISR(void)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
// Transmit address bytes irrespective of send or receive mode.

if (i2c_packet.addr_index==0){
	UCB0TXBUF = i2c_packet.addr_high_byte;
	i2c_packet.addr_index++;
}
else if (i2c_packet.addr_index==1 && i2c_packet.addr_type==TWOBYTEADDR)
{
	UCB0TXBUF = i2c_packet.addr_low_byte;
	i2c_packet.addr_index++;
}
else if(UCB0TXIFG & IFG2 && i2c_packet.isr_mode==READ)
{
// USCI_B is ready to get a new data byte to transmit it, and we are in READ mode.
// So, we should not continue writing, but should exit to the calling function to
// switch the USCI_B into read mode
	IFG2 &= ~UCB0TXIFG;			// Clear USCI_B0 TX int flag manually as we did not write to the UCB0TXBUF
	 LPM0_EXIT;							// Exit LPM0
}
else if(UCB0TXIFG & IFG2 && i2c_packet.isr_mode==WRITE)
{
// USCI_B is ready to get a new data byte to transmit it, and we are in write mode.
	if(i2c_packet.buf_index == i2c_packet.buf_size)
	{					// If no more data to transmit, then issue stop condition and wake CPU.
		IFG2 &= ~UCB0TXIFG;		// Clear USCI_B0 TX int flag manually as we did not write to the UCB0TXBUF
		UCB0CTL1 |= UCTXSTP;		// I2C stop condition
		LPM0_EXIT;			// Exit LPM0
	}
	else
	{
		UCB0TXBUF = i2c_packet.data_buf[i2c_packet.buf_index];
		i2c_packet.buf_index++;		// Increment TX byte counter
	}

}
else if (UCB0RXIFG & IFG2 && i2c_packet.addr_index==i2c_packet.addr_type)
{

						// Read mode, and we already completed writing the address

	i2c_packet.data_buf[i2c_packet.buf_index]= UCB0RXBUF;
	i2c_packet.buf_index++;			// Increment RX byte counter

     	if(i2c_packet.buf_index == i2c_packet.buf_size)
	{					// If last byte to receive, then issue stop condition and wake CPU.
		IFG2 &= ~UCB0RXIFG;		// Clear USCI_B0 RX int flag
		UCB0CTL1 |= UCTXSTP;		// I2C stop condition here to avoid reading any extra bytes
		LPM0_EXIT;			// Exit LPM0

     	}

}
}


unsigned char bcdToDec(unsigned char val)
{
  return( (val/16*10) + (val%16) );
}

unsigned char decToBcd(unsigned char val)
{
   return ((val / 10 * 16) + (val % 10));
}


float rad2len(float rads, float a, float b)
{  
   return sqrt(a * a + b * b - 2 * a * b * cos(rads));
}

float ang(float x, float y)
{
	double ang;
	if (x > 0)
	  ang = atan(y/x);
	else if (x < 0)
	  ang = atan(y/x) + PI;
	else if (y > 0)
	  ang = 1 * PI / 2;
	else if (y < 0)
	  ang = -1 * PI / 2;
	else
	  ang = 0;

	return ang;
}

void P2C(float AZ, float EL, float *x, float *y, float *z)
{
	float c;
	*z = sin(EL);
	c = cos(EL);
	*x = c * sin(AZ);
	*y = c * cos(AZ);
}

void C2P(float x, float y, float z, float *AZ, float *EL)
{
	*EL = ang(sqrt(x * x + y * y), z);
	*AZ = ang(y, x);
	if (*AZ < 0)
	  *AZ = *AZ + 2 * PI;
}



//-----------------------UART_OutUDec-----------------------

// Output a 32-bit number in unsigned decimal format

// Input: 32-bit number to be transferred

// Output: none

// Variable format 1-10 digits with no space before or after

void UART_OutUDec(unsigned long n){

// This function uses recursion to convert decimal number

//   of unspecified length as an ASCII string

  if(n >= 10){

    UART_OutUDec(n/10);

    n = n%10;

  }

  UART_char(n+'0'); /* n is between 0 and 9 */

}


void UART_char(int  tx_data) // Define a function which accepts a character pointer to an array
{
    
     while ((UCA0STAT & UCBUSY)); // Wait if line TX/RX module is busy with data
     UCA0TXBUF = tx_data; // Send out element i of tx_data array on UART bus
    
}


void UART_TX(char * tx_data) // Define a function which accepts a character pointer to an array
{
    unsigned int i=0;
    while(tx_data[i]) // Increment through array, look for null pointer (0) at end of string
    {
        while ((UCA0STAT & UCBUSY)); // Wait if line TX/RX module is busy with data
        UCA0TXBUF = tx_data[i]; // Send out element i of tx_data array on UART bus
        i++; // Increment variable for array address
    }
}

//-----------------------UART_ConvertDistance-----------------------
// Converts a 32-bit distance into an ASCII string
// Input: 32-bit number to be converted (resolution 0.001cm)
// Output: store the conversion in global variable String[10]
// Fixed format 1 digit, point, 3 digits, space, units, null termination
// Examples
//    4 to "0.004 cm"  
//   31 to "0.031 cm" 
//  102 to "0.102 cm" 
// 2210 to "2.210 cm"
//10000 to "*.*** cm"  any value larger than 9999 converted to "*.*** cm"
void UART_ConvertDistance(unsigned long n){
// as part of Lab 11 implement this function
  if(n<=100000) {
    String[0] = n/10000 +0x30;
    String[1] = '.';
    String[2] = (n/1000)%10 +0x30;
    String[3] = (n/100)%10 +0x30;
    String[4] = (n/10)%10 +0x30;
    String[5] = n%10 +0x30;
    
  } else {
    String[0] = '*';
    String[1] = '.';
    String[2] = '*';
    String[3] = '*';
    String[4] = '*';
    String[5] = '*';
  }
  //String[5] = ' ';
  String[6] = 0;
}

//-----------------------UART_OutDistance-----------------------
// Output a 32-bit number in unsigned decimal fixed-point format
// Input: 32-bit number to be transferred (resolution 0.001cm)
// Output: none
// Fixed format 1 digit, point, 3 digits, space, units, null termination
void UART_OutDistance(unsigned long n){
  UART_ConvertDistance(n);      // convert using your function
  UART_TX(String);       // output using your function
}



float rad2deg(float radians)
{
    return (180.0/PI)*radians;
}

float deg2rad(float degrees)
{
    return (PI/180.0)*degrees;
}


float tan(float x)
{
        return sin(x)/cos(x);
}
/* ================================================================== */
/* sqrt(x)                                                             */
/* ================================================================== */
float sqrt(float number)  
{
float lower, upper, guess;

 if (number < 1)
 {
  lower = number;
  upper = 1;
 }
 else
 {
  lower = 1;
  upper = number;
 }

 while ((upper-lower) > 0.001)
 {
  guess = (lower + upper)/2;
  if(guess*guess > number)
   upper =guess;
  else
   lower = guess; 
 }
 return (lower + upper)/2;

}  

/* ================================================================== */
/* sin(x)                                                             */
/* ================================================================== */
float sin(float value)
{  
  float sign = 1.;
  float x = value;
  float y = 0.;  
  if(x < 0) 
  {
        sign = (-1.);
        x *= (-1.);
  }

  if(x > 3.14159265359)
  {
        x = x - 3.14159265359;
        sign = -sign;
  }

  if(x > 1.57079632679)
        x = 3.14159265359 - x;

  y = x*x;
  x = sign*x*(((((-.0000000239*y + .0000027526)*y - .0001984090)*y + .0083333315)*y - .1666666664)*y + 1);
return x;
}

/* ================================================================== */
/* cos(x)                                                             */
/* ================================================================== */
float cos(float value)
{  
  float sign=1.;
  float x=value;
  float y=0.;  
  if(x<0) 
  {
        x*=(-1.);
  }
  if(x > 3.14159265359)
  {
        x = x - 3.14159265359;
        sign = -sign;
  }

  if(x > 1.57079632679)
  {
        x = 3.14159265359 - x;
        sign = -sign;
  }

  y = x*x;
  x = sign*(((((-.0000002605*y + .0000247609)*y - .0013888397)*y + .0416666418)*y - .4999999963)*y + 1);
return x;
}


float atan(float value)
{
  float sign=1.;
  float x=value;
  float y=0.;
  if(value==0.)
	return 0;
  if(x<0)
	x*=(-1.);

  x=(x-1.)/(x+1.);
  y=x*x;

  x = ((((((((.0028662257 *y - .0161657367)*y + .0429096138)*y - .0752896400)*y + .1065626393)*y - .1420889944)*y + .1999355085)*y - .3333314528)*y + 1)*x;

  x= .785398163397 +x;

  if ( value < 0 )
	return x = x * -1;
  else
	return x;
}

//------------------------------------------------------------------------------
// micro seconds delays
//
void usdelay(unsigned int interval){
    								// Setup TimerA
	TA0CCTL0 = CCIE;           	// interrupt enabled
 	TA0CCR0 = TA0R + interval;  // micro secs @ 1Mhz Clock
    	TA0CTL = TASSEL_1 + MC_2;  	// SMCLK, continuous mode.
    	LPM0;						// suspend CPU
}

// Timer A1 interrupt service routine. TIMERx_Ay_VECTOR.(x being the index of the timer, y of the vector for this timer)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
	TA0CTL = 0;						// Stop Timer_A0
    	LPM0_EXIT;						// Return active
}

int floor(float x) {
    int xi = (int)x;
    return x < xi ? xi - 1 : xi;
}




