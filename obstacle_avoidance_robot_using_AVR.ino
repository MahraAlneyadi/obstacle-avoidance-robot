
/*aruino autonomous car with aoiding obstacles project 
by Mahra Alneyadi


#define F_CPU 1600000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


/*--------------------------------------------initiating variables-------------------------------------------------------------*/
long timep, distance, right,front,left;
double dcyc =0;
int Ain;
float dout, dd;
/*--------------------------------------ultrasonic distance function-------------------------------------------------------*/
 

 void distancevalue(uint8_t trigPin,uint8_t echoPin) /**ultrasonic sensor distance measurement**/
{
DDRD=0x97; /**assignning data direction register portd to be 1001 0111**/

PORTD &= ~(trigPin); /**initially set trigpin to be low for 2 micro s**/
_delay_us(2);

PORTD |= (trigPin); /** set trigpin to be high for 10 micro s**/
_delay_us(10);

PORTD &= ~(trigPin);/**set trig pin again to low**/

timep = pulseIn(echoPin, HIGH); /**we used pulse in function to calculate the time through the pulse that is recieved from echo pin- high**/
distance = timep*0.034/2;  /**formula to measure the distance**/
 
}
/*----------------------------------------------ISR for PWM- Buzzer---------------------------------------------------------*/

ISR (TIMER0_ovf_vect){
OCR0A = (dcyc/100)*255; /*to compare the value with the time again*/
}

/*-----------------------------------------------main----------------------------------------------------------*/

int main(void)
{
   DDRD=0x97; /*1001 0111*/ 
   DDRB=0xFF; /*1111 1111*/ 
   DDRC=0xF0; /*1111 0000*/ 
   
   uint8_t x1= PIND3;/**echo for trig1- PWM pin*/
   uint8_t x2= PIND5;/**echo for trig2- PWM pin*/
   uint8_t x3= PIND6;/**echo for trig3- PWM pin*/
  
   /*------------------------------------ PWM for buzzer----------------------------------------------*/

  TCCR0A = (1<<COM0A1) |(1<<WGM00) |(1<<WGM01);/*fast PWM */
  TIMSK0 = (1<<TOIE0);/*timer interrupt overflow enable when it is high*/
  OCR0A = (dcyc/100)*255; /*to compare the value with the time*/
  TCCR0A= (1<<CS01);/*clk/8 prescaler*/
  sei(); /*initiate global interrupt*/
 
  
   /*------------------------------------ADC for IR sensor----------------------------------------------*/
   
  ADCSRA &= ~(1<<ADIE); /*disable interrupt*/
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1); /*prescalar 64 */
  ADCSRA |= (1<<ADEN); /* enable AD */
  ADMUX |=(1<<MUX0)|(1<<REFS0); 
 
  PORTC&=~1<<PORTC4; /**red led initially low*/
  PORTC&=~1<<PORTC5; /**green led initially low*/
  beeb(false);/**initially low*/
  moveforward(); /**initial direction*/

/*-----------------------------------------------loop----------------------------------------------------------*/

/**start loop*/
while(1) {
    
/**ultrasonic sensor reading*/
 
distancevalue(0x04, x1); /**portb 2**/
front= distance; 

distancevalue(0x10, x2); /**portb 4**/
left = distance;

distancevalue(0x80, x3); /**portb 7**/
right = distance;

 PORTC^=1<<PORTC5; 
 
  moveforward(); /**move forward as default**/
  
/*-----------------------------------------------ultrasonic start comparing----------------------------------------------------------*/

  if(front<=20){ /**if front yltrasonic is less than or equal 20cm**/
  moveback();   /**move back for moment**/
  _delay_ms(100);
  PORTC&=~1<<PORTC5; /*green led will turn off*/
  beeb(true); /**obstacle alarm are high**/
  /**----------------comparing between distance of left and right to decide which direction to be taken----------------**/
  if (left>right) { /*if left distance bigger than right it will turn left*/
    moveleft();
    _delay_ms(100);
    PORTC&=~1<<PORTC5; /*green led will turn off*/
    beeb(true);} /*buzzer is on*/
  else{
    moveright();/*else it will turn right*/
    _delay_ms(100);
    PORTC&=~1<<PORTC5; /*green led will turn off*/
    beeb(true);/*buzzer is on*/
  }
  }
  /**----------------if right ultrasonic distance is less than or equal 20 it will turn left with obstacle alarm on----------------**/
  else if(right<=20){ 
   moveleft();
   PORTC&=~1<<PORTC5; 
  beeb(true);
  }
  /**----------------if left ultrasonic distance is less than or equal 20 it will turn right with obstacle alarm on----------------**/
  else if (left<=20){
      moveright();
      PORTC&=~1<<PORTC5; 
    beeb(true);
    }
 /*-----------------------------------------------IR sensor detecting black spot----------------------------------------------------------*/
   if(readIR(0x41)==HIGH){ /*if IR is in high mode means the black light is detcted and absorbed*/
    stopmotor(); 
    PORTC=1<<PORTC4; /*red led*/
    }
    
  beeb(false); /**obstacle alarm is off in default**/
 }
}

/*-------------------------------------------Buzzer function --------------------------------------------------------------*/

void beeb (boolean buzz){
       
        if(buzz==true){
          DDRB =1<<PORTB2; /*buzzer is on for 50 ms*/
          _delay_ms(50);  
           dcyc +=20;/*duty cycle +20*/
          if(dcyc>=100){  /*duty cycle will reset when reaches to 100*/
            dcyc=0;}}    /* Wait for some time*/
        else{
          DDRB &=~ 1<<PORTB2;
          }
}

/*-----------------------------------------Directions functions----------------------------------------------------------------*/


void moveright(){/**move right **/
PORTB |=0x10;
PORTB &=~0x08;
PORTB &=~0x02; 
PORTB &=~ 0x01;
}
void moveforward(){/**move forward **/
PORTB |=0x10;
PORTB &=~0x08;
PORTB |= 0x02;
PORTB &=~0x01;
 }

void moveback(){/**move back **/
PORTB &=~0x10;
PORTB |=0x08;
PORTB &=~0x02;
PORTB |=0x01; 
 }

void moveleft(){/**move left **/
PORTB |= 0x02;
PORTB &=~0x10;
PORTB &=~ 0x01;
PORTB &=~ 0x08;
}

void stopmotor(){ /** stop motor **/
  PORTB &=~ 0x10; 
  PORTB &=~ 0x08;
  PORTB &=~ 0x02;
  PORTB &=~ 0x01; 
  PORTC&=~1<<PORTC5; /**initially low*/
}


/*--------------------------------------------IR SENSOR DIGITAL READ FROM ANALOG PIN-------------------------------------------------------------*/

uint8_t readIR (uint8_t x){ 
  /*method to read ADC from multiple channels*/
  ADCSRA |= (1<<ADEN); /* enable AD*/
  ADMUX =x&0x47; /* input of the main that include REFS0 and MUXn*/
   ADCSRA |= (1<<ADSC); /*start coversion*/
  while (ADCSRA & (1<<ADSC)){}  /*conversion process done*/

    Ain=ADC;
    dd=Ain*1*5/1024; /*ADC conversion formula*/
    dout=dd/dd; /*the logic here is if Ain is a number>0  we divide the Ain over itself we get 1=high else if the number is zero then low */
    if(dout>=1){
      return HIGH;}/*it will return high when the value is one dd/dd*/
    else{
      return LOW;} /*it will return low when the value is 0*/
}
