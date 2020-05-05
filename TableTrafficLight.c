// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Alexander Drath
// March 25, 2020

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

#define SENSOR							(*((volatile unsigned long *)0x4002401C))		//Access PE0-PE2
#define STREET_LIGHT				(*((volatile unsigned long *)0x400050FC))		//Access PB0-PB5
#define WALK_LIGHT					(*((volatile unsigned long *)0x40025028))		//Access PF1&PF3

#define goW					0		//West green		South red			walk red
#define waitW				1		//West yellow		South red			walk red
#define goS					2		//West red			South green		walk red
#define waitS				3		//West red			South yellow	walk red
#define walk				4		//West red			South red			walk green
#define wRed1				5		//West red			South red			walk red
#define wOff2				6		//West red			South red			walk off
#define	wRed3				7		//West red			South	red			walk red
#define wOff4				8		//West red			South red			walk red

// ***** 2. Global Declarations Section *****
struct State {
	unsigned long Traffic;
	unsigned long Crosswalk;
	unsigned long Time;
	unsigned long Next[8];
};
typedef const struct State STyp;

STyp FSM[9]={
	{0x0C,0x02,500,{goW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},
	{0x14,0x02,100,{goS,goS,goS,goS,walk,walk,walk,walk}},
	{0x21,0x02,500,{goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},
	{0x22,0x02,100,{goW,goW,goW,goW,walk,goW,walk,goW}},
	{0x24,0x08,200,{walk,wRed1,wRed1,wRed1,walk,wRed1,wRed1,wRed1}},
	{0x24,0x02, 50,{wOff2,wOff2,wOff2,wOff2,wOff2,wOff2,wOff2,wOff2}},
	{0x24,0x00, 50,{wRed3,wRed3,wRed3,wRed3,wRed3,wRed3,wRed3,wRed3}},
	{0x24,0x02, 50,{wOff4,wOff4,wOff4,wOff4,wOff4,wOff4,wOff4,wOff4}},
	{0x24,0x00, 50,{goS,goW,goS,goS,goS,goW,goS,goS}}
};

unsigned long S;			//index to the current state
unsigned long Input;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
void PLL_Init(void);
void PortBEF_Init(void);
void SysTick_Init(void);
void SysTick_Wait(unsigned long);
void SysTick_Wait10ms(unsigned long);

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	PLL_Init();
	SysTick_Init();
	PortBEF_Init();
  EnableInterrupts();
	S = goS;
  while(1){
		STREET_LIGHT = FSM[S].Traffic;			//Set street lights
		WALK_LIGHT = FSM[S].Crosswalk;	//Set crosswalk
		SysTick_Wait10ms(FSM[S].Time);	//Wait the correct time
		Input = SENSOR;									//read sensors
		S = FSM[S].Next[Input];					//Set the index value to the current input
  }
}

void PortBEF_Init(){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32;							// 1) B E F
	delay = SYSCTL_RCGC2_R;							// 2) no need to unlock
	//B
	GPIO_PORTB_AMSEL_R &= ~0x3F; 				// 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; 	// 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    				// 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; 				// 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    				// 7) enable digital on PB5-0
	//E
	GPIO_PORTE_AMSEL_R &= ~0x07;				// 3) diable analog function for PE2-0
	GPIO_PORTE_PCTL_R	&= ~0x00000FFF;		// 4) enable regular GPIO
	GPIO_PORTE_DIR_R &= ~0x07;					// 5) inputs on PE2-0
	GPIO_PORTE_AFSEL_R &= ~0x07;				// 6) regular function on PE2-0
	GPIO_PORTE_DEN_R |= 0x07;						// 7) enable function on PE2-0
	//F
	GPIO_PORTF_AMSEL_R &= ~0x0A; 				// 3) disable analog function on PF1&PF3
  GPIO_PORTF_PCTL_R &= ~0x000F0F0; 		// 4) enable regular GPIO
  GPIO_PORTF_DIR_R |= 0x0A;    				// 5) outputs on PF1&PF3
  GPIO_PORTF_AFSEL_R &= ~0x0A; 				// 6) regular function on PF1&PF3
  GPIO_PORTF_DEN_R |= 0x0A;    				// 7) enable digital on PF1&PF3
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}
