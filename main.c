/* --------0.Project information--------------------
 * SRF05 Module Distance Measurement
 * Author : TRAN MINH THUAN
 * Date: May 9th 2019
 * Project associate with TM4C123, CCS version 9.
---------------------------------------------------*/

/* --------1.System requirement---------------------
Using mode 1 of SRF05 to get the distance of obstacles (Mode 1)
FA3     - Trigger pin           - Output
FA2     - Echo pin              - Input
Timer2  - Pulse length          -
---------------------------------------------------*/


////// ----------------2. Pre-processor Directives Section--------------------///////
#include <UserLibraries/Userlibs.h>
#include "UserLibraries/Profile.h"

#define TIMER_VALUE     (*((volatile unsigned long *)0x40032050))
#define Trigger_PIN     GPIO_PIN_5
#define Trigger_PORT    GPIO_PORTA_BASE

#define Echo_PIN        GPIO_PIN_3
#define Echo_PORT       GPIO_PORTB_BASE
#define ECho_Flag       GPIO_INT_PIN_3
//////------------------------------------------------------------------------///////

////// ----------------3.Global Declarations Section--------------------------///////
unsigned char Echo_wait=0;
unsigned int  Pulse_Width=0;
unsigned int  test_var=0;
unsigned char Done=1;
const double  temp = 1.0/80.0;

/* -----------SRF05_Init----------------
 * GPIO Init for SRF05 (Trigger pin and Echo pin)
         * Trigger pin - output
         * Echo pin - input
 * Timer Init for SRF05 (To measure the pulse width of Echo pin)
         * Timer 32-bit full width, periodic up
 * Input: No
 * Output: No
 */
void SRF05_Init(void);

/* -----------SRF05_Handler----------------
 * Echo pin interrupt both edges handler
 * If detect rising edge, then reset timer value and start timer
 * If detect falling edge, then get timer value (Pulse_Width) and disable timer
 * Input: No
 * Output: No
 * Affect global variable:
     * Echo_wait - is true if SRF05 received respond signal
     * Pulse_Width - The duration between two edges
 */
void SRF05_Handler(void);
//////------------------------------------------------------------------------///////

////// ----------------4. Subroutines Section---------------------------------///////
void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
    Profile_Init();
    SRF05_Init();                                                                           //initialize GPIO for SRF05
    GPIOPinWrite(Trigger_PORT, Trigger_PIN, 0);                                             //LOW is initial state
    IntMasterEnable();
    while(1)
    {
        if(Echo_wait != 1)
        {
            Profile_Toggle(PROFILE_PIN2);
            Profile_Toggle(PROFILE_PIN2);

            GPIOPinWrite(Trigger_PORT, Trigger_PIN, Trigger_PIN);
            Profile_Toggle(PROFILE_PIN3);
            SysCtlDelay(266);                   //9.975us
            GPIOPinWrite(Trigger_PORT, Trigger_PIN, 0);
            Profile_Toggle(PROFILE_PIN3);

            while(Echo_wait != 0);
            //
            // Way 1
            //
            if(Pulse_Width<1000000)
            {
                test_var=Pulse_Width;
                test_var =(uint32_t)(temp * test_var);
                test_var = test_var / 58;
            }
            //
            // Way 2
            //

            Profile_Toggle(PROFILE_PIN2);
        }
        SysCtlDelay(SysCtlClockGet()/300);                   //9.975us
    }
}

void SRF05_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlDelay(3);
    //Trigger pin - output
    GPIOPinTypeGPIOOutput(Trigger_PORT, Trigger_PIN);
    //Echo pin - input interrupt both edges
    GPIOPinTypeGPIOInput(Echo_PORT, Echo_PIN);
    GPIOIntTypeSet(Echo_PORT, Echo_PIN,GPIO_BOTH_EDGES);
    GPIOIntRegister(Echo_PORT,SRF05_Handler);
    GPIOIntEnable(Echo_PORT, ECho_Flag);
    //Timer periodic up full width (32-bit)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlDelay(3);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE,TIMER_A);
}

void SRF05_Handler(void)
{
	static int  valid=0;
    GPIOIntClear(Echo_PORT, ECho_Flag);
    if(GPIOPinRead(Echo_PORT, Echo_PIN)== Echo_PIN) //Rising edge
    {
        TIMER_VALUE=0;                              //Reset timer
        TimerEnable(TIMER2_BASE, TIMER_A);
        Echo_wait=1;
		valid=1;
    }
    if  ( (GPIOPinRead(Echo_PORT, Echo_PIN) == 0) && (valid==1) )       //Falling edge
    {
        Pulse_Width=TIMER_VALUE;
        TimerDisable(TIMER2_BASE, TIMER_A);
        Echo_wait=0;
		valid=0;
    }
}
////////------------------------------------------------------------------------///////
