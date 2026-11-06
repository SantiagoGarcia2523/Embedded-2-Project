#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "uart0.h"

#define M1PWM5_MASK 0x02
#define PWM_LOAD 2500  // 500hz

#define IN1 0x04  //pd2
#define IN2 0x08  //pd3


#define TRIG 0x02  // PE1
#define ECHO 0x04  // PE2

//change PWM approximately every 30 ms, measure sonar every two steps.
#define PWM_STEP_US 30000U // do 30ms delay
#define HOLD_TIME_US 1000000U

#define RISE_THRESHOLD_MM 10.0f
#define FALL_THRESHOLD_MM 10.0f

#define ECHO_TIMEOUT_TICKS 1200000U


#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))



// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16
#define PUSH_BUTTON2_MASK 0x01


volatile uint8_t dutyCycle;
volatile uint32_t echo_duration = 0;
volatile uint32_t start = 0;
volatile uint32_t end = 0;
volatile uint32_t done = 0;
volatile uint32_t echo_state = 0;

void initHw(void);
void printUint32(uint32_t number);
void pwm_init(void);
void printDuty(void);
void setPWMduty(uint8_t duty);
void init_WTIMER(void);
void init_gpio(void);
void init_echo_interrupt(void);
void GPIOPortE_Handler(void);
uint8_t readDistanceMm(float *distance);
void printDistanceMm(float distance);
int main(void)
{
    initHw();
    init_gpio();
    initUart0();
    setUart0BaudRate(115200, 40000000);
    init_WTIMER();
    init_echo_interrupt();
    pwm_init();

    float bottomDistance = 0.0f;
    float topDistance = 0.0f;
    float distance = 0.0f;
    float riseDistance = 0.0f;
    float fallDistance = 0.0f;
    int riseDuty = -1;
    int fallDuty = -1;
    uint8_t invalidReadings = 0;

    //fan stopped. Wait 5 seconds, then record bottom position.
    dutyCycle = 0;
    setPWMduty(dutyCycle);
    putsUart0("Fan stopped. Waiting 5 seconds...\r\n");
    waitMicrosecond(HOLD_TIME_US);

    if (!readDistanceMm(&bottomDistance))
    {
        putsUart0("Cannot read bottom position. Stopping.\r\n");
        setPWMduty(0);
        return 1;
    }

    putsUart0("Bottom: 0% at ");
    printDistanceMm(bottomDistance);
    putsUart0("Increasing PWM...\r\n");

    //LOOP 1: Increase PWM until the plunger starts rising
       for (int duty = 50; duty <= 100; duty++)
       {
           dutyCycle = (uint8_t)duty;
           setPWMduty(dutyCycle);

           waitMicrosecond(PWM_STEP_US); // 50 ms

           //hold for 5 seconds while gradually increasing through 50%-100% duty cycle.
           if (duty >= 50 && duty <= 100 && duty % 5 == 0)
           {
               putsUart0("Holding PWM at ");
               printUint32(duty);
               putsUart0("%\r\n");

               uint32_t holdStart = WTIMER0_TAV_R;

               while ((uint32_t)(WTIMER0_TAV_R - holdStart) < 200000000U)
               {
                   waitMicrosecond(60000);

                   if (readDistanceMm(&distance))
                   {
                       putsUart0("Distance: ");
                       printDistanceMm(distance);

                       if (distance <= bottomDistance - RISE_THRESHOLD_MM)
                       {
                           riseDuty = duty;
                           riseDistance = distance;

                           putsUart0("Plunger started rising!\r\n");
                           break;
                       }
                   }
               }

               //exit the rising loop if movement was detected
               if (riseDuty >= 0)
                   break;
           }

           //read ultrasonic sensor every two PWM steps
           if ((duty % 2) != 0)
               continue;

           if (!readDistanceMm(&distance))
           {
               putsUart0("Echo timeout while increasing\r\n");

               if (++invalidReadings >= 3)
               {
                   setPWMduty(0);
                   putsUart0("Sensor lost. Fan stopped.\r\n");
                   return 1;
               }

               continue;
           }

           invalidReadings = 0;

           printDuty();
           putsUart0("Distance: ");
           printDistanceMm(distance);

           //detect rising during the normal PWM ramp
           if (distance <= bottomDistance - RISE_THRESHOLD_MM)
           {
               riseDuty = duty;
               riseDistance = distance;

               putsUart0("Plunger started rising!\r\n");
               break;
           }
       }

/*
    if (riseDuty < 0)
    {
        dutyCycle = 100;
        setPWMduty(dutyCycle);

        putsUart0("Holding PWM at 100%...\r\n");

        uint32_t holdStart = WTIMER0_TAV_R;

        // Hold for up to 10 additional seconds
        while ((uint32_t)(WTIMER0_TAV_R - holdStart) < 400000000U)
        {
            if (readDistanceMm(&distance))
            {
                putsUart0("Distance: ");
                printDistanceMm(distance);

                if (distance <= bottomDistance - RISE_THRESHOLD_MM)
                {
                    riseDuty = 100;
                    riseDistance = distance;

                    putsUart0("Plunger started rising!\r\n");
                    break;
                }
            }

            waitMicrosecond(60000);
        }
    }
*/
    if (riseDuty < 0)
    {
        putsUart0("Rise not detected. Stopping.\r\n");
        setPWMduty(0);
        return 1;
    }

    // STEP 2: Run fan at 100%, wait 5 seconds, measure top position.
    dutyCycle = 100;
    setPWMduty(dutyCycle);
    putsUart0("Fan at 100%. Waiting 5 seconds...\r\n");
    waitMicrosecond(HOLD_TIME_US);

    if (!readDistanceMm(&topDistance))
    {
        putsUart0("Cannot read top position. Stopping.\r\n");
        setPWMduty(0);
        return 1;
    }

    putsUart0("100% position (assumed top): ");
    printDistanceMm(topDistance);
    putsUart0("Decreasing PWM...\r\n");

    // LOOP 2: Decrease PWM 1% per step until the plunger starts to fall.
    invalidReadings = 0;
    for (int duty = 99; duty >= 0; duty--)
    {
        dutyCycle = (uint8_t)duty;
        setPWMduty(dutyCycle);
        waitMicrosecond(PWM_STEP_US);

        /*
        // Read sonar every two PWM steps, rather than every 30 ms.
        if ((duty % 2) != 0)
            continue;
*/
        if (!readDistanceMm(&distance))
        {
            putsUart0("Echo timeout while decreasing\r\n");
            if (++invalidReadings >= 3)
            {
                setPWMduty(0);
                putsUart0("Sensor lost. Fan stopped.\r\n");
                return 1;
            }
            continue;
        }

        invalidReadings = 0;
        printDuty();
        putsUart0("Distance: ");
        printDistanceMm(distance);

        if (distance >= topDistance + FALL_THRESHOLD_MM)
        {
            fallDuty = duty;
            fallDistance = distance;
            putsUart0("Plunger started falling!\r\n");
            break; // Stop the decrease as soon as falling is detected
        }
    }

    setPWMduty(0); // Stop the fan at end of experiment
    dutyCycle = 0;

    putsUart0("\r\nDONE!\r\n");
    putsUart0("Bottom: 0% at ");
    printDistanceMm(bottomDistance);

    putsUart0("Rise: ");
    printUint32((uint32_t)riseDuty);
    putsUart0("% at ");
    printDistanceMm(riseDistance);

    putsUart0("100% reference (assumed top): ");
    printDistanceMm(topDistance);

    if (fallDuty >= 0)
    {
        putsUart0("Fall: ");
        printUint32((uint32_t)fallDuty);
        putsUart0("% at ");
        printDistanceMm(fallDistance);
    }
    else
    {
        putsUart0("Fall: not detected\r\n");
    }

    return (fallDuty >= 0) ? 0 : 1;
}




void initHw(void){
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= PUSH_BUTTON2_MASK;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~(PUSH_BUTTON_MASK | PUSH_BUTTON2_MASK);               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK | PUSH_BUTTON2_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK | PUSH_BUTTON2_MASK; // enable internal pull-up for push button
    // mojo to program the pins for SW2. read the data sheet if you are curious

    GPIO_PORTD_DIR_R |= IN1 | IN2;

    GPIO_PORTD_DEN_R |= IN1 | IN2;

    GPIO_PORTD_DATA_R &= ~IN1;
    GPIO_PORTD_DATA_R |= IN2;
    waitMicrosecond(100000);


}


/*
 Your code  Correct register
PWM1_0_GENA_R   PWM1_2_GENB_R
PWM1_0_LOAD_R   PWM1_2_LOAD_R
PWM1_0_CMPA_R   PWM1_2_CMPB_R
PWM1_0_CTL_R    PWM1_2_CTL_R
 */


void pwm_init(void){

    SYSCTL_RCGCPWM_R    |= SYSCTL_RCGCPWM_R1;                                                                       // Config the PWM Clocks - Ports Enabled in Main
    _delay_cycles(3);

    GPIO_PORTF_DIR_R |= M1PWM5_MASK;
    GPIO_PORTF_DEN_R    |= M1PWM5_MASK;
    GPIO_PORTF_AMSEL_R &= ~M1PWM5_MASK;
    GPIO_PORTF_AFSEL_R  |= M1PWM5_MASK;
    GPIO_PORTF_PCTL_R   &= ~GPIO_PCTL_PF1_M ;
    GPIO_PORTF_PCTL_R   |= GPIO_PCTL_PF1_M1PWM5;                                           //ALT FUNCT. Selection
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~SYSCTL_RCC_PWMDIV_M) | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32;            //System is at 40MHz - PWM_DIV is 2 by Default = 20 MHz
    //PWM0_ENABLE_R       &= ~(PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
    PWM1_ENABLE_R  &= ~PWM_ENABLE_PWM5EN;

    PWM1_2_CTL_R = 0;
    PWM1_2_GENB_R        = PWM_2_GENB_ACTLOAD_ZERO | PWM_2_GENB_ACTCMPBD_ONE;                                       //When Triggered to ACT; Drive to 1
    //PWM0_1_GENB_R        = PWM_1_GENB_ACTLOAD_ZERO | PWM_1_GENB_ACTCMPBD_ONE;
    PWM1_2_LOAD_R      = PWM_LOAD - 1;
    PWM1_2_CMPB_R       = (PWM_LOAD - 1) / 2;
    //PWM0_1_CMPB_R       |= (PWM_LOAD - 1) / 2;
                                                                     //Enables / Synchs
    //enable PWM0 generator 0 and generator 1 counter
    PWM1_2_CTL_R        |= PWM_2_CTL_ENABLE;
    //PWM0_1_CTL_R        |= PWM_1_CTL_ENABLE;

    //PWM1_ENABLE_R |= PWM_ENABLE_PWM5EN;
}


void setPWMduty(uint8_t duty)
{
    if (duty == 0)
    {
        //force PF1 PWM output low
        PWM1_ENABLE_R &= ~PWM_ENABLE_PWM5EN;
        return;
    }

    if (duty >= 100)
    {
        //hold output high
        PWM1_2_GENB_R = PWM_2_GENB_ACTLOAD_ONE;
    }
    else
    {
        //normal PWM generation
        PWM1_2_GENB_R = PWM_2_GENB_ACTLOAD_ZERO | PWM_2_GENB_ACTCMPBD_ONE;

        //PWM1_2_CMPB_R = ((PWM_LOAD - 1) * duty) / 100;
        PWM1_2_CMPB_R = ((PWM_LOAD * duty) / 100) - 1;
    }

    //enable PF1 PWM output
    PWM1_ENABLE_R |= PWM_ENABLE_PWM5EN;
}


void printUint32(uint32_t number){
    char buffer[11];
    uint8_t i = 10;

    buffer[10] = '\0';

    do
    {
        buffer[--i] = (number % 10) + '0';
        number /= 10;
    }
    while (number > 0);

    putsUart0(&buffer[i]);
}

void printDuty(void){
    putsUart0("PWM Duty Cycle: ");
    printUint32(dutyCycle);
    putsUart0("%\r\n");
}


void GPIOPortE_Handler(void){

    if(GPIO_PORTE_MIS_R & ECHO){
        GPIO_PORTE_ICR_R = ECHO;
        //check if echo is high
        if(GPIO_PORTE_DATA_R & ECHO){
            if(echo_state == 1){
                start = WTIMER0_TAV_R;
                echo_state = 2;
            }
            //check if echo is low and calcualte the duration
        }else{

            if(echo_state == 2){
                end = WTIMER0_TAV_R;

                //calculate elapesed timer ticks
                echo_duration = end - start;
                echo_state = 3;
                done = 1;

            }

        }
    }
}

void init_echo_interrupt(void){
    GPIO_PORTE_IM_R &= ~ECHO;
    GPIO_PORTE_IS_R &= ~ECHO; //edge sensitive
    GPIO_PORTE_IBE_R |= ECHO; //trigger both edges
    GPIO_PORTE_ICR_R = ECHO;
    GPIO_PORTE_IM_R |= ECHO;
    //PORT E interrupt number is 4
    NVIC_EN0_R |= 0x00000010;
}

void init_gpio(void){

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);


    GPIO_PORTE_DIR_R |= TRIG;
    GPIO_PORTE_DIR_R &= ~ECHO;
    GPIO_PORTE_DEN_R |= TRIG | ECHO;
    GPIO_PORTE_AFSEL_R &= ~(TRIG | ECHO);
    GPIO_PORTE_AMSEL_R &= ~(TRIG | ECHO);

    //init trig to low
    GPIO_PORTE_DATA_R &= ~TRIG;



}


// Wide Timer 0: free-running 32-bit counter, counts up at 40 MHz
void init_WTIMER(void){

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);

    //turn of wide timer at first
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;

    WTIMER0_CFG_R = 4;

    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // periodic, count up

    WTIMER0_TAILR_R = 0xFFFFFFFF; // count all the way up, then wrap

    WTIMER0_CTL_R |= TIMER_CTL_TAEN; // turn-on timer

}


uint8_t readDistanceMm(float *distance)
{
    //do not begin a reading while ECHO is high
    if (GPIO_PORTE_DATA_R & ECHO)
        return 0;

    //reset measurement
    GPIO_PORTE_IM_R &= ~ECHO;
    done = 0;
    echo_state = 1;
    echo_duration = 0;

    GPIO_PORTE_ICR_R = ECHO;
    GPIO_PORTE_IM_R |= ECHO;

    uint32_t trigger_time = WTIMER0_TAV_R;

    //send 10 us trigger pulse
    GPIO_PORTE_DATA_R |= TRIG;
    waitMicrosecond(10);
    GPIO_PORTE_DATA_R &= ~TRIG;

    //wait for echo or timeout
    while ((done == 0) &&
           ((uint32_t)(WTIMER0_TAV_R - trigger_time)
                < ECHO_TIMEOUT_TICKS))
    {
    }

    echo_state = 0;

    if (done == 0)
        return 0;

    //403 MHz timer = 40 ticks per microsecond
    float elapsed_us = (float)echo_duration / 40.0f;

    //convert round-trip sound travel to distance
    *distance = (elapsed_us * 343.0f) / 2000.0f;

    return 1;
}

void printDistanceMm(float distance)
{
    uint32_t scaled =
        (uint32_t)(distance * 100.0f + 0.5f);

    printUint32(scaled / 100);
    putsUart0(".");

    if ((scaled % 100) < 10)
    {
        putsUart0("0");
    }

    printUint32(scaled % 100);
    putsUart0(" mm\r\n");
}



