/**********************************************************************/
// ENGR-2350 Template Project
// Name: Tyler Layton
// RIN: 662018228
// This is the base project for several activities and labs throughout
// the course.  The outline provided below isn't necessarily *required*
// by a C program; however, this format is required within ENGR-2350
// to ease debugging/grading by the staff.
/**********************************************************************/

// We'll always add this include statement. This basically takes the
// code contained within the "engr_2350_msp432.h" file and adds it here.
#include "engr2350_msp432.h"

// Add function prototypes here, as needed.
void GPIOInit();
void TimerInit();
void Encoder_ISR();

// Add global variables here, as needed.
Timer_A_CaptureModeConfig capconfig_l;
Timer_A_CaptureModeConfig capconfig_r;
Timer_A_CompareModeConfig comconfig_l;
Timer_A_CompareModeConfig comconfig_r;
Timer_A_ContinuousModeConfig config_con;
Timer_A_UpModeConfig config_up;

int32_t enc_counts_track = 0; //Keep track the timer counts since the capture event, track timer counts between encoder edges

int32_t timer_counts = 0; //store summation of wheel speed

uint8_t timer_measure = 0; //track number of measurements in the summation variable

uint8_t cur_pwm; //store the current PWM





int main(void) /* Main Function */
{
    // Add local variables here, as needed.

    // We always call the "SysInit()" first to set up the microcontroller
    // for how we are going to use it.
    SysInit();
    GPIOInit();
    TimerInit();

    // Place initialization code (or run-once) code here

    while(1){

    }
}

// Add function declarations here as needed
void GPIOInit() {


    GPIO_setAsOutputPin(3,GPIO_PIN6); //right enable
    GPIO_setAsOutputPin(5,GPIO_PIN5); //right direction
    GPIO_setAsOutputPin(3,GPIO_PIN7); //left enable
    GPIO_setAsOutputPin(5,GPIO_PIN4); //left direction

    GPIO_setAsPeripheralModuleFunctionOutputPin(2 , GPIO_PIN6 , GPIO_PRIMARY_MODULE_FUNCTION ); //pwm speed right, timer 0 register 3
    GPIO_setAsPeripheralModuleFunctionOutputPin( 2 , GPIO_PIN7 , GPIO_PRIMARY_MODULE_FUNCTION ); //pwm speed left, timer 0 register 4
    GPIO_setAsPeripheralModuleFunctionInputPin(10 , GPIO_PIN4 , GPIO_PRIMARY_MODULE_FUNCTION ); //timer 3 register 0
    GPIO_setAsPeripheralModuleFunctionInputPin(10 , GPIO_PIN5 , GPIO_PRIMARY_MODULE_FUNCTION ); //timer 3 register 1
}

void TimerInit() {
    //timer up mode
    config_up.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    config_up.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    config_up.timerPeriod = 800;
    Timer_A_configureUpMode(TIMER_A0_BASE, &config_up);

    //timer continuous mode
    config_con.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    config_con.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    config_con.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE, &config_con);
    Timer_A_registerInterrupt(TIMER_A3_BASE ,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT ,Encoder_ISR);

    //capture mode left
    capconfig_l.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    capconfig_l.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    capconfig_l.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    capconfig_l.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    capconfig_l.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_registerInterrupt(TIMER_A3_BASE ,TIMER_A_CCR0_INTERRUPT ,Encoder_ISR);
    Timer_A_initCapture(TIMER_A3_BASE, &capconfig_l);

    //capture mode right
    capconfig_r.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    capconfig_r.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    capconfig_r.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    capconfig_r.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    capconfig_r.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_registerInterrupt(TIMER_A3_BASE ,TIMER_A_CCR0_INTERRUPT ,Encoder_ISR);
    Timer_A_initCapture(TIMER_A3_BASE, &capconfig_r);

    //compare mode left
    comconfig_l.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    comconfig_l.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    comconfig_l.compareValue = 800;
    Timer_A_initCompare(TIMER_A0_BASE, &comconfig_l);

    //compare mode right
    comconfig_r.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    comconfig_r.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    comconfig_r.compareValue = 800;
    Timer_A_initCompare(TIMER_A0_BASE, &comconfig_r);

    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void Encoder_ISR() {

    //left capture
    if (Timer_A_getCaptureCompareEnabledInterruptStatus(
            TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)
                    & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) {

    }
    else if (Timer_A_getCaptureCompareEnabledInterruptStatus(
            TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)
                    & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) {

    }
}
// Add interrupt functions last so they are easy to find
