// ENGR-2350 Lab 4 Template S22

#include "engr2350_msp432.h"

void GPIOInit();
void TimerInit();
void ADCInit();
void Encoder_ISR();
void T1_100ms_ISR();

void checkState();//checks the current state of the car(straight or turning) updates var state if done

Timer_A_UpModeConfig TA0cfg;
Timer_A_UpModeConfig TA1cfg;
Timer_A_ContinuousModeConfig TA3cfg;
Timer_A_CompareModeConfig TA0_ccr3;
Timer_A_CompareModeConfig TA0_ccr4;
Timer_A_CaptureModeConfig TA3_ccr0;
Timer_A_CaptureModeConfig TA3_ccr1;


// Encoder total events
uint32_t enc_total_L,enc_total_R;
// Speed measurement variables
float potSpeed,desiredSpeed,actualSpeedL,actualSpeedR;
float potDist,desiredDist,actualDist;
int32_t Tach_L_count,Tach_L,Tach_L_sum,Tach_L_sum_count,Tach_L_avg; // Left wheel
int32_t Tach_R_count,Tach_R,Tach_R_sum,Tach_R_sum_count,Tach_R_avg; // Right wheel
//speed control
float ki = 0.1;
float pwm_setL,pwm_setR,pwm_max,pwm_min;
//turn control
float pwm_set,dt,rl,rr,vl,vr,r,dv;

uint16_t curDistance; //current distance
uint8_t state; //1 means straight and 0 means turning
uint32_t errorSum;


uint8_t run_control = 0; // Flag to denote that 100ms has passed and control should be run.

int main(void)
{
    SysInit();
    GPIOInit();
    ADCInit();
    TimerInit();

    __delay_cycles(24e6);

    r = 74.5;
    pwm_max = 720;//90%
    pwm_min = 80;//10%

    while(1){
        if(run_control){    // If 100 ms has passed
            run_control = 0;    // Reset the 100 ms flag
            //get speed
            ADC14_toggleConversionTrigger();
            while(ADC14_isBusy()){}
            potSpeed = ADC14_getResult(ADC_MEM0);
            desiredSpeed = (800*potSpeed)/16384;
            actualSpeedL = (8*1500000)/Tach_L_sum;//actual speed for left wheel
            actualSpeedR = (8*1500000)/Tach_R_sum;//actual speed for right wheel

            //left wheel
            errorSum += desiredSpeed - actualSpeedL;//integration for left
            pwm_setL = desiredSpeed + ki*errorSum;//PI control equation
            if(pwm_setL > pwm_max) {
                pwm_setL = pwm_max;
            }
            else if (pwm_setL < pwm_min) {
                pwm_setL = pwm_min;
            }
            Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,pwm_setL); // enforce pwm control output


            //right wheel
            errorSum += desiredSpeed - actualSpeedR;//integration for left
            pwm_setR = desiredSpeed + ki*errorSum;//PI control equation
            if(pwm_setR > pwm_max) {
                pwm_setR = pwm_max;
            }
            else if (pwm_setR < pwm_min) {
                pwm_setR = pwm_min;
            }
            Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,pwm_setR); // enforce pwm control output


            potDist = ADC14_getResult(ADC_MEM1);


            printf("\r\n\npotSpeed: %1.3f  desiredSpeed: %1.3f    \r\nactualR: %1.3f   actualL: %1.3f   \r\npwmR: %1.3f      pwmL: %1.3f  \r\n%d",potSpeed,desiredSpeed,actualSpeedR, actualSpeedL,pwm_setR,pwm_setL,Tach_L_sum);


            //turn control//
//            dv = (pwm_set*0.5*74.5)/500000;


//
//            //speed control//
//            //left inetgration
//            Tach_L_sum += desiredSpeed - actualSpeed;//integration
//            pwm_setL = kp*(pwm_max-pwm_min)/desiredSpeed-ki*Tach_L_sum; // PI control equation
//            if(pwm_setL > pwm_max) pwm_setL = pwm_max;  // Set limits on the pwm control output
//            if(pwm_setL < pwm_min) pwm_setL = pwm_min;
//            pwm_set = pwm_setL;
//            Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,pwm_setL); // enforce pwm control output
//            //right integration
//            Tach_R_sum += desiredSpeed - actualSpeed;//integration
//            pwm_setR = kp*(pwm_max-pwm_min)/desiredSpeed-ki*Tach_R_sum; // PI control equation
//            if(pwm_setR > pwm_max) pwm_setR = pwm_max;  // Set limits on the pwm control output
//            if(pwm_setR < pwm_min) pwm_setR = pwm_min;
//            Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4,pwm_setR); // enforce pwm control output
//            __delay_cycles(240e3);
        }
    }
}

void ADCInit(){
    // Add your ADC initialization code here.
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A12, false);//p4 pin1(speed)
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, false);//p4 pin4(turn)
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);//part2
    ADC14_enableConversion();
    //  Don't forget the GPIO, either here or in GPIOInit()!!


}

void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4,GPIO_PIN1,GPIO_TERTIARY_MODULE_FUNCTION);//desired speed potentiometer
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4,GPIO_PIN4,GPIO_TERTIARY_MODULE_FUNCTION);//desired turning radius

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motors are OFF
}

void TimerInit(){
    // Configure PWM timer for 30 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 800;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA1cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA1cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA1cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA1cfg.timerPeriod = 37500;
    Timer_A_configureUpMode(TIMER_A1_BASE,&TA1cfg);
    Timer_A_registerInterrupt(TIMER_A1_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T1_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}


void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_R_count += 65536;
        Tach_L_count += 65536;
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_R++;   // Increment the total number of encoder events for the left encoder
        // Calculate and track the encoder count values
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        // If 6 values have been received, average them.
        if(Tach_R_sum_count == 6){
            Tach_R_avg = Tach_R_sum/6;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_L++;
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        if(Tach_L_sum_count == 6){
            Tach_L_avg = Tach_L_sum/6;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    }
}

void T1_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    run_control = 1;
}

void checkState(){
    curDistance = 0;

    curDistance = (enc_total_L * 0.60956)/360;

//    //distance of the straight away
//    if (curdistance >= actualDist) {
//        state = 1; //1 means that it switches to turning
//    }
//    else if (curdistance >= actualDist) {
//        state = 0; //0 means it switches turning
//    }

}
