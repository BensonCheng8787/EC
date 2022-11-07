// ENGR-2350 Template Project
#include "engr2350_msp432.h"

void PWMInit();
void ADCInit();
void GPIOInit();
void PWM_ISR();
void Encoder_ISR();
void setSpeed();

//timer configs
Timer_A_UpModeConfig TA2cfg; // Using P5.7, TA2.2
Timer_A_ContinuousModeConfig TA3cfg; //capture speed
Timer_A_CompareModeConfig TA0_ccrL; //speed left  (set)
Timer_A_CompareModeConfig TA0_ccrR; //speed right (set)
Timer_A_CaptureModeConfig capconfig_l;
Timer_A_CaptureModeConfig capconfig_r;

//encoder isr
int32_t enc_countsL = 0; //Keep track the timer counts since the capture event, track timer counts between encoder edges
int32_t enc_countsR = 0;
int32_t enc_trackL = 0;//keep track of timer since capture
int32_t enc_trackR = 0;
int32_t timer_sumL = 0; //store summation of wheel speed
int32_t timer_sumR = 0;
uint8_t timer_countL = 0; //track number of measurements in the summation variable
uint8_t timer_countR = 0;
uint32_t avgL;
uint32_t avgR;

//pwm
uint8_t timer_flag = 0;
int16_t pwm_max = 2300; // Maximum limit on PWM output
int16_t pwm_min = 0; // Minimum limit on PWM output
int16_t pwm_setL = 1200; // Calculated PWM output (control output)
int16_t pwm_setR = 1200;
//left control
float kpL = 0.05; // proportional control gain
float error_sumL = 0; // Integral control error summation
float kiL = 65536; // integral control gain
//right control
float kpR = 0.05; // proportional control gain
float error_sumR = 0; // Integral control error summation
float kiR = 65536; // integral control gain


uint16_t pot_val; // ADC value from potetiometer
float desired; // Current "setpoint" voltage, from POT
uint16_t rc_val; // ADC value from RC circuit
float actual = 0; // Current output voltage from RC circuit

// Main Function
int main(void)
{
    SysInit();

    GPIOInit();
    PWMInit();
    ADCInit();

    printf("\r\n\nDes. ADC\tDes. V\tAct. ADC\tAct. V\tPWM Set\r\n");

    while(1){
        // If the PWM has cycled, request an ADC sample
        if(timer_flag){
            // Add ADC conversion code here
            ADC14_toggleConversionTrigger();
            while(ADC14_isBusy()){}
            //part 1
            pot_val = ADC14_getResult(ADC_MEM14);
            desired = 1.0*pot_val*3.3/16384;
            //part2
            rc_val = ADC14_getResult(ADC_MEM15);
            actual = 1.0*rc_val*3.3/16384;


            // *********** CONTROL ROUTINE *********** //
            /*error_sum += desired-actual; // perform "integration"
            pwm_set = kp*(pwm_max-pwm_min)/desired-ki*error_sum; // PI control equation
            if(pwm_set > pwm_max) pwm_set = pwm_max;  // Set limits on the pwm control output
            if(pwm_set < pwm_min) pwm_set = pwm_min;
            Timer_A_setCompareValue(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,pwm_set); // enforce pwm control output
            // ********* END CONTROL ROUTINE ********* //*/



            printf("\r%5u\t   %1.3f\t  %5u\t   %1.3f\t%5u",pot_val,desired,rc_val,actual,pwm_set); // report
            __delay_cycles(240e3); // crude delay to prevent this from running too quickly
            timer_flag = 0; // Mark that we've performed the control loop

        }
    }
}

void setSpeed(){
    //get ADC sample
    ADC14_toggleConversionTrigger();
    while(ADC14_isBusy()){}
    //left pot
    pot_valL = ADC14_getResult(ADC_MEM14);
    desiredL = 1.0*pot_val*3.3/16384;
    //right pot
    pot_valR = ADC14_getResult(ADC_MEM15);
    desiredR = 1.0*pot_val*3.3/16384;

    //conversion, set PWM duty cycle to match voltage
    /*error_sum += desired-actual; // perform "integration"
    pwm_set = kp*(pwm_max-pwm_min)/desired-ki*error_sum; // PI control equation
    if(pwm_set > pwm_max) pwm_set = pwm_max;  // Set limits on the pwm control output
    if(pwm_set < pwm_min) pwm_set = pwm_min;
    Timer_A_setCompareValue(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,pwm_set); // enforce pwm control output*/

}




void GPIOInit(){
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION); // PWM output right
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION); // left
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1,GPIO_TERTIARY_MODULE_FUNCTION); //read in pot v
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,GPIO_TERTIARY_MODULE_FUNCTION);
}


void ADCInit(){
    // Activity Stuff...
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_1, 0);
    ADC14_configureConversionMemory(ADC_MEM14, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, 0);//p6 pin1
    ADC14_configureConversionMemory(ADC_MEM15, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, 0);//p6 pin0
    //ADC14_configureSingleSampleMode(ADC_MEM14, true); // part 1
    ADC14_configureMultiSequenceMode(ADC_MEM14, ADC_MEM15, true);//part2
    //ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE6, false);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();
}

//set up timer, pwm, and encoder
void PWMInit(){
    // Set up Timer_A0 to run at 100ms
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA0cfg.timerPeriod = 37500;//100ms period
    TA0cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA2cfg);

    // Set up Timer_A3 to run contonious
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE, &TA3cfg);
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
    //Timer_A_registerInterrupt(TIMER_A3_BASE ,TIMER_A_CCR0_INTERRUPT ,Encoder_ISR);
    Timer_A_initCapture(TIMER_A3_BASE, &capconfig_r);

    // Configure TA2.CCR2 for PWM generation (left)
    TA0_ccrL.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccrL.compareValue = pwm_setL;
    TA0_ccrL.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    Timer_A_initCompare(TIMER_A0_BASE,&TA2_ccr);
    // Configure TA2.CCR3 for PWM generation (right)
    TA0_ccrR.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccrR.compareValue = pwm_setL;
    TA0_ccrR.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    Timer_A_initCompare(TIMER_A0_BASE,&TA3_ccr);

    Timer_A_registerInterrupt(TIMER_A0_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,PWM_ISR);
    Timer_A_startCounter(TIMER_A2_BASE,TIMER_A_UP_MODE);
}

void PWM_ISR(){//timer ISR
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    timer_flag = 1;
}

void Encoder_ISR(){
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE)){
            Timer_A_clearInterruptFlag(TIMER_A3_BASE);
            enc_trackL+=65536;
            enc_trackR+=65536;
        }
    //left capture
    else if (Timer_A_getCaptureCompareEnabledInterruptStatus(
            TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)
                    & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        //calculate timer between edges
        enc_countsL = enc_trackL + Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_trackL = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        timer_sumL+= enc_countsL;
        timer_countL++;
        enc_totalL++;
        if(timer_countL==6){
            avgL = timer_sumL/6;
            timer_sumL=0;
            timer_countL=0;
        }
    }
    //right capture
    else if (Timer_A_getCaptureCompareEnabledInterruptStatus(
            TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)
                    & TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) {
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        //calculate timer between edges
        enc_countsR = enc_trackR + Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_trackR = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        timer_sumR+= enc_countsR;
        timer_countR++;
        enc_totalR++;
        if(timer_countR==6){
            avgR = timer_sumR/6;
            timer_sumR=0;
            timer_countR=0;
        }
    }
}


