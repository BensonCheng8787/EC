/**********************************************************************/
// ENGR-2350 Template Project
// Name: XXXX
// RIN: XXXX
// This is the base project for several activities and labs throughout
// the course.  The outline provided below isn't necessarily *required*
// by a C program; however, this format is required within ENGR-2350
// to ease debugging/grading by the staff.
/**********************************************************************/

// We'll always add this include statement. This basically takes the
// code contained within the "engr_2350_msp432.h" file and adds it here.
#include "engr2350_msp432.h"

void GPIOInit();
void I2CInit();
uint16_t readCompass();
uint16_t readRanger();
void I2C_writeData( uint32_t moduleInstance , uint8_t PeriphAddress , uint8_t StartReg , uint8_t * data , uint8_t len );
void I2C_readData( uint32_t moduleInstance , uint8_t PeriphAddress , uint8_t StartReg , uint8_t * data , uint8_t len );

uint8_t datacom[2];
uint8_t lencom=2;
uint16_t heading;

uint8_t  dataran[2];
uint8_t lenran=2;
uint16_t range;



eUSCI_I2C_MasterConfig config;


// Add global variables here, as needed.


int main(void) /* Main Function */
{
    // Add local variables here, as needed.

    // We always call the "SysInit()" first to set up the microcontroller
    // for how we are going to use it.
    SysInit();
    GPIOInit();
    I2CInit();
    // Place initialization code (or run-once) code here

    while(1){
        // Place code that runs continuously in here

        printf("b\r\n");
        printf("Compass: %4u\tRanger: %4u\r\n",readCompass(),readRanger());
        printf("a\r\n");

         __delay_cycles(2.4e6); // Wait 1/10 of a second
    }
}

void I2CInit(){
    config.selectClockSource = (EUSCI_B_I2C_CLOCKSOURCE_SMCLK);
    config.i2cClk=24000000;
    config.dataRate=(EUSCI_B_I2C_SET_DATA_RATE_100KBPS);
    config.byteCounterThreshold=(0);
    I2C_initMaster( EUSCI_B3_BASE , &config );
    I2C_enableModule(EUSCI_B3_BASE);
}

void GPIOInit(){
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6,GPIO_PIN6,GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6,GPIO_PIN7,GPIO_SECONDARY_MODULE_FUNCTION);
}

uint16_t readCompass(){
    if(lencom >= 2) {
        I2C_readData( EUSCI_B3_BASE , 0x60 , 2 , datacom , lencom );
        heading = (datacom[0]<<8);
        heading |= datacom[1];
        return heading;
    }
    else
        return 0;
}

uint16_t readRanger(){
    if(lenran >= 2) {
        I2C_readData( EUSCI_B3_BASE , 0x70 , 2 , dataran , lenran );
        range = (dataran[0]<<8);
        range |= dataran[1];

        dataran[0]=0x51;
        I2C_writeData(EUSCI_B3_BASE,0x70,0,dataran,1);
        return range;
    }
    else
        return 0;
}



void I2C_writeData(uint32_t moduleInstance
                  ,uint8_t PeriphAddress
                  ,uint8_t StartReg
                  ,uint8_t *data
                  ,uint8_t len)
{
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_TRANSMIT_MODE); // Indicate a write operation

    I2C_masterSendMultiByteStart(moduleInstance,StartReg); // Start the communication.
                // This function does three things. It sends the START signal,
                // sends the address, and then sends the start register.

    // This code loops through all of the bytes to send.
    uint8_t ctr;
    for(ctr = 0;ctr<len;ctr++){
        I2C_masterSendMultiByteNext(moduleInstance,data[ctr]);
    }
    // Once all bytes are sent, the I2C transaction is stopped by sending the STOP signal
    I2C_masterSendMultiByteStop(moduleInstance);

    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}

void I2C_readData(uint32_t moduleInstance
                 ,uint8_t PeriphAddress
                 ,uint8_t StartReg
                 ,uint8_t *data
                 ,uint8_t len)
{
    // First write the start register to the peripheral device. This can be
    // done by using the I2C_writeData function with a length of 0.
    I2C_writeData(moduleInstance,PeriphAddress,StartReg,0,0);

    Interrupt_disableMaster(); //  Disable all interrupts to prevent timing issues

    // Then do read transaction...
    I2C_setSlaveAddress(moduleInstance,PeriphAddress); // Set the peripheral address
    I2C_setMode(moduleInstance,EUSCI_B_I2C_RECEIVE_MODE); // Indicate a read operation
    I2C_masterReceiveStart(moduleInstance); // Start the communication. This function
                // doe two things: It first sends the START signal and
                // then sends the peripheral address. Once started, the eUSCI
                // will automatically fetch bytes from the peripheral until
                // a STOP signal is requested to be sent.

    // This code loops through 1 less than all bytes to receive
    uint8_t ctr;
    for(ctr = 0;ctr<(len-1);ctr++){
        uint32_t tout_tmp = 10000;
        while(!(EUSCI_B_CMSIS(moduleInstance)->IFG & EUSCI_B_IFG_RXIFG0) && --tout_tmp); // Wait for a data byte to become available
        if(tout_tmp){
            data[ctr] = I2C_masterReceiveMultiByteNext(moduleInstance); // read and store received byte
        }else{
            data[ctr] = 0xFF;
        }
    }
    // Prior to receiving the final byte, request the STOP signal such that the
    // communication will halt after the byte is received.
    data[ctr] = I2C_masterReceiveMultiByteFinish(moduleInstance); // send STOP, read and store received byte

    Interrupt_enableMaster(); // Re-enable interrupts

    __delay_cycles(200); // A short delay to avoid starting another I2C transaction too quickly
}
// Add function declarations here as needed

// Add interrupt functions last so they are easy to find
