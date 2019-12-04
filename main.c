#include <stdlib.h>
#include <stdio.h>
#include <msp430.h>
#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"

/*
 * This project contains some code samples that may be useful.
 *
 */


char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

void step_x(){

    P8DIR = 0x02;
    P2DIR = 0x80;
    P1DIR = 0x03;

    P8OUT = 0x02;
    P2OUT = 0x00;
    P1OUT = 0x02;

    volatile unsigned int i;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);

    P8OUT = 0x00;
    P2OUT = 0x00;
    P1OUT = 0x03;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);


    P8OUT = 0x00;
    P2OUT = 0x80;
    P1OUT = 0x01;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);



    P8OUT = 0x02;
    P2OUT = 0x80;
    P1OUT = 0x00;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);

}

void rstep_x(){

    P8DIR = 0x02;
    P2DIR = 0x80;
    P1DIR = 0x03;




    volatile unsigned int i;




    P8OUT = 0x02;
    P2OUT = 0x80;
    P1OUT = 0x00;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);

    P8OUT = 0x00;
        P2OUT = 0x80;
        P1OUT = 0x01;

        i = 2000;
        do{
            i--;
        }
        while(i > 0);

        P8OUT = 0x00;
            P2OUT = 0x00;
            P1OUT = 0x03;

            i = 2000;
            do{
                i--;
            }
            while(i > 0);

            P8OUT = 0x02;
                P2OUT = 0x00;
                P1OUT = 0x02;



                i = 2000;
                do{
                    i--;
                }
                while(i > 0);

}


void rstep_y(){
    P8DIR = 0x0c;
    P2DIR = 0x20;
    P5DIR = 0x02;




    volatile unsigned int i;



    P8OUT = 0x08;
    P2OUT = 0x00;
    P5OUT = 0x02;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);

    P8OUT = 0x0c;
        P2OUT = 0x00;
        P5OUT = 0x00;

        i = 2000;
        do{
            i--;
        }
        while(i > 0);

        P8OUT = 0x04;
            P2OUT = 0x20;
            P5OUT = 0x00;

            i = 2000;
            do{
                i--;
            }
            while(i > 0);

            P8OUT = 0x00;
                P2OUT = 0x20;
                P5OUT = 0x02;



                i = 2000;
                do{
                    i--;
                }
                while(i > 0);


}

void step_y(){
    P8DIR = 0x0c;
    P2DIR = 0x20;
    P5DIR = 0x02;

    P8OUT = 0x00;
    P2OUT = 0x20;
    P5OUT = 0x02;

    volatile unsigned int i;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);

    P8OUT = 0x04;
    P2OUT = 0x20;
    P5OUT = 0x00;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);


    P8OUT = 0x0c;
    P2OUT = 0x00;
    P5OUT = 0x00;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);



    P8OUT = 0x08;
    P2OUT = 0x00;
    P5OUT = 0x02;

    i = 2000;
    do{
        i--;
    }
    while(i > 0);
}

void main(void)
{
    char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    //Init_GPIO();    //Sets all pins to output low as a default
    //Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    //Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    //__enable_interrupt();




    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

//    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1);
//    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN3 | GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN3);

    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN3 | GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);


    int coords[5][2];

//    memset(coords[0], 0, 2);

    for (int k = 0; k < 5; k++){

        int length[2] = { };
        unsigned int num[2][3] = { };
        int isY = 0;

        while(1){
            if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){
                if (length[isY] <= 3){
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){ //2
                        num[isY][length[isY]] = 2;
                        length[isY]++;
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH);
                    }

                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN6);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){ //5

                        num[isY][length[isY]] = 5;
                        length[isY]++;
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH);
                    }


                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4 | GPIO_PIN6);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);


                    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){ //8

                        num[isY][length[isY]] = 8;
                        length[isY]++;
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH);
                    }

                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN6);

                    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){ //0
                        num[isY][length[isY]] = 0;
                        length[isY]++;
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH);
                    }
                }
            }
            else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){

                if (length[isY] <= 3){



                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){ //1

                    num[isY][length[isY]] = 1;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH);
                }

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){ //4

                    num[isY][length[isY]] = 4;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH);
                }

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4 | GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);


                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){ //7

                    num[isY][length[isY]] = 7;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH);
                }


                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN6);


                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_HIGH){ //*

                }
                }
            }

            else if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH){

                if (length[isY] <= 3){
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH){ //3

                    num[isY][length[isY]] = 3;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH);
                }

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH){ //6

                    num[isY][length[isY]] = 6;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH);
                }


                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4 | GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);


                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH){ //9

                    num[isY][length[isY]] = 9;
                    length[isY]++;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH);
                }

                }
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN6);

                if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH){ //#

                    if (isY){
                        clearLCD();
                        isY = 0;
                        while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH);
                        break;
                    }
                    isY = 1;
                    while(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH);
                }
            }
            else{

                if(length[0] == 0){
                    showChar('0', 8);
                    showChar('0', 6);
                    showChar('0', 4);
                }
                else if(length[0] == 1){
                    showChar(num[0][0] + '0', 8);

                }
                else if(length[0] == 2){
                    showChar(num[0][0] + '0', 6);
                    showChar(num[0][1] + '0', 8);
                }
                else if(length[0] == 3){
                    showChar(num[0][2] + '0', 8);
                    showChar(num[0][1] + '0', 6);
                    showChar(num[0][0] + '0', 4);
                }


                if(length[1] == 0){
                    showChar('0', 18);
                    showChar('0', 2);
                    showChar('0', 10);
                }
                else if(length[1] == 1){
                    showChar(num[1][0] + '0', 18);

                }
                else if(length[1] == 2){
                    showChar(num[1][0] + '0', 2);
                    showChar(num[1][1] + '0', 18);
                }
                else if(length[1] == 3){
                    showChar(num[1][2] + '0', 18);
                    showChar(num[1][1] + '0', 2);
                    showChar(num[1][0] + '0', 10);
                }

            }

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN3 | GPIO_PIN4);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);




        }

        int x = 0;
        int y = 0;

        for (int i = 0; i < length[0]; i++){
            x = (x*10) + num[0][i];
        }
        coords[k][0] = x;

        for (int j = 0; j < length[1]; j++){
            y = (y*10) + num[1][j];
        }
        coords[k][1] = y;
    }


    int curr_x = 0;
    int curr_y = 0;

    for (int i = 0; i < 5; i++){
        int delta_x = coords[i][0] - curr_x;
        int delta_y = coords[i][1] - curr_y;

        while(curr_x != coords[i][0]) {

            if (delta_x < 0) {
                rstep_x();
                curr_x--;
            } else {
                step_x();
                curr_x++;
            }
            int k = curr_x;
            showChar((k % 10) + '0', 8);
            k /= 10;
            showChar((k % 10) + '0', 6);
            k /= 10;
            showChar((k % 10) + '0', 4);

        }
        while(curr_y != coords[i][1]){

            if (delta_y < 0) {
                rstep_y();
                curr_y--;
            } else {
                step_y();
                curr_y++;
            }
            int k = curr_y;
            showChar((k % 10) + '0', 18);
            k /= 10;
            showChar((k % 10) + '0', 2);
            k /= 10;
            showChar((k % 10) + '0', 10);
        }

        int j = 20000;
        do {
            j--;
        } while (j < 0);

    }
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, 0x80, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, 0x09, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
