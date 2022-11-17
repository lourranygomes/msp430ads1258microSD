/* --COPYRIGHT--,BSD
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "hal.h"


#define SSI_PERIP (SSI3_BASE)


//****************************************************************************
//
// Internal variables and macros
//
//****************************************************************************

// Flag to indicate if an
static volatile bool flag_nDRDY_INTERRUPT = false;

/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)



//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitSPI(void);



//****************************************************************************
//
// External Functions (prototypes declared in interface.h)
//
//****************************************************************************

/**
 * \fn void InitADCPeripherals(void)
 * \brief Initializes MCU peripherals for interfacing with the ADS1258
 */
void InitADCPeripherals(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.
    // TODO: If you have additional code to power the ADC, you may want to add it here:


    // Initialize GPIOs pins used by ADS1258
    InitGPIO();

    // Initialize SPI peripheral used by ADS1258
    InitSPI();
}



//****************************************************************************
//
// Timing functions
//
//****************************************************************************

/**
 * \fn void delay_ms(uint32_t delay_time_ms, uint32_t sysClock_Hz)
 * \brief Provides a timing delay with ms resolution
 * \param delay_time_ms number of ms to delay
 */
void delay_ms(uint32_t delay_time_ms)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */

    //const uint32_t cycles_per_loop = 3;
    //const uint32_t delay = (delay_time_ms * getSysClock() / (cycles_per_loop * 1000u));
    uint32_t i=0;
    for(i=0; i<delay_time_ms; i++)
        __delay_cycles(1000);
}

/**
 * \fn void delay_ns(uint32_t delay_time_us, uint32_t sysClock_Hz)
 * \brief Provides a timing delay with ns resolution
 * \param delay_time_us number of us to delay
 */
void delay_ns(uint32_t delay_time_us)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */

    //const uint32_t cycles_per_loop = 3;
    uint32_t i=0;
    for(i=0; i<delay_time_us; i++)
        __delay_cycles(1);
}



//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************

/**
 * \fn void InitGPIO(void)
 * \brief Configures the ADS1258 GPIO pins
 */
void InitGPIO(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...



    //Set P1.1 for AD reset
    GPIO_setOutputHighOnPin(
        GPIO_PORT_P1,
        GPIO_PIN1
        );




    //Set P1.1 for AD reset
    //Set P1.0 to output direction
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0
        );





    /* Enable the clock to the GPIO Port D and wait for it to be ready */
    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)))
    {

    }*/

    /* Configure the GPIO for 'CLKSEL' as output and set low */
    //MAP_GPIOPinTypeGPIOOutput(CLKSEL_PORT, CLKSEL_PIN);
    //MAP_GPIOPinWrite(CLKSEL_PORT, CLKSEL_PIN, 0);

    GPIO_setAsOutputPin(CLKSEL_PORT, CLKSEL_PIN);
    GPIO_setOutputLowOnPin(CLKSEL_PORT, CLKSEL_PIN);


    /* Enable the clock to the GPIO Port M and wait for it to be ready */
    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)))
    {

    }*/

    /* Configure the GPIO for 'nCS' as output and set high */
   // MAP_GPIOPinTypeGPIOOutput(nCS_PORT, nCS_PIN);
    //MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);
    GPIO_setAsOutputPin(nCS_PORT, nCS_PIN);
    GPIO_setOutputHighOnPin(nCS_PORT, nCS_PIN);

    /* Configure the GPIO for 'nRST_nPWDN' as output and set high */
    //MAP_GPIOPinTypeGPIOOutput(nRESET_PORT, nRESET_PIN);
    //MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, nRESET_PIN);
    GPIO_setAsOutputPin(nRESET_PORT, nRESET_PIN);
    GPIO_setOutputHighOnPin(nRESET_PORT, nRESET_PIN);

    /* Configure the GPIO for 'nRST_nPWDN' as output and set high */
    //MAP_GPIOPinTypeGPIOOutput(nPWDN_PORT, nPWDN_PIN);
    //MAP_GPIOPinWrite(nPWDN_PORT, nPWDN_PIN, nPWDN_PIN);
    GPIO_setAsOutputPin(nPWDN_PORT, nPWDN_PIN);
    GPIO_setOutputHighOnPin(nPWDN_PORT, nPWDN_PIN);

    /* Enable the clock to the GPIO Port P and wait for it to be ready */
    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)))
    {

    }*/

    /* Configure the GPIO for 'nDRDY' as input with falling edge interrupt */
    //MAP_GPIOPinTypeGPIOInput(nDRDY_PORT, nDRDY_PIN);
    //MAP_GPIOIntTypeSet(nDRDY_PORT, nDRDY_PIN, GPIO_FALLING_EDGE);
    //MAP_GPIOIntEnable(nDRDY_PORT, nDRDY_PIN);
    //MAP_IntEnable(nDRDY_INT);
    GPIO_setAsInputPin(nDRDY_PORT, nDRDY_PIN);
    GPIO_clearInterrupt(nDRDY_PORT, nDRDY_PIN);
    GPIO_selectInterruptEdge(nDRDY_PORT, nDRDY_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt(nDRDY_PORT, nDRDY_PIN);

    /* Enable the clock to the GPIO Port P and wait for it to be ready */
    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {

    }*/

    /* Configure the GPIO for 'START' as output and set high */
    //MAP_GPIOPinTypeGPIOOutput(START_PORT, START_PIN);
    //MAP_GPIOPinWrite(START_PORT, START_PIN, START_PIN);
    GPIO_setAsOutputPin(START_PORT, START_PIN);
    GPIO_setOutputHighOnPin(START_PORT, START_PIN);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}



//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************

/**
 * \fn void GPIOP5_IRQHandler(void)
 * \brief Interrupt handler for /DRDY falling edge interrupt
 */
void GPIOP5_IRQHandler(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    //TODO: Rename or register this interrupt function with your processor

    // Possible ways to handle this interrupt:
    //If you decide to read data here, you may want to disable other interrupts to avoid partial data reads.

    //In this example we set a flag and exit the interrupt routine. In the main program loop, your application can example
    // all state flags and decide which state (operation) to perform next.

    /* Get the interrupt status from the GPIO and clear the status */
    uint32_t getIntStatus = GPIO_getInterruptStatus(nDRDY_PORT, nDRDY_PIN);

    if((getIntStatus & nDRDY_PIN) == nDRDY_PIN)
    {
        GPIO_clearInterrupt(nDRDY_PORT, nDRDY_PIN);

        flag_nDRDY_INTERRUPT = true;
    }
}


//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************

/**
 * \fn bool getCS(void)
 * \brief Returns the state of the "/CS" GPIO pin
 * \return boolean (false=low, true=high)
 */
bool getCS(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    return (bool) GPIO_getInputPinValue(nCS_PORT, nCS_PIN);
}

/**
 * \fn bool getPWDN(void)
 * \brief Returns the state of the "/PWDN" GPIO pin
 * \return boolean (false=low, true=high)
 */
bool getPWDN(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    return (bool) GPIO_getInputPinValue(nPWDN_PORT, nPWDN_PIN);
}

/**
 * \fn bool getSTART(void)
 * \brief Returns the state of the "START" GPIO pin
 * \return boolean (false=low, true=high)
 */
bool getSTART(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    return (bool) GPIO_getInputPinValue(START_PORT, START_PIN);
}

/**
 * \fn bool getRESET(void)
 * \brief Returns the state of the "/RESET" GPIO pin
 * \return boolean (false=low, true=high)
 */
bool getRESET(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    return (bool) GPIO_getInputPinValue(nRESET_PORT, nRESET_PIN);
}

/**
 * \fn void setCS(bool state)
 * \brief Sets the state of the "/CS" GPIO pin
 * \param state boolean indicating which state to set the CS pin (0=low, 1=high)
 */
void setCS(bool state)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    //uint8_t value = (uint8_t) (state ? nCS_PIN : 0);
    if(state)
        GPIO_setOutputHighOnPin(nCS_PORT, nCS_PIN);
    else
        GPIO_setOutputLowOnPin(nCS_PORT, nCS_PIN);
    //MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, value);

    // td(SCCS) & td(CSSC) delay
    delay_ns(80);
}

/**
 * \fn void setPWDN(bool state)
 * \brief Sets the state of the "/PWDN" GPIO pin
 * \param state boolean indicating which state to set the /PWDN pin (0=low, 1=high)
 */
void setPWDN(bool state)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    //uint8_t value = (uint8_t) (state ? nPWDN_PIN : 0);
    if(state)
        GPIO_setOutputHighOnPin(nPWDN_PORT, nPWDN_PIN);
    else
        GPIO_setOutputLowOnPin(nPWDN_PORT, nPWDN_PIN);
    //MAP_GPIOPinWrite(nPWDN_PORT, nPWDN_PIN, value);

    // Minimum nPWDN width: 2 tCLKs
    delay_ns(125);

    // Reset register array when powering down
    if(!state){
        restoreRegisterDefaults();
    }
}

/**
 * \fn void setRESET(bool state)
 * \brief Sets the state of the "/RESET" GPIO pin
 * \param state boolean indicating which state to set the CS pin (0=low, 1=high)
 */
void setRESET(bool state)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    //uint8_t value = (uint8_t) (state ? nRESET_PIN : 0);
    if(state)
        GPIO_setOutputHighOnPin(nRESET_PORT, nRESET_PIN);
    else
        GPIO_setOutputLowOnPin(nRESET_PORT, nRESET_PIN);

    //MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, value);
}

/**
 * \fn void setSTART(bool state)
 * \brief Sets the state of the "START" GPIO pin
 * \param state boolean indicating which state to set the START pin (0=low, 1=high)
 */
void setSTART(bool state)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    //uint8_t value = (uint8_t) (state ? START_PIN : 0);
    if(state)
        GPIO_setOutputHighOnPin(START_PORT, START_PIN);
    else
        GPIO_setOutputLowOnPin(START_PORT, START_PIN);

    //MAP_GPIOPinWrite(START_PORT, START_PIN, value);

    // Minimum START width ~4 tCLKs
    // REFERENCE: https://e2e.ti.com/support/data-converters/f/73/p/431463/1543880
    delay_ns(250);
}

/**
 * \fn void toggleRESET(void)
 * \brief Toggles the /RESET pin
 */
void toggleRESET(void)
{
    /* --- TODO: INSERT YOUR CODE HERE --- */
    GPIO_setOutputLowOnPin(nRESET_PORT, nRESET_PIN);

    // Minimum nRESET width: 2 tCLKs
    delay_ns(125);

    GPIO_setOutputHighOnPin(nRESET_PORT, nRESET_PIN);
}


/**
 * \fn bool waitForDRDYinterrupt(uint32_t timeout_ms)
 * \brief Waits for a nDRDY interrupt or until a timeout condition occurs
 * \param timeout_ms number of milliseconds to allow until a timeout
 * \return Returns true if nDRDY interrupt occurred before the timeout
 */
bool waitForDRDYinterrupt(uint32_t timeout_ms)
{
    /* --- TODO: INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

    // Convert ms to # of loop iterations or use a timer
    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupts
    //IntMasterEnable();
    __enable_interrupt();

    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    return (timeout > 0);           // Did a nDRDY interrupt occur?
}



//****************************************************************************
//
// SPI initialization
//
//****************************************************************************

/**
 * \fn void initSPI(void)
 * \brief Configures the MCU's SPI peripheral
 */
void InitSPI(void)
{
    uint8_t returnValue = 0x00;
    // USCI B:
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN1);
    //GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN2);

    //Initialize Master
    USCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = SPICLK;
    param.msbFirst = USCI_B_SPI_MSB_FIRST;
    param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    returnValue =  USCI_B_SPI_initMaster(USCI_B0_BASE, &param);

    while(STATUS_FAIL == returnValue) //Wait for debugging

    //Enable SPI module
    //USCI_B_SPI_enable(USCI_B0_BASE);

    //Enable Receive interrupt
    __no_operation();
    USCI_B_SPI_clearInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);
    UCB0CTL1 &= ~(UCSWRST);
    USCI_B_SPI_enableInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);
    /*
    UCB0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCB0CTL0 |= UCMSB + UCMST + UCSYNC;
    UCB0CTL1 |= UCSSEL_2;                     // SMCLK
    UCB0BR0 |= 0x20;                          // /2
    UCB0BR1 = 0;                              //
    //UCB0MCTL = 0;                             // No modulation must be cleared for SPI
    UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCB0IE |= UCRXIE;// Enable USCI0 RX interrupt
*/
}



//****************************************************************************
//
// SPI helper functions
//
//****************************************************************************

/**
 * \fn void SPI_SendReceive(uint8_t *DataTx, uint8_t *DataRx, uint8_t byteLength)
 * \brief Sends SPI commands to ADC and returns a response in array format
 * \param *DataTx array of SPI data to send to DIN pin
 * \param *DataRx array of SPI data that will be received from DOUT pin
 * \param byteLength number of bytes to send/receive on the SPI
 */
void spiSendReceiveArrays(uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength)
{
    /*  --- TODO: INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     *
     */

    /* Set the nCS pin LOW */
    setCS(LOW);

    /* Remove any residual or old data from the receive FIFO */
    //uint32_t junk;
    //while (SSIDataGetNonBlocking(SSI3_BASE, &junk));

    /* SSI TX & RX */
    uint8_t i;
    for (i = 0; i < byteLength; i++)
    {
        DataRx[i] = spiSendReceiveByte(DataTx[i]);
    }

    /* Set the nCS pin HIGH */
    setCS(HIGH);
}


/**
 * \fn uint8_t spiSendReceiveByte(uint8_t dataTx)
 * \brief Sends SPI command to ADC and returns a response, one byte at a time.
 * \param dataTx data to send over SPI
 * \return Returns SPI response byte
 */
uint8_t spiSendReceiveByte(uint8_t dataTx)
{
    /*  --- TODO: INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    /* Remove any residual or old data from the receive FIFO */
    //uint32_t junk;
    //while (SSIDataGetNonBlocking(SSI3_BASE, &junk));

    /* SSI TX & RX */
    uint8_t dataRx;
    USCI_B_SPI_transmitData(USCI_B0_BASE, dataTx);
    dataRx=USCI_B_SPI_receiveData(USCI_B0_BASE);
    //MAP_SSIDataPut(SSI3_BASE, dataTx);
    //MAP_SSIDataGet(SSI3_BASE, &dataRx);

    return dataRx;
}
