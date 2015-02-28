#include "si5351.h"
#include "Wire.h"

/*
 * Si5351 WSPR Transmitter
 *
 * Copyright (C) 2014 Joseph Consugar (joesugar@erols.com)
 *
 * Basic controller for the Classie WSPR transceiver.
 *
 * This program is placed in the public domain and is distributed
 * in the hope that it will be useful but WITHOUT ANY WARRANTY, 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR 
 * A PARTICULAR PURPOSE.
 */
 
#define uint32  unsigned long
#define uint16  unsigned int
#define uint8   unsigned char
#define int32   long
#define int16   int
#define int8    char

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE ~FALSE
#endif

#define CALLSIGN "KC3XM "
#define LOCATOR  "FM19"
#define POWER    30

/* Timer periods.
 */
#define BIT_PERIOD_NUMER    8192
#define BIT_PERIOD_DENOM    12000
#define BIT_PERIOD_MULT     12

#define RESET_DELAY             3000
#define TRANSMIT_PERIOD_NUMBER  120000
#define TRANSMIT_PERIOD_DENOM   1
#define TRANSMIT_PERCENTAGE     25

#define RX_FREQ             10125000
#define TX_FREQ             10140200
//#define TX_FREQ_SHIFT       (BIT_PERIOD_DENOM / BIT_PERIOD_NUMBER)
#define TX_FREQ_SHIFT       2

#define ONE_SECOND          1000
#define NUMBER_OF_MSG_BITS  162

#define WAIT_FOR_TRANSMIT_TIMEOUT   0
#define WAIT_FOR_TRANSMIT_START     1
#define WAIT_FOR_TRANSMIT_COMPLETE  2

#define POWER_UP            0x00
#define POWER_DOWN          0x04

#define VCONTROL            9
#define LED                 13

#define ENABLE_TRANSMIT     digitalWrite(VCONTROL, LOW)
#define ENABLE_TRANSMIT_LED digitalWrite(LED, HIGH)
#define ENABLE_RECEIVE      digitalWrite(VCONTROL, HIGH)
#define ENABLE_RECEIVE_LED  digitalWrite(LED, LOW)

/* Variable declarations.
 */
uint32 lastTime = 0;
uint32 currentTime = 0;
uint32 deltaTime = 0;

uint32 bitTimer = BIT_PERIOD_NUMER;
uint32 deltaBitTimer = 0;

uint32 transmitTimer = 0;
uint32 deltaTransmitTimer = 0;
uint16 transmitAcc = 0;

uint32 transmitStartTimer = 0;
uint8  transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;

uint32 frequencyMin = 0;
uint8 bitIndex = 0;

/* WSPR data
 * KC3XM FM19 10
 */
/*
uint8 wsprData[] = {
    3, 3, 2, 2, 0, 0, 0, 0, 3, 0, 0, 2, 3, 3, 3, 0, 0, 2,
    3, 0, 2, 1, 2, 1, 1, 1, 1, 0, 2, 0, 0, 2, 2, 0, 3, 2, 
    2, 1, 0, 3, 2, 2, 2, 2, 2, 2, 3, 0, 3, 3, 0, 2, 3, 1, 
    0, 3, 0, 0, 0, 1, 3, 2, 1, 0, 0, 0, 2, 1, 1, 0, 3, 0, 
    1, 0, 3, 2, 1, 2, 0, 1, 0, 0, 1, 2, 3, 3, 2, 0, 0, 1,
    3, 0, 3, 0, 1, 2, 2, 0, 1, 2, 0, 0, 0, 2, 3, 0, 0, 3, 
    0, 2, 1, 3, 1, 2, 3, 1, 2, 0, 1, 3, 0, 1, 2, 2, 0, 3, 
    1, 1, 2, 0, 2, 0, 0, 1, 0, 3, 2, 2, 3, 3, 0, 0, 0, 2, 
    0, 2, 0, 3, 1, 2, 1, 0, 3, 3, 2, 2, 2, 3, 3, 2, 0, 2
};
*/

/* WSPR data
 * KC3XM FM19 23
 */
/*
uint8 wsprData[] = {
    3, 3, 2, 2, 0, 0, 0, 0, 3, 2, 0, 0, 3, 1, 3, 2, 0, 2,
    3, 2, 2, 3, 2, 3, 1, 1, 1, 0, 2, 2, 0, 2, 2, 0, 3, 2, 
    2, 3, 0, 1, 2, 0, 2, 2, 2, 0, 3, 0, 3, 1, 0, 0, 3, 3, 
    0, 1, 0, 2, 0, 1, 3, 0, 1, 2, 0, 0, 2, 1, 1, 2, 3, 0, 
    1, 2, 3, 2, 1, 2, 0, 3, 0, 0, 1, 2, 3, 3, 2, 2, 0, 1,
    3, 2, 3, 2, 1, 0, 2, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 3, 
    0, 2, 1, 3, 1, 0, 3, 3, 2, 0, 1, 1, 0, 3, 2, 0, 0, 3, 
    1, 3, 2, 0, 2, 0, 0, 1, 0, 3, 2, 2, 3, 1, 0, 0, 0, 0, 
    0, 2, 0, 3, 1, 2, 1, 2, 3, 3, 2, 0, 2, 3, 3, 0, 0, 2
};
*/

/* WSPR data
 * KC3XM FM19 30
 */
uint8 wsprData[] = {
    3, 3, 2, 0, 0, 0, 0, 0, 3, 0, 0, 2, 3, 1, 3, 0, 0, 2, 
    3, 2, 2, 3, 2, 3, 1, 3, 1, 2, 2, 0, 0, 0, 2, 0, 3, 2, 
    2, 1, 0, 3, 2, 2, 2, 0, 2, 2, 3, 2, 3, 1, 0, 0, 3, 3, 
    0, 3, 0, 0, 0, 1, 3, 2, 1, 0, 0, 0, 2, 1, 1, 0, 3, 2, 
    1, 2, 3, 2, 1, 0, 0, 3, 0, 0, 1, 2, 3, 3, 2, 2, 0, 3,
    3, 2, 3, 2, 1, 2, 2, 0, 1, 2, 0, 0, 0, 0, 3, 0, 0, 1, 
    0, 2, 1, 1, 1, 0, 3, 1, 2, 2, 1, 1, 0, 1, 2, 2, 0, 1, 
    1, 1, 2, 0, 2, 2, 0, 3, 0, 1, 2, 0, 3, 1, 0, 2, 0, 2, 
    0, 2, 0, 3, 1, 0, 1, 2, 3, 1, 2, 0, 2, 3, 3, 2, 0, 2
};

/* Si5351 setup routine.
 */   
Si5351 si5351;

void setup()
{
    /* Initialize variables.
     */
    lastTime = millis();
    transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
    transmitTimer = TRANSMIT_PERIOD_NUMBER - RESET_DELAY;
    transmitAcc = 0;     
    
    /* Configure the A2D
     */
    configureA2D();
    
    /* Initialize the output pins.
     */
    pinMode(VCONTROL, OUTPUT);
    pinMode(LED, OUTPUT);
    ENABLE_RECEIVE;
    ENABLE_RECEIVE_LED;
    
    /* Initialize the Si5351
     */
    si5351.init(SI5351_CRYSTAL_LOAD_8PF);
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
    
    /* Enable/disable the clocks.
     */
    si5351.set_correction(-391);
    si5351.clock_enable(SI5351_CLK0, 0);
    si5351.clock_enable(SI5351_CLK1, 0);
    si5351.clock_enable(SI5351_CLK2, 1);
    
    /* Start in receive mode.
     */
    SetFrequency(RX_FREQ, POWER_DOWN);
}

/* Interrupt servicing routine.
 * Called when a new ADC value is ready.
 */
uint8 newData = 0;
ISR(ADC_vect) 
{
    /* Get value from A0.
     */
    newData = ADCH;
}

/* WSPR main loop.
 */
void loop()
{
    /* Update the values used to track time.
     */
    currentTime = millis();
    deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    /* Update the timer values.
     *
     * The idea is to decrement time timer by an amount corresponding
     * to the amount of time that's passed.  When the timer goes < 0
     * it's time to process the timer event.
     *
     * Rather than wait for the timer to go < 0 we compare the current
     * value to the value by which it is to be decremented.  If the 
     * current value is < decrement value add the period to the timer.
     * That way the value will stay > 0 when the subtraction is done.
     *
     * Also, we preserve the remainder in the timer.  The result is that
     * while we may not get exact single bits it will be correct on
     * the average.
     */

    uint8 transmitTimedOut = UpdateTransmitTimer(&transmitTimer, deltaTime);
    if (WAIT_FOR_TRANSMIT_TIMEOUT == transmitState)
    {
        /* Waiting for a move from one 2 minute period to the next.
         */
        if (transmitTimedOut)
        {
            /* Moved into the next transmit/received slot.  See if it's supposed
             * to be for transmit or receive.
             */
            uint8 transmitFlag = SetTransmitFlag(&transmitAcc);
            if (transmitFlag)
            {
                /* This period is to be a transmit period so set up the 1 second
                 * delay before transmission.
                 */
                ENABLE_TRANSMIT_LED;
                SetFrequencyList(TX_FREQ);
                transmitStartTimer = ONE_SECOND;
                transmitState = WAIT_FOR_TRANSMIT_START;
            }
            else
            {
                /* This period is to be a receive period so set up as such with
                 * no change in state.
                 */
                ENABLE_RECEIVE_LED;
                SetFrequency(RX_FREQ, POWER_DOWN);
            }
        }
    }
    else if (WAIT_FOR_TRANSMIT_START == transmitState)
    {
        /* Within the first second of a 2 minute transmit period.  This is here because
         * the standard calls for transmissions to start 1 second into the period.
         */
        uint8 transmitStartTimedOut = UpdateTransmitStartTimer(&transmitStartTimer, deltaTime);
        if (transmitStartTimedOut)
        {
            /* End of 1 second delay.  Begin transmission.
             */
            bitIndex = 1;
            bitTimer = BIT_PERIOD_NUMER;
            SetFrequency(frequencyList(wsprData[0]), POWER_UP);
            transmitState = WAIT_FOR_TRANSMIT_COMPLETE;
        }
    }
    else if (WAIT_FOR_TRANSMIT_COMPLETE == transmitState)
    {
        /* You are within a transmit period.
         */
        uint8 bitTimedOut = UpdateBitTimer(&bitTimer, deltaTime);
        if (bitTimedOut)
        {
            /* Time to move to the next bit.
             */
            if (bitIndex <= NUMBER_OF_MSG_BITS)
            {
                /* Still sending bits so set the frequency and update the index.
                 */
                SetFrequency(frequencyList(wsprData[bitIndex]), POWER_UP);
                bitIndex += 1;
            }
            else
            {
                /* Done sending bits so move to the next state.
                 */
                SetFrequency(RX_FREQ, POWER_DOWN);
                transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
            }
        }
    }
}

/* Configure the A2D
 */
void configureA2D()
{
    /* Configure for using the A2D
     * Disable the interrupts.
     */
    cli();
    
    /* Continuous sampling of analog pin 0
     */
    ADCSRA = 0;
    ADCSRB = 0;
    
    /* Set the reference voltage.
     */
    ADMUX |= (1 << REFS0); //set reference voltage
    
    /* Left align the ADC value- so we can read highest 
     * 8 bits from ADCH register only
     */
    ADMUX |= (1 << ADLAR);
  
    /* Set ADC clock with 32 prescaler- 8mHz/32=250kHz
     */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS0); 
    
    /* Enable auto trigger
     */
    ADCSRA |= (1 << ADATE); 
    
    /* Enable interrupts when measurement complete
     */
    ADCSRA |= (1 << ADIE); 
    
    /* Enable ADC
     */
    ADCSRA |= (1 << ADEN);
    
    /* Start ADC measurements
     */
    ADCSRA |= (1 << ADSC);
  
    /* Enable interrupts
     */
    sei();
}
    
/* Update the transmit start timer.
 */
uint8 UpdateTransmitStartTimer(uint32 *transmitStartTimer, uint32 deltaStartTimer)
{
    uint8 startTimerTimedOut = (*transmitStartTimer <= deltaStartTimer) ? TRUE : FALSE;
    if (startTimerTimedOut)
        *transmitStartTimer = *transmitStartTimer + ONE_SECOND;
    *transmitStartTimer = *transmitStartTimer - deltaStartTimer;
    return startTimerTimedOut;
}

/* Update the transmit timer value
 */
uint8 UpdateTransmitTimer(uint32 *transmitTimer, uint32 deltaTransmitTimer)
{
    uint8 transmitTimedOut = (*transmitTimer <= deltaTransmitTimer) ? TRUE : FALSE;
    if (transmitTimedOut)
        *transmitTimer = *transmitTimer + TRANSMIT_PERIOD_NUMBER;
    *transmitTimer = *transmitTimer - deltaTransmitTimer;
    return transmitTimedOut;
}

/* Update the bit timer value.
 */
uint8 UpdateBitTimer(uint32 *bitTimer, uint32 deltaBitTimer)
{
    /* The change in the bit timer involves a multiplier because the timer
     * accumulator is scaled.  The period of a bit is (8192 / 12000).
     * This gives a condition for the timer of:
     *   acc + delta(sec) > (8192 / 12000).
     * To eliminate the fraction multiply through by 12000:
     *   acc + 12000 * delta(sec) > 8192 
     * delta(sec) = millis() / 1000.  Substituting, we get the condition:
     *   acc + 12 * millis() > 8192.
     * We actually do a decrement from the accumulator instead of an increment
     * but the idea still holds.
     */
    deltaBitTimer = BIT_PERIOD_MULT * deltaTime;
    uint8 bitTimedOut = (*bitTimer <= deltaBitTimer) ? TRUE : FALSE;
    if (bitTimedOut)
        *bitTimer += BIT_PERIOD_NUMER;
    *bitTimer = *bitTimer - deltaBitTimer;
    return bitTimedOut;
}

/* Update the transmit accumulator and set the transmit flag.
 */
uint8 SetTransmitFlag(uint16 *transmitAcc)
{
    uint8 transmitFlag;

    /* Check to see if the next period should be for transmitting.
     */
    *transmitAcc += TRANSMIT_PERCENTAGE;
    transmitFlag = (*transmitAcc >= 100);
    if (*transmitAcc >= 100)
        *transmitAcc -= 100;
    return transmitFlag;
}

/* Initialize the frequency list.
 */
void SetFrequencyList(uint32 frequency)
{
    frequencyMin = frequency - TX_FREQ_SHIFT - (TX_FREQ_SHIFT / 2);
}

/* Return the WSPR frequency for the specified index.
 */
uint32 frequencyList(uint8 index)
{
    return frequencyMin + index * TX_FREQ_SHIFT;
}

/* Set the DDS frequency.
 */
void SetFrequency(uint32 frequency, int8 power_up_down)
{
    /* Set the output frequency.
     */
    si5351.set_freq(frequency, 0, SI5351_CLK2); 

    /* Apply power to/from the amplifier.
     */
    if (power_up_down == POWER_UP)
        ENABLE_TRANSMIT;
    else
        ENABLE_RECEIVE;
}
