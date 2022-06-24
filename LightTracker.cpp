/*
 * This file is part of the LightTracker distribution
 *   (https://github.com/nuncio-bitis/LightTracker
 * Copyright (c) 2022 James P. Parziale.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * LightTracker.c
 *
 * Code to control a robotic arm using 4 light-dependent-resistors to move to
 * face the brightest light source.
 * 
 *  Created on: June 1, 2022
 *      Author: jparziale
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <libgen.h>
#include <sys/types.h>
#include <unistd.h>

#include <pigpio.h>

#include "ADS1115.h"

// Convenience macro to delay by milliseconds.
#ifndef delayMs
#define delayMs(ms) gpioDelay(ms * 1000)
#endif

//#define DEBUG_PRINT // @DEBUG
#ifdef DEBUG_PRINT
#define DPRINTF(fmt, args...) do{ printf(fmt, ## args); fflush(stdout); } while(0)
#else
#define DPRINTF(fmt, args...)
#endif // DEBUG_PRINT

// *****************************************************************************

// Flag used to terminate the program on an external signal.
static bool run = true;

static void SignalHandler(int sig);

// *****************************************************************************
// Servo utilities
// *****************************************************************************

// GPIO 12 (physical pin 32)
static const int cGpioPin_h = 12;
// GPIO 13 (physical pin 33)
static const int cGpioPin_v = 13;

static const int cMin     =  500;  // 5% duty cycle, left-most/top-most position
static const int cMid     = 1300;  // known middle position
static const int cMax     = 2100;  // 21% duty cycle, right-most position
static const int cMaxV    = (cMid + 100);  // bottom-most position
                                           // Broken servo, it also shouldn't need to go lower than this anyway.
static const int cParkPos = cMid;  // Park position = middle for both servos.

static const int cMinStepSize = 5;

static const int cStepDelay = 50; // ms

static void initServos(void)
{
    DPRINTF("Init:\n");
    DPRINTF("Min   %d\n", cMin);
    DPRINTF("Mid   %d\n", cMid);
    DPRINTF("Max   %d\n", cMax);
    DPRINTF("\n");

    DPRINTF("Park servos:\n");
    gpioServo(cGpioPin_h, cParkPos);
    DPRINTF("Position H:%d\n", cParkPos);
    delayMs(50);
    gpioServo(cGpioPin_v, cParkPos);
    DPRINTF("Position V:%d\n", cParkPos);
    delayMs(50);
    DPRINTF("\n");

    delayMs(250);
}

static void resetServos(void)
{
    printf("Reset\n");

    // Park the servos in their middle positions.
    gpioServo(cGpioPin_h, cParkPos);
    DPRINTF("Position H:%d\n", cParkPos);
    delayMs(50);
    gpioServo(cGpioPin_v, cParkPos);
    DPRINTF("Position V:%d\n", cParkPos);
    delayMs(50);
    DPRINTF("\n");

    delayMs(250);
}

static int setAzimuth(int az)
{
    int pos = az;
    bool change = false;

    // Constrain input
    if (pos < cMin)
        pos = cMin;
    else if (pos > cMax)
        pos = cMax;
    else
        change = true;

    if (change) gpioServo(cGpioPin_h, pos);
    return pos;
}

static int setElevation(int el)
{
    int pos = el;
    bool change = false;

    // Constrain input
    if (pos < cMin)
        pos = cMin;
    else if (pos > cMaxV)
        pos = cMaxV;
    else
        change = true;

    if (change) gpioServo(cGpioPin_v, pos);
    return pos;
}

// *****************************************************************************
// Light sensor utilities
// *****************************************************************************

static ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);

static const int cFilterLength = 8;

// Compensation ratio values (from experimentation)
static const double a0Compensation = 0.8; //1.06;  // TL
static const double a1Compensation = 1.05; //1.07;  // TR
static const double a2Compensation = 1.05; //1.06;  // BL
static const double a3Compensation = 1.10; //0.85;  // BR

// Compensated raw light values.
static double topLeft, topRight, bottomLeft, bottomRight;

// Computed directional light values.
static double top, bottom, left, right;

double ReadOutput(uint8_t chan)
{
    adc0.setMultiplexer(chan);
    adc0.triggerConversion();
    double filtered = 0.0;
    for (int i=0; i < cFilterLength; ++i)
    {
        double converted = adc0.getMilliVolts();;
        // Filter the value (average)
        filtered += (converted / cFilterLength);
    }
    return filtered;
}

static void ReadLightValues(void)
{
    double a0, a1, a2, a3;

    // Get light values, scaling up and compensating.
    a0 = ReadOutput(ADS1115_MUX_P0_NG);
    a1 = ReadOutput(ADS1115_MUX_P1_NG);
    a2 = ReadOutput(ADS1115_MUX_P2_NG);
    a3 = ReadOutput(ADS1115_MUX_P3_NG);

    topLeft     = a0 * a0Compensation;
    topRight    = a1 * a1Compensation;
    bottomLeft  = a2 * a2Compensation;
    bottomRight = a3 * a3Compensation;

    DPRINTF("[A0,A1,A2,A3]: [%6.2f, %6.2f, %6.2f, %6.2f]\n", a0, a1, a2, a3 );
    DPRINTF("[TL,TR,BL,BR]: [%6.2f, %6.2f, %6.2f, %6.2f]\n", topLeft, topRight, bottomLeft, bottomRight );

    // TODO Subtract shadow and/or ambient value from readings.
}

static void ComputeDirectionalValues(void)
{
    top    = (topLeft    + topRight)    / 2.0;
    bottom = (bottomLeft + bottomRight) / 2.0;
    left   = (topLeft    + bottomLeft)  / 2.0;
    right  = (topRight   + bottomRight) / 2.0;
    DPRINTF("[ T, B, L, R]: [%6.2f, %6.2f, %6.2f, %6.2f]\n", top, bottom, left, right);
}

static int ComputeAzimuthStep(void)
{
    // Position increases from left to right
    int az = (int)((right - left) + 0.5);
    return az;
}

static int ComputeElevationStep(void)
{
    // Position increases from top to bottom
    int el = (int)((bottom - top) + 0.5);
    return el;
}

// *****************************************************************************

int main(int argc, char *argv[])
{
    printf("\n%s: [%s] %s @ %d\n\n", basename(argv[0]), __FILE__, __FUNCTION__, __LINE__);

    // Must initialize pigpio GPIO utilities.
    if (gpioInitialise() < 0)
    {
        fprintf(stdout, "oops: %s\n", strerror(errno));
        return -1;
    }

    gpioSetSignalFunc(SIGINT, SignalHandler);
    gpioSetSignalFunc(SIGABRT, SignalHandler);

    // Initialize ADS1115 16 bit A/D chip
    // PGA = 000 : 6.144V
    // MODE = 1  : single-shot
    // DR = 111  : 860 SPS
    // Comparator disabled
    adc0.initialize();
    adc0.setGain(ADS1115_PGA_6P144);
    adc0.setRate(ADS1115_RATE_860);
    adc0.setMode(ADS1115_MODE_SINGLESHOT);
    adc0.setComparatorMode(false);

    // Initialize the servos and set them to their middle positions
    initServos();

    // Positions to be computed from values of light sensors.
    int hPos = cParkPos;
    int vPos = cParkPos;
    bool changed = false;

    while (run)
    {
        DPRINTF("\n");

        // Read light sensors and calculate next position
        ReadLightValues();
        ComputeDirectionalValues();
        int azStep = ComputeAzimuthStep();
        int elStep = ComputeElevationStep();
        DPRINTF("Step: [%d, %d]\n", azStep, elStep);

        if (abs(azStep) > cMinStepSize)
        {
            hPos += azStep;
            changed = true;
        }
        if (abs(elStep) > cMinStepSize)
        {
            vPos += elStep;
            changed = true;
        }
        if (changed)
        {
            DPRINTF("New Position H:%d V:%d\n", hPos, vPos);
            hPos = setAzimuth(hPos);
            vPos = setElevation(vPos);
            DPRINTF("Set Position H:%d V:%d\n", hPos, vPos);
            changed = false;
        }
        else
        {
            // Freeze position (stop PWM pulses) - saves power
            gpioPWM(cGpioPin_h, 0);
            gpioPWM(cGpioPin_v, 0);
        }

        delayMs(cStepDelay);
    }

    // Park the servos.
    resetServos();

    // Clean up and stop using GPIO utilities (pigpio call)
    gpioTerminate();

    return EXIT_SUCCESS;
}

// *****************************************************************************

void SignalHandler(int sig)
{
    fprintf(stderr, "\nSignal %d caught by %d\n", sig, getpid());
    run = false;
}

// *****************************************************************************
