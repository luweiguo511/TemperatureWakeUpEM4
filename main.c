/**************************************************************************//**
 * @file
 * @brief This project demonstrates the ability for a pin to wake the device
 * from EM4. See Readme.txt for more information.
 * @version 0.0.1
 ******************************************************************************
 * @section License
 * <b>Copyright 2018 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "bsp.h"
#include "tempdrv.h"

#define EM4WU_PIN           BSP_GPIO_PB1_PIN
#define EM4WU_PORT          BSP_GPIO_PB1_PORT
#define EM4WU_PIN1           10
#define EM4WU_PORT1          gpioPortE
#define EM4WU_EM4WUEN_NUM   (2)                       // PA0 is EM4WUEN pin 0
#define EM4WU_EM4WUEN_MASK  (1 << EM4WU_EM4WUEN_NUM)

#define EM4_RSTCAUSE_MASK	RMU_RSTCAUSE_EM4RST

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void) 
{
  // Configure GPIO pins
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB1 as input and EM4 wake-up source
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(EM4WU_PORT1, EM4WU_PIN1, gpioModeInputPullFilter, 1);
  GPIO_EM4EnablePinWakeup(EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);
  //GPIO_EM4EnablePinWakeup(0x204 << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);
  GPIO_EM4EnablePinWakeup(0x200 << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);
  // Configure PD15 as an output
  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 0);

  // Configure LED0 and LED1 as output
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief Toggle STK LEDs forever
 *****************************************************************************/
void toggleLEDs(void) 
{
  while (1) {
    GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
    GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);

    // Arbitrary delay between toggles
    for (volatile uint32_t delay = 0; delay < 0xFFFFF; delay++);
  }
}
void toggleLED1(void)
{
  while (1) {
    //GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
    GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);

    // Arbitrary delay between toggles
    for (volatile uint32_t delay = 0; delay < 0xFFFFF; delay++);
  }
}
void toggleLED2(void)
{
  while (1) {
    GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
    //GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);

    // Arbitrary delay between toggles
    for (volatile uint32_t delay = 0; delay < 0xFFFFF; delay++);
  }
}
uint32_t causeresult = 0;
/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void) 
{

	EMU_EM4Init_TypeDef em4init  = EMU_EM4INIT_DEFAULT;
	CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

  // Chip errata
  CHIP_Init();
  // Enable clock to the LE modules interface
  CMU_ClockEnable(cmuClock_HFLE, true);
  //CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  // Initializations
  initGPIO();
  em4init.em4State = emuEM4Hibernate;
  em4init.retainLfxo = false;
  em4init.retainLfrco = false;
  em4init.retainUlfrco = false;
  em4init.pinRetentionMode = emuPinRetentionEm4Exit;
  EMU_EM4Init(&em4init);


  //if (RMU_ResetCauseGet() & RMU_RSTCAUSE_EM4RST) {
	CMU_LFXOInit(&lfxoInit);

  // Get the last Reset Cause
  uint32_t rstCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  // If the last Reset was due to leaving EM4, toggle LEDs. Else, enter EM4
  if (rstCause == EM4_RSTCAUSE_MASK)
  {
	  causeresult = GPIO_EM4GetPinWakeupCause();
	  if (causeresult&0x2000000)
	  {
		  toggleLED1();
	  }
	  if (causeresult&0x40000)
		  toggleLED2();
    //toggleLEDs();

  }
  else
  {
    for (volatile uint32_t delay = 0; delay < 0xFFF; delay++);
    EMU_EnterEM4();
  }

  // Will never get here!
  while (1);
}


#define TEMP_EMU_LOW       60
#define TEMP_EMU_HIGH      80
//#define TEMPDRV_LIMIT_HIGH 1
//#define TEMPDRV_LIMIT_LOW 1
#define EMU_CUSTOM_IRQ_HANDLER true

typedef union
{
  uint32_t l;
  uint16_t w[2];
  uint8_t  b[4];
} ulong;

struct EmuTemp
{
  uint8_t  devInfoCalTemp;
  uint8_t  devInfoEmuTemp;
  uint16_t val;
} emuTemp;

void emuResetTemplimitsEnableWakeup(void);
void prepareAndEnterEM4H(int8_t temp, TEMPDRV_LimitType_t limit);
void exitEM4HCallback(int8_t temp, TEMPDRV_LimitType_t limit);

void prepareAndEnterEM4H(int8_t temp, TEMPDRV_LimitType_t limit)
{
  emuResetTemplimitsEnableWakeup();

  // Register low wake-up callback
  TEMPDRV_RegisterCallback(TEMP_EMU_LOW, TEMPDRV_LIMIT_LOW, exitEM4HCallback);

  // Clear the reset cause in order to catch EM4H reset
  // For some reason program won't register EM4 reset if you don't clear the reset clause
  RMU_ResetCauseClear();

  SLEEP_ForceSleepInEM4();
}

// A dummy callback. It shouldn't really reach it, because MCU exits EM4H with reset
void exitEM4HCallback(int8_t temp, TEMPDRV_LimitType_t limit)
{
  emuResetTemplimitsEnableWakeup();

  TEMPDRV_RegisterCallback(TEMP_EMU_HIGH, TEMPDRV_LIMIT_HIGH, prepareAndEnterEM4H);
}


void initEmuTemp()
{
  TEMPDRV_Init();

  emuResetTemplimitsEnableWakeup();

  if (TEMPDRV_GetTemp() < TEMP_EMU_HIGH)
  {
    TEMPDRV_RegisterCallback(TEMP_EMU_HIGH, TEMPDRV_LIMIT_HIGH, prepareAndEnterEM4H);
  }
  else
  {
    prepareAndEnterEM4H(TEMPDRV_GetTemp(), TEMPDRV_LIMIT_HIGH);
  }
}

void emuResetTemplimitsEnableWakeup()
{
  ulong aux;

  aux.b[0] = 0x00;
  aux.b[1] = 0xff;
  aux.b[2] = 0x01;

  EMU->TEMPLIMITS = aux.l;

  EMU->IFC = (EMU_IFC_TEMPLOW | EMU_IFC_TEMPHIGH);
  NVIC_ClearPendingIRQ(EMU_IRQn);
}


void i1nitEmuTemp1(void)
{
  ulong aux;

  emuTemp.devInfoCalTemp = DEVINFO->CAL >> 16;
  emuTemp.devInfoEmuTemp = DEVINFO->EMUTEMP & 0xFF;


  // TEMPLOW (0.278 for EM01)
  aux.b[0] = (uint8_t) (emuTemp.devInfoEmuTemp - 1.0f * (TEMP_EMU_HIGH - emuTemp.devInfoCalTemp) /
      (0.278f + emuTemp.devInfoEmuTemp * 0.01f) + 0.5f);

  // TEMPHIGH (0.268 for EM234)
  aux.b[1] = (uint8_t) (emuTemp.devInfoEmuTemp - 1.0f * (TEMP_EMU_LOW - emuTemp.devInfoCalTemp) /
      (0.268f + emuTemp.devInfoEmuTemp * 0.01f) + 0.5f);

  // EM4WUEN = 1: Allow device to wake-up from low or high temperature from EM4H
  aux.b[2] = 0x01;

  EMU->TEMPLIMITS = aux.l;


  // Interrupts
  EMU->IFC = (EMU_IFC_TEMPLOW | EMU_IFC_TEMPHIGH);

  // Allow only one of the EMU_TEMP interrupts at a time, depending on temperature
  //emuTemp.val = getEmuTemp();
  if (emuTemp.val < TEMP_EMU_HIGH) {
    EMU->IEN |= EMU_IEN_TEMPLOW;
    EMU->IEN &= ~EMU_IEN_TEMPHIGH;
  } else {
    EMU->IEN |= EMU_IEN_TEMPHIGH;
    EMU->IEN &= ~EMU_IEN_TEMPLOW;
  }

  NVIC_ClearPendingIRQ(EMU_IRQn);
  NVIC_SetPriority(EMU_IRQn, 3);
  NVIC_EnableIRQ(EMU_IRQn);
}

