/*!
 *  @file Adafruit_PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "PCA9685.h"
#include "pico/float.h"

#include <stdio.h>

//#define ENABLE_DEBUG_OUTPUT

PCA9685 get_pca9685_def(i2c_inst_t * i2c, uint8_t addr, uint32_t oscillator_freq) {
  return (PCA9685){.i2c = i2c, .addr = addr, .oscillator_freq = oscillator_freq};
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void pca9685_begin(PCA9685 * device, uint8_t prescale) {
  pca9685_reset(device);
  if (prescale) {
    //setExtClk(prescale); //need this?
  } else {
    // set a default frequency
    setPWMFreq(device, 1000);
  }
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void pca9685_reset(PCA9685 * device) {
  uint8_t buf[2] = {PCA9685_MODE1, MODE1_RESTART};
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// /*!
//  *  @brief  Puts board into sleep mode
//  */
// void Adafruit_PWMServoDriver::sleep() {
//   uint8_t awake = read8(PCA9685_MODE1);
//   uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
//   write8(PCA9685_MODE1, sleep);
//   delay(5); // wait until cycle ends for sleep to be active
// }

// /*!
//  *  @brief  Wakes board from sleep
//  */
// void Adafruit_PWMServoDriver::wakeup() {
//   uint8_t sleep = read8(PCA9685_MODE1);
//   uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
//   write8(PCA9685_MODE1, wakeup);
// }

// /*!
//  *  @brief  Sets EXTCLK pin to use the external clock
//  *  @param  prescale
//  *          Configures the prescale value to be used by the external clock
//  */
// void Adafruit_PWMServoDriver::setExtClk(uint8_t prescale) {
//   uint8_t oldmode = read8(PCA9685_MODE1);
//   uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
//   write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

//   // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
//   // use the external clock.
//   write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

//   write8(PCA9685_PRESCALE, prescale); // set the prescaler

//   delay(5);
//   // clear the SLEEP bit to start
//   write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print("Mode now 0x");
//   Serial.println(read8(PCA9685_MODE1), HEX);
// #endif
// }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void setPWMFreq(PCA9685 * device, float freq) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif
  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((device->oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: ");
  Serial.println(prescale);
#endif
  uint8_t buf[2];
  uint8_t mode_addr = PCA9685_MODE1;
  uint8_t oldmode;
  i2c_write_blocking(device->i2c, device->addr, &mode_addr, 1, true);
  i2c_read_blocking(device->i2c, device->addr, &oldmode, 1, false);

  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  //write8(PCA9685_MODE1, newmode);                             // go to sleep
  buf[0] = PCA9685_MODE1;
  buf[1] = newmode;
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
  //write8(PCA9685_PRESCALE, prescale); // set the prescaler
  buf[0] = PCA9685_PRESCALE;
  buf[1] = prescale;
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
  //write8(PCA9685_MODE1, oldmode);
  buf[0] = PCA9685_MODE1;
  buf[1] = oldmode;
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
  // This sets the MODE1 register to turn on auto increment.
  //write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
  buf[0] = PCA9685_MODE1;
  buf[1] = oldmode | MODE1_RESTART | MODE1_AI;
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false); 

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
    printf("prescale: %u\n", pca9685_read_prescale(device));

}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void pca9685_set_output_mode(PCA9685 * device, bool totempole) {
  uint8_t oldmode;
  uint8_t reg_addr = PCA9685_MODE2;
  i2c_write_blocking(device->i2c, device->addr, &reg_addr, 1, true);
  i2c_read_blocking(device->i2c, device->addr, &oldmode, 1, false);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  uint8_t buf[2] = {PCA9685_MODE2, newmode};
  i2c_write_blocking(device->i2c, device->addr, buf, 2, false);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t pca9685_read_prescale(PCA9685 * device) {
  uint8_t ret;
  uint8_t reg_addr = PCA9685_PRESCALE;
  i2c_write_blocking(device->i2c, device->addr, &reg_addr, 1, true);
  i2c_read_blocking(device->i2c, device->addr, &ret, 1, false);
  return ret;
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t getPWM(PCA9685 * device, uint8_t num) {
  uint8_t reg_addr = PCA9685_LED0_ON_L + 4 * num;
  uint8_t ret;
  i2c_write_blocking(device->i2c, device->addr, &reg_addr, 1, true);
  i2c_read_blocking(device->i2c, device->addr, &ret, 1, false);
  return ret;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 *  @return result from endTransmission
 */
void setPWM(PCA9685 * device, uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif
  uint8_t buf[5] = {PCA9685_LED0_ON_L + 4 * num,
                    (uint8_t)(on & 0xFFu),
                    (uint8_t)((on >> 8) & 0xFFu),
                    (uint8_t)(off & 0xFFu),
                    (uint8_t)((off >> 8) & 0xFFu)};
  i2c_write_blocking(device->i2c, device->addr, buf, 5, false);
}

// /*!
//  *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
//  * on/off tick placement and properly handles a zero value as completely off and
//  * 4095 as completely on.  Optional invert parameter supports inverting the
//  * pulse for sinking to ground.
//  *   @param  num One of the PWM output pins, from 0 to 15
//  *   @param  val The number of ticks out of 4096 to be active, should be a value
//  * from 0 to 4095 inclusive.
//  *   @param  invert If true, inverts the output, defaults to 'false'
//  */
// void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert) {
//   // Clamp value between 0 and 4095 inclusive.
//   val = min(val, (uint16_t)4095);
//   if (invert) {
//     if (val == 0) {
//       // Special value for signal fully on.
//       setPWM(num, 4096, 0);
//     } else if (val == 4095) {
//       // Special value for signal fully off.
//       setPWM(num, 0, 4096);
//     } else {
//       setPWM(num, 0, 4095 - val);
//     }
//   } else {
//     if (val == 4095) {
//       // Special value for signal fully on.
//       setPWM(num, 4096, 0);
//     } else if (val == 0) {
//       // Special value for signal fully off.
//       setPWM(num, 0, 4096);
//     } else {
//       setPWM(num, 0, val);
//     }
//   }
// }

// /*!
//  *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
//  * microseconds, output is not precise
//  *  @param  num One of the PWM output pins, from 0 to 15
//  *  @param  Microseconds The number of Microseconds to turn the PWM output ON
//  */
// void Adafruit_PWMServoDriver::writeMicroseconds(uint8_t num,
//                                                 uint16_t Microseconds) {
// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print("Setting PWM Via Microseconds on output");
//   Serial.print(num);
//   Serial.print(": ");
//   Serial.print(Microseconds);
//   Serial.println("->");
// #endif

//   double pulse = Microseconds;
//   double pulselength;
//   pulselength = 1000000; // 1,000,000 us per second

//   // Read prescale
//   uint16_t prescale = readPrescale();

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(prescale);
//   Serial.println(" PCA9685 chip prescale");
// #endif

//   // Calculate the pulse for PWM based on Equation 1 from the datasheet section
//   // 7.3.5
//   prescale += 1;
//   pulselength *= prescale;
//   pulselength /= _oscillator_freq;

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(pulselength);
//   Serial.println(" us per bit");
// #endif

//   pulse /= pulselength;

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(pulse);
//   Serial.println(" pulse for PWM");
// #endif

//   setPWM(num, 0, pulse);
// }

