#ifndef power_config_h
#define power_config_h

#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define CYCLES_SOMMEIL 1
#define BATTERY_PIN_VCC 0

// volatile int f_wdt=1;

// INSERER DANS LA DEFINITION DU SKETCH (avant setup)
//volatile int f_wdt=1;
//// const byte sleep_cycles=CYCLES_SOMMEIL; // Nombre de cycles de sommeil (par reveil watchdog)
//// byte current_sleep_count=0; // Cycles actuels
//ISR(WDT_vect)
//{
//  if(f_wdt == 0)
//  {
//    f_wdt=1;
//  }
//}


// INSERER DANS LE setup() du arduino
//  /* Clear the reset flag. */
//  MCUSR &= ~(1<<WDRF);
//  /* In order to change WDE or the prescaler, we need to
//   * set WDCE (This will allow updates for 4 clock cycles).
//   */
//  WDTCSR |= (1<<WDCE) | (1<<WDE);
//  /* set new watchdog timeout prescaler value */
//  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
//  /* Enable the WD interrupt (note no reset). */
//  WDTCSR |= _BV(WDIE);


// Methode pour lire le voltage d'alimentation de l'Arduino
class ArduinoPower
{
  public:
    void deepSleep();
    long readVcc();
    unsigned int lireVoltageBatterie();

  private:
    byte _battery_pin = BATTERY_PIN_VCC; // Defaut est VCC
    byte _current_sleep_count = 0; // Cycles actuels
    const byte _sleep_cycles=CYCLES_SOMMEIL; // Nombre de cycles de sommeil (par reveil watchdog)
    
};

#endif
//
// END OF FILE
//