#ifndef power_config_h
#define power_config_h

#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "Config.h"
#include "MGAppareilsProt.h"


#define ALIMENTATION_INCONNU 0
#define ALIMENTATION_SECTEUR 1
#define ALIMENTATION_BATT_AA 2
#define ALIMENTATION_BATT_LITHIUM 3
#define ALIMENTATION_NB_CONFIRMATIONS 5

#define BATTERY_PIN A4
#define SENSEUR_ALIMENTATION_VCC
// #define SENSEUR_ALIMENTATION_ANALOG

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
class ArduinoPower : public FournisseurLecturePower
{
  public:
    void singleCycleSleep();
    void deepSleep(bool* wakeUp);
    void lireVoltageBatterie();
    uint32_t millivolt();
    byte reservePct();
    byte alerte();
    bool isAlimentationSecteur();  // True si l'appareil n'est pas sur batterie

  private:
    byte _battery_pin = BATTERY_PIN;
    byte _current_sleep_count = 0; // Cycles actuels
    uint32_t _lectureVcc;
    byte _nbLecturesVerificationTypeCourant = 0; // Nombre de lectures qui confirment le type d'alimentation (secteur, batterie AA ou lithium)
    byte _typeAlimentation = ALIMENTATION_INCONNU;
    byte _reserve = 0;

    long readVcc();
    void _calculerReservePct();

    void setPrescalerMax();
    void resetPrescaler();

    void sleep(byte cycles);

};

#endif
//
// END OF FILE
//
