#include "Power.h"
#include <Arduino.h>

bool ArduinoPower::isAlimentationSecteur() {
  // Bypass pour tester comportement batterie en developpeemnt
  #if defined(MG_DEV_TEST_BATTERIE) || defined(SENSEUR_ALIMENTATION_VCC) || defined(SENSEUR_ALIMENTATION_ANALOG)
    return false;
  #endif
  
  return _typeAlimentation == ALIMENTATION_SECTEUR;
}

long ArduinoPower::readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  return result; // Vcc in millivolts
};

void ArduinoPower::lireVoltageBatterie() {

  long voltage = 0;

  voltage = readVcc();
  #ifdef SENSEUR_ALIMENTATION_ANALOG
    int sensorValue = analogRead(_battery_pin);
    voltage = map(sensorValue, 0, 1023, 0, voltage);
  #endif
  _lectureVcc = uint32_t(voltage);

  _calculerReservePct();
}

void ArduinoPower::singleCycleSleep() {
  // set_sleep_mode(SLEEP_MODE_ADC);
  // Le power saving est maximal - le premier byte sur le UART est perdu
  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  _current_sleep_count = 1;  // Compter nombre de cycles de sleep

  sleep(1);

//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  setPrescalerMax();
//  wdt_reset(); // Reset watchdog
//  sleep_enable();
//  sleep_mode();
//      
//  /* The program will continue from here after the WDT timeout*/
//  sleep_disable(); /* First thing to do is disable sleep. */
//
//  resetPrescaler();
//  
//  /* Re-enable the peripherals. */
//  power_all_enable();

}

void ArduinoPower::deepSleep(bool* wakeUp) {
  // set_sleep_mode(SLEEP_MODE_ADC);
  // Le power saving est maximal - le premier byte sur le UART est perdu
  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//
//  setPrescalerMax();
//  
//  wdt_reset(); // Reset watchdog

  sleep(CYCLES_SOMMEIL - _current_sleep_count);

//  while( _current_sleep_count++ < _sleep_cycles) { // && !*wakeUp) {
//    // sleep_enable();
//    // wdt_reset(); // Reset watchdog
//    
//    /* Now enter sleep mode. */
//    sleep_mode();
//  }
      
//  /* The program will continue from here after the WDT timeout*/
//  sleep_disable(); /* First thing to do is disable sleep. */
//
//  resetPrescaler();
//  
//  /* Re-enable the peripherals. */
//  power_all_enable();

  _current_sleep_count = 0; // Reset sleep cycles
  
}

uint32_t ArduinoPower::millivolt() {
  return _lectureVcc;
}

void ArduinoPower::_calculerReservePct() {
  
  // On fait un certain nombre de lectures pour confirmer le type d'alimentation
  // Une fois le nombre de confirmations atteint, le type ne changera plus durant l'execution (jusqu'a un reset)
  if (_nbLecturesVerificationTypeCourant < ALIMENTATION_NB_CONFIRMATIONS) {
    
    // Detecter le type d'alimentation
    byte typeDetecte = ALIMENTATION_INCONNU;
    if (_lectureVcc >= 3150 && _lectureVcc <= 3425) {
      typeDetecte = ALIMENTATION_SECTEUR;
      #ifdef LOGGING_DEV
        Serial.print(F("Alimentation secteur "));
      #endif
    } else if(_lectureVcc > 3425) {
      typeDetecte = ALIMENTATION_BATT_LITHIUM;
      #ifdef LOGGING_DEV
        Serial.print(F("Alimentation lithium "));
      #endif
    } else {
      typeDetecte = ALIMENTATION_BATT_AA;
      #ifdef LOGGING_DEV
        Serial.print(F("Alimentation AA "));
      #endif
    }

    #ifdef LOGGING_DEV
      Serial.println(_lectureVcc);
    #endif

    // Augmenter le compte du nombre de confirmations
    if( _typeAlimentation == ALIMENTATION_INCONNU ) {
      // Set tentativement le type d'alimentation
      _typeAlimentation = typeDetecte;
      _nbLecturesVerificationTypeCourant = 1;
    } else if(_typeAlimentation == typeDetecte) {
      _nbLecturesVerificationTypeCourant++;
      #ifdef LOGGING_DEV
        Serial.print(F("Confirmation type "));
        Serial.print(_typeAlimentation);
        Serial.print(F(" nb confirmations: "));
        Serial.println(_nbLecturesVerificationTypeCourant);
      #endif
    } else {
      _typeAlimentation = ALIMENTATION_INCONNU;
    }
    
  }
  
  // Detecter le type d'alimentation
  long reserve = 0;
  if ( _typeAlimentation == ALIMENTATION_SECTEUR ) {
  
    // Mode alimentation secteur
    reserve = 100;

  } else if( _typeAlimentation == ALIMENTATION_BATT_LITHIUM ) {
    
    // Mode lithium
    if(_lectureVcc > 4200) {
      reserve = 100;
    } else if(_lectureVcc > 3950) {
      reserve = map(_lectureVcc, 3950, 4200, 80, 100);
    } else if(_lectureVcc > 3600) {
      reserve = map(_lectureVcc, 3600, 3950, 20, 80);
    } else if(_lectureVcc > 2700) {
      reserve = map(_lectureVcc, 2700, 3600, 0, 20);
    } else {
      reserve = 0;
    }
    
  } else if ( _typeAlimentation == ALIMENTATION_BATT_AA ) {
    
    // Mode AA
    reserve = map(_lectureVcc, 2700, 3150, 0, 100);
  
  } else {
    
    reserve = 0;
  
  }

  if(reserve >= 0 && reserve <= 100) {
    _reserve = (byte)reserve;
  } else {
    _reserve = 101; // Erreur lecture
  }
  
}


byte ArduinoPower::reservePct() {
  return _reserve;
}

byte ArduinoPower::alerte() {
  byte alerte = 0;

  if( _typeAlimentation == ALIMENTATION_BATT_LITHIUM ) {
    if(_lectureVcc < 3000) {
      alerte = 2;
    } else  if(_lectureVcc < 3500) {
      alerte = 1;
    }
  } else if ( _typeAlimentation == ALIMENTATION_BATT_AA ) {
    if(_lectureVcc < 2740) {
      alerte = 2;
    } else if(_lectureVcc < 2775) {
      alerte = 1;
    }
  }

  return alerte;
}

void ArduinoPower::setPrescalerMax() {
    /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}

void ArduinoPower::resetPrescaler() {
    /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 0;  // Reset a 15ms
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}


void ArduinoPower::sleep(byte cycles) {
  byte sleepCount = 0;

  // Le power saving est maximal - le premier byte sur le UART est perdu
  setPrescalerMax();

  MCUSR = 0; // clear various "reset" flags
  wdt_reset(); // Reset watchdog
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts (); // timed sequence follows

  // turn off brown‐out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);
  interrupts (); // guarantees next instruction executed
  
  while( sleepCount++ < cycles) {
    /* Now enter sleep mode. */
    sleep_mode();
  }
      
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  resetPrescaler();
  
  /* Re-enable the peripherals. */
  power_all_enable();
}
