#include "Power.h"
#include <Arduino.h>

bool ArduinoPower::isAlimentationSecteur() {
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
  if( _battery_pin != BATTERY_PIN_VCC ) {
    int sensorValue = analogRead(_battery_pin);
    voltage = map(sensorValue, 0, 1023, 0, voltage);
  }
  _lectureVcc = uint32_t(voltage);
}

void ArduinoPower::deepSleep() {
    // set_sleep_mode(SLEEP_MODE_ADC);
  // Le power saving est maximal - le premier byte sur le UART est perdu
  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  while( _current_sleep_count++ < _sleep_cycles ) {
    sleep_enable();

    /* Now enter sleep mode. */
    sleep_mode();
  }
      
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
      
  /* Re-enable the peripherals. */
  power_all_enable();

  _current_sleep_count = 0; // Reset sleep cycles
  
}

uint32_t ArduinoPower::millivolt() {
  return _lectureVcc;
}

byte ArduinoPower::reservePct() {
  byte reserve = 100; // Retourne type erreur

  // On fait un certain nombre de lectures pour confirmer le type d'alimentation
  // Une fois le nombre de confirmations atteint, le type ne changera plus durant l'execution (jusqu'a un reset)
  if (_nbLecturesVerificationTypeCourant < ALIMENTATION_NB_CONFIRMATIONS) {
    
    // Detecter le type d'alimentation
    byte typeDetecte = ALIMENTATION_INCONNU;
    if (_lectureVcc >= 3150 && _lectureVcc <= 3425) {
      typeDetecte = ALIMENTATION_SECTEUR;
    } else if(_lectureVcc > 3425) {
      typeDetecte = ALIMENTATION_BATT_LITHIUM;
    } else {
      typeDetecte = ALIMENTATION_BATT_AA;
    }

    // Augmenter le compte du nombre de confirmations
    if( _typeAlimentation == ALIMENTATION_INCONNU ) {
      // Set tentativement le type d'alimentation
      _typeAlimentation = typeDetecte;
      _nbLecturesVerificationTypeCourant = 1;
    } else if(_typeAlimentation == typeDetecte) {
      _nbLecturesVerificationTypeCourant++;
      Serial.print(F("Confirmation type "));
      Serial.print(_typeAlimentation);
      Serial.print(F(" nb confirmations: "));
      Serial.println(_nbLecturesVerificationTypeCourant);
    } else {
      _typeAlimentation = ALIMENTATION_INCONNU;
    }
    
  }
  
  // Detecter le type d'alimentation
  if ( _typeAlimentation == ALIMENTATION_SECTEUR ) {
  
    // Mode alimentation secteur
    reserve = 100;

  } else if( _typeAlimentation == ALIMENTATION_BATT_LITHIUM ) {
    
    // Mode lithium
    if(_lectureVcc > 3950) {
      reserve = map(3950, 4200, 80, 100, _lectureVcc);      
    } else if(_lectureVcc > 3700) {
      reserve = map(3700, 3950, 20, 80, _lectureVcc);      
    } else {
      reserve = map(2700, 3700, 0, 20, _lectureVcc);      
    }
    
  } else if ( _typeAlimentation == ALIMENTATION_BATT_AA ) {
    
    // Mode AA
    reserve = map(2700, 3150, 0, 100, _lectureVcc);
  
  } else {
    
    reserve = 0;
  
  }

  return reserve;
}

byte ArduinoPower::alerte() {
  byte alerte = 0;
  if(_lectureVcc > 3400 && _lectureVcc < 3500) {
    alerte = 1;
  } else if(_lectureVcc < 2750) {
    alerte = 1;
  }

  return alerte;
}



