#ifndef millegrilles_appareils_paquet_h
#define millegrilles_appareils_paquet_h
// Structures de communication entre les appareils MilleGrilles via NRF24Mesh

#include <Arduino.h>


#define MG_CHANNEL 87
#define PAQUET_LEN 24

class MilleGrillesAppareilsPaquet {

  public:
    byte transmettrePaquet0(uint16_t type);
    byte transmettrePaquet(uint32_t noPaquet, byte payload);

};

#endif
//
// END OF FILE
//
