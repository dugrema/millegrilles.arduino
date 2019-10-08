#ifndef mgAppareilsProt_config_h
#define mgAppareilsProt_config_h

#include <Arduino.h>
#include <RF24Mesh.h>
#include "Config.h"

class FournisseurLectureTH {
  public:
    virtual int temperature();
    virtual uint16_t humidite();
};

class FournisseurLectureTP {
  public:
    virtual int temperature();
    virtual uint16_t pression();
};

class FournisseurLectureMillivolt {
  public:
    virtual uint32_t millivolt1();
    virtual uint32_t millivolt2();
    virtual uint32_t millivolt3();
    virtual uint32_t millivolt4();
};

class FournisseurLecturePower {
  public:
    virtual uint32_t millivolt();
    virtual byte reservePct();
    virtual byte alerte();
};

class MGProtocoleV7 {

  public:
    MGProtocoleV7(const byte* uuid, RF24Mesh* mesh) {
      _uuid = uuid;
      _mesh = mesh;
    };
    
    bool transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets);

    // Paquets classe SenseursPassifs
    bool transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur);
    bool transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur);
    bool transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur);
//    bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4);
//    bool transmettrePaquetLecturePower(uint16_t noPaquet, uint32_t millivolt, byte reservePct, byte alerte);

  private:
    const byte* _uuid;
    RF24Mesh* _mesh;
    
    byte _buffer[24];

    void ecrireUUID(byte* destination);

    bool transmettrePaquet();

};


#endif
//
// END OF FILE
//
