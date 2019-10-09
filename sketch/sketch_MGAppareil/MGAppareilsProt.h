#ifndef mgAppareilsProt_config_h
#define mgAppareilsProt_config_h

#include <Arduino.h>
#include <RF24Mesh.h>
#include "Config.h"

#define NO_TEMP -32768
#define NO_PRESSURE 0xFF
#define NO_HUMIDITY 0XFF

#define PAYLOAD_TAILLE_SIMPLE 24
#define PAYLOAD_TAILLE_DOUBLE 48

#define MSG_TYPE_REQUETE_DHCP 0x1
#define MSG_TYPE_REPONSE_DHCP 0x2

#define MSG_TYPE_LECTURES_COMBINEES 0x101
#define MSG_TYPE_LECTURE_TH 0x102
#define MSG_TYPE_LECTURE_TP 0x103
#define MSG_TYPE_LECTURE_POWER 0x104
#define MSG_TYPE_LECTURE_ONEWIRE 0x105

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

class FournisseurLectureOneWire {
  public:
    virtual byte* adresse(); // byte[8]
    virtual byte* data();  // byte[12]
};

class MGProtocoleV7 {

  public:
    MGProtocoleV7(const byte* uuid, RF24Mesh* mesh) {
      _uuid = uuid;
      _mesh = mesh;
    };

    byte lireReponseDhcp(byte* data);

    bool transmettreRequeteDhcp();
    bool transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets);

    // Paquets classe SenseursPassifs
    bool transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur);
    bool transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur);
    bool transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur);
    bool transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur);
    
//    bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4);
//    bool transmettrePaquetLecturePower(uint16_t noPaquet, uint32_t millivolt, byte reservePct, byte alerte);

  private:
    const byte* _uuid;
    RF24Mesh* _mesh;
    
    byte _buffer[48]; // Supporte 2 * 24 bytes (taille du plus grand payload)

    void ecrireUUID(byte* destination);

    // Transmet un paquet; il faut indiquer la taille du payload 
    // (PAYLOAD_TAILLE_SIMPLE ou PAYLOAD_TAILLE_DOUBLE)
    bool transmettrePaquet(byte taillePayload);

};


#endif
//
// END OF FILE
//
