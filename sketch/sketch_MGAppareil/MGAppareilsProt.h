#ifndef mgAppareilsProt_config_h
#define mgAppareilsProt_config_h

#include <Arduino.h>
#include <RF24Mesh.h>

#define VERSION 0x7

#define MESH_MASTER_ID 0    
#define NO_TEMP -32768
#define NO_PRESSURE 0xFF
#define NO_HUMIDITY 0XFF

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 7

class MGProtocoleV7 {

  public:
    MGProtocoleV7(const byte* uuid, RF24Mesh* mesh) {
      _uuid = uuid;
      _mesh = mesh;
    };
    
    void transmettrePaquets();

  private:
    const byte* _uuid;
    RF24Mesh* _mesh;
    
    byte _buffer[24];

    void ecrireUUID(byte* destination);

    bool transmettrePaquet();
    bool transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets);

    // Paquets classe SenseursPassifs
    bool transmettrePaquetLectureTHP(uint16_t noPaquet, int temperature, uint16_t humidite, uint16_t pression);
    bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4);
};

#endif
//
// END OF FILE
//
