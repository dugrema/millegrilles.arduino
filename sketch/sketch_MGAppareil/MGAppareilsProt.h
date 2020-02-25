#ifndef mgAppareilsProt_config_h
#define mgAppareilsProt_config_h

#include <Arduino.h>
// #include <RF24Mesh.h>
#include "Config.h"

// Cryptographie, utilise ISR(WDT_vect)
#define WATCHDOG_INITD
#include <Crypto.h>
#include <CryptoLW.h>
#include <EAX.h>
#include <AES.h>
#include <Curve25519.h>

#define NO_TEMP -32768
#define NO_PRESSURE 0xFF
#define NO_HUMIDITY 0XFF

#define PAYLOAD_TAILLE_SIMPLE 32

#define LIMITE_RETRANSMISSION 5

#define PAIRING_PAS_INIT 0xFF
#define PAIRING_CLE_PRIVEE_PRETE 0x01
#define PAIRING_ADRESSE_DHCP_ASSIGNEE 0x02
#define PAIRING_SERVEUR_CLE 0x03

#define MSG_TYPE_REQUETE_DHCP 0x1
#define MSG_TYPE_REPONSE_DHCP 0x2
#define MSG_TYPE_BEACON_DHCP  0x3

#define MSG_TYPE_CLE_LOCALE_1   0x4
#define MSG_TYPE_CLE_LOCALE_2   0x5
#define MSG_TYPE_CLE_DISTANTE_1 0x6
#define MSG_TYPE_CLE_DISTANTE_2 0x7
#define MSG_TYPE_NOUVELLE_CLE   0x8
#define MSG_TYPE_REPONSE_ACK    0x9

#define MSG_TYPE_PAQUET0       0x0000
#define MSG_TYPE_PAQUET_INCONNU 0xF0F0
#define MSG_TYPE_PAQUET_IV     0xFFFE
#define MSG_TYPE_PAQUET_FIN    0xFFFF

#define MSG_TYPE_LECTURES_COMBINEES 0x101
#define MSG_TYPE_LECTURE_TH 0x102
#define MSG_TYPE_LECTURE_TP 0x103
#define MSG_TYPE_LECTURE_POWER 0x104
#define MSG_TYPE_LECTURE_ONEWIRE 0x105
#define MSG_TYPE_LECTURE_ANTENNE 0x106

struct StatTransmissions {
  uint8_t forceSignalPct;
  uint16_t nombreTransmissions;
  uint16_t nombreErreurs;
  byte nombrePaquets;
  byte nombreCyclesAbortConsecutifs;
};

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

class FournisseurLectureAntenne {
  public:
    virtual byte pctSignal();     // byte
    virtual byte forceEmetteur(); // byte
    virtual byte canal();         // byte
};

class MGProtocoleV9 : public FournisseurLectureAntenne {

  public:
    MGProtocoleV9(const byte* uuid, RF24* radio, const byte* nodeId) {
      _uuid = uuid;
      _nodeId = nodeId;
      _radio = radio;
    };

    // Implementation methodes FournisseurLectureAntenne
    byte pctSignal();
    byte forceEmetteur();
    byte canal();

    byte* getCleBuffer(); // Retourne le buffer pour la cle - utiliser pour setter la cle publique distante ou cle secrete
    byte* getIvBuffer(); // Retourne le buffer pour la cle - utiliser pour setter la cle publique distante ou cle secrete
    byte* executerDh1();  // DH passe 1 pour generer cle privee. Retourne byte* vers cle publique.
    bool executerDh2();   // DH passe 2 pour extraire cle secrete. Retourne false si le processus a echoue.

    void activerCryptage();
    bool initCipher(byte* authData, byte authDataLen); // Reinitialise cipher avec iv et cle existants
    void encryptBuffer(byte* buffer, byte bufferLen);  // Encrypte buffer, resultat mis dans meme buffer
    void decryptBuffer(byte* buffer, byte bufferLen);  // Decrypte buffer, resultat mis dans meme buffer
    void computeTag(byte* outputTag);                  // Retourne le tag (hash) du message

    void lireBeaconDhcp(byte* data, byte* adresseServeur);
    byte lireReponseDhcp(byte* data, byte* adresseNoeud);

    bool transmettreRequeteDhcp();
    byte transmettrePaquet0(uint16_t typeMessage);
    bool transmettrePaquetFin(byte noPaquet);
    bool transmettrePaquetsClePublique(uint16_t noPaquet);

    // Paquets classe SenseursPassifs
    bool transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur);
    bool transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur);
    bool transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur);
    bool transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur);
    bool transmettrePaquetLectureAntenne(uint16_t noPaquet, FournisseurLectureAntenne* fournisseur);
    bool isTransmissionOk();
    bool isAckRecu();
    byte nombreCyclesAbortConsecutifs();  // Retourne le nombre de transmissions consecutives qui n'ont pas finit avec un ACK
    void resetNombreCyclesAbortConsecutifs();

    void loop();

    // Recevoir paquet
    uint16_t recevoirPaquet(byte* buffer, byte bufferLen);  // Recoit un paquet, insere donnees dans le buffer au besoin. Retourne le type de paquet recu.
    
//    bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4);
//    bool transmettrePaquetLecturePower(uint16_t noPaquet, uint32_t millivolt, byte reservePct, byte alerte);

  private:
    const byte* _uuid;
    const byte* _nodeId;
    RF24* _radio;
    EAX<AES256>* eax256 = 0x0;
    StatTransmissions stats = {
      .forceSignalPct = 0,
      .nombreTransmissions = 0,
      .nombreErreurs = 0,
      .nombrePaquets = 0,
      .nombreCyclesAbortConsecutifs = 0
    };
    
    // byte _buffer[32]; // 32 bytes, max pour RF24
    byte _cle[32];  // Buffer de 32 bytes pour stocker des cles (publique durant echange ed25519 et secrete une fois pairing complete)
    byte _iv[16];  // IV pour transmissions cryptees
    byte * _bufferTemp = 0x0;  // Byte* d'un buffer sur heap pour Ed25519, permet de conserver une valeur secondaire lorsque necesssaire (e.g. cle privee)
    bool _transmissionOk = false;  // Vrai si la derniere transmission s'est rendue correctement (ACK RF24 recu)
    bool _ackRecu = true;          // Faux si on attend un ACK pour une transmission

    void ecrireUUID(byte* destination);

    // Transmet un paquet; il faut indiquer la taille du payload 
    // (PAYLOAD_TAILLE_SIMPLE ou PAYLOAD_TAILLE_DOUBLE)
    bool transmettrePaquet(byte taillePayload, byte* buffer);
    bool transmettrePaquetIv(byte noPaquet);
    bool transmettrePaquetCrypte(byte taillePaquet, byte* buffer);
};


#endif
//
// END OF FILE
//
