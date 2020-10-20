#ifndef mgAppareilsProt_config_h
#define mgAppareilsProt_config_h

#include <Arduino.h>
// #include <RF24Mesh.h>
#include <EEPROM.h>
#include "Config.h"

// Cryptographie, utilise ISR(WDT_vect)
#define WATCHDOG_INITD
#include <Crypto.h>
#include <CryptoLW.h>
// #include <EAX.h>
// #include <AES.h>
#include <Acorn128.h>
#include <Curve25519.h>
#include <RNG.h>

#define CLE_PRIVEE_ETAT_PRETE 1

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

#define MSG_TYPE_CLE_LOCALE_1   0x0004
#define MSG_TYPE_CLE_LOCALE_2   0x0005
#define MSG_TYPE_CLE_DISTANTE_1 0x0006
#define MSG_TYPE_CLE_DISTANTE_2 0x0007
#define MSG_TYPE_NOUVELLE_CLE   0x0008
#define MSG_TYPE_REPONSE_ACK    0x0009
#define MSG_TYPE_ECHANGE_IV     0x000A

// Paquets de debut (00xx) et de fin (FFxx)
#define MSG_TYPE_PAQUET0       0x0000
#define MSG_TYPE_PAQUET_INCONNU 0xF0F0
#define MSG_TYPE_PAQUET_IV     0xFFFE
#define MSG_TYPE_PAQUET_FIN    0xFFFF

// Paquets d'une transaction chiffree (01xx)
#define MSG_TYPE_LECTURES_COMBINEES 0x101
#define MSG_TYPE_LECTURE_TH 0x102
#define MSG_TYPE_LECTURE_TP 0x103
#define MSG_TYPE_LECTURE_POWER 0x104
#define MSG_TYPE_LECTURE_ONEWIRE 0x105
#define MSG_TYPE_LECTURE_ANTENNE 0x106

// Message chiffre - paquet 32 bytes tout inclus (02xx)
#define MSG_TYPE_LECTURE_TH_ANTENNE_POWER 0x0202
#define MSG_TYPE_LECTURE_TP_ANTENNE_POWER 0x0203


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
    MGProtocoleV9(RF24* radio, const byte* nodeId) {
      _nodeId = nodeId;
      _radio = radio;

      // Lire l'etat de la cle privee dans le EEPROM
      byte lectureEtatClePrivee;
      EEPROM.get(EEPROM_CLE_PRIVEE_ETAT, lectureEtatClePrivee);
      _clePriveePrete = lectureEtatClePrivee == CLE_PRIVEE_ETAT_PRETE;

    };

    // Implementation methodes FournisseurLectureAntenne
    byte pctSignal();
    byte forceEmetteur();
    byte canal();

    byte* getCleBuffer(); // Retourne le buffer pour la cle - utiliser pour setter la cle publique distante ou cle secrete
    byte* executerDh1();  // DH passe 1 pour generer cle privee. Retourne byte* vers cle publique.
    bool executerDh2();   // DH passe 2 pour extraire cle secrete. Retourne false si le processus a echoue.

    void activerCryptage();
    bool initCipher(byte* authData, byte authDataLen, byte* iv); // Reinitialise cipher avec iv et cle existants
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

    // Messages all-included, dependent du setup UUID, cle, iv prealables
    bool transmettreLectureTHAntennePower(FournisseurLectureTH* th, FournisseurLectureAntenne* antenne, FournisseurLecturePower* power);
    bool transmettreLectureTPAntennePower(FournisseurLectureTP* tp, FournisseurLectureAntenne* antenne, FournisseurLecturePower* power);
    
    // bool transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur);
    
    bool isTransmissionOk();
    bool isAckRecu();
    byte nombreCyclesAbortConsecutifs();  // Retourne le nombre de transmissions consecutives qui n'ont pas finit avec un ACK
    void resetNombreCyclesAbortConsecutifs();
    bool isClePriveePrete();

    void loop();

    // Recevoir paquet
    uint16_t recevoirPaquet(byte* buffer, byte bufferLen);  // Recoit un paquet, insere donnees dans le buffer au besoin. Retourne le type de paquet recu.
    
//    bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4);
//    bool transmettrePaquetLecturePower(uint16_t noPaquet, uint32_t millivolt, byte reservePct, byte alerte);

  private:
    const byte* _nodeId;
    RF24* _radio;

    // EAX<AES256> eax256;
    Acorn128 cipher;  
    
    StatTransmissions stats = {
      .forceSignalPct = 0,
      .nombreTransmissions = 0,
      .nombreErreurs = 0,
      .nombrePaquets = 0,
      .nombreCyclesAbortConsecutifs = 0
    };
    
    byte _cle[32];  // Buffer de 32 bytes pour stocker des cles (publique durant echange ed25519 et secrete une fois pairing complete)
    byte _iv[16];   // IV connu par le serveur (transmis et confirme - permet messages 0x02xx)
    
    bool _transmissionOk = false;  // Vrai si la derniere transmission s'est rendue correctement (ACK RF24 recu)
    bool _ackRecu = true;          // Faux si on attend un ACK pour une transmission
    bool _clePriveePrete;          // Vrai si la cle privee est deja generee
    bool _cryptageActif = false;   // Vrai si on utilise le cryptage

    void ecrireUUID(byte* destination);

    // Transmet un paquet; il faut indiquer la taille du payload 
    // (PAYLOAD_TAILLE_SIMPLE ou PAYLOAD_TAILLE_DOUBLE)
    bool transmettrePaquet(byte taillePayload, byte* buffer);
    bool transmettrePaquetIv(byte noPaquet, byte* iv);
    bool transmettrePaquetCrypte(byte taillePaquet, byte* buffer);
    bool transmettreMessageCrypte(byte taillePayload, byte* buffer);

    void setIvBuffer(byte* buffer);
    byte* getIvBuffer();  // Retourne le buffer avec le IV confirme comme recu cote serveur - permet d'utilise message cryptes (one shot)

};


#endif
//
// END OF FILE
//
