#include <Crypto.h>
#include <CryptoLW.h>
#include <EAX.h>
#include <AES.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

#include <RF24.h>

#include <RNG.h>
#include <TransistorNoiseSource.h>

#define MAX_PLAINTEXT_LEN 64

struct TestVector
{
    const char *name;
    uint8_t key[16];
    uint8_t plaintext[MAX_PLAINTEXT_LEN];
    uint8_t ciphertext[MAX_PLAINTEXT_LEN];
    uint8_t authdata[20];
    uint8_t iv[16];
    uint8_t tag[16];
    size_t authsize;
    size_t datasize;
    size_t tagsize;
    size_t ivsize;
};

// Test vectors for AES in EAX mode from Appendix G of:
// http://www.cs.ucdavis.edu/~rogaway/papers/eax.pdf
static TestVector const testVectorEAX1 PROGMEM = {
    .name        = "EAX #1",
    .key         = {0x23, 0x39, 0x52, 0xDE, 0xE4, 0xD5, 0xED, 0x5F,
                    0x9B, 0x9C, 0x6D, 0x6F, 0xF8, 0x0F, 0xF4, 0x78},
    .plaintext   = {0x00},
    .ciphertext  = {0x00},
    .authdata    = {0x6B, 0xFB, 0x91, 0x4F, 0xD0, 0x7E, 0xAE, 0x6B},
    .iv          = {0x62, 0xEC, 0x67, 0xF9, 0xC3, 0xA4, 0xA4, 0x07,
                    0xFC, 0xB2, 0xA8, 0xC4, 0x90, 0x31, 0xA8, 0xB3},
    .tag         = {0x34, 0xD1, 0xE2, 0xDB, 0x32, 0xEB, 0xA3, 0xAD,
                    0xD5, 0xE5, 0xE1, 0xA8, 0x33, 0x2E, 0x41, 0x4A},
    .authsize    = 8,
    .datasize    = 0,
    .tagsize     = 16,
    .ivsize      = 16
};

// TestVector testVector;

EAX<AES256> *eax256 = new EAX<AES256>();

bool testCipher_N(AuthenticatedCipher *cipher, const struct TestVector *test, size_t inc)
{
    size_t posn, len;
    uint8_t tag[16];
    byte buffer[128];

    crypto_feed_watchdog();

    cipher->clear();
    if (!cipher->setKey(test->key, cipher->keySize())) {
        Serial.print("setKey ");
        return false;
    }
    if (!cipher->setIV(test->iv, test->ivsize)) {
        Serial.print("setIV ");
        return false;
    }

    memset(buffer, 0xBA, sizeof(buffer));

    if (!inc)
        inc = 1;

    for (posn = 0; posn < test->authsize; posn += inc) {
        len = test->authsize - posn;
        if (len > inc)
            len = inc;
        cipher->addAuthData(test->authdata + posn, len);
    }

    for (posn = 0; posn < test->datasize; posn += inc) {
        len = test->datasize - posn;
        if (len > inc)
            len = inc;
        cipher->encrypt(buffer + posn, test->plaintext + posn, len);
    }

    if (memcmp(buffer, test->ciphertext, test->datasize) != 0) {
        Serial.print(buffer[0], HEX);
        Serial.print("->");
        Serial.print(test->ciphertext[0], HEX);
        return false;
    }

    cipher->computeTag(tag, sizeof(tag));
    Serial.print("Tag : ");
    printArray((byte*)&tag, sizeof(tag));
    Serial.println();
    if (memcmp(tag, test->tag, sizeof(tag)) != 0) {
        Serial.print("computed wrong tag ... ");
        return false;
    }

    cipher->setKey(test->key, cipher->keySize());
    cipher->setIV(test->iv, test->ivsize);

    for (posn = 0; posn < test->authsize; posn += inc) {
        len = test->authsize - posn;
        if (len > inc)
            len = inc;
        cipher->addAuthData(test->authdata + posn, len);
    }

    for (posn = 0; posn < test->datasize; posn += inc) {
        len = test->datasize - posn;
        if (len > inc)
            len = inc;
        cipher->decrypt(buffer + posn, test->ciphertext + posn, len);
    }

    if (memcmp(buffer, test->plaintext, test->datasize) != 0)
        return false;

    if (!cipher->checkTag(tag, sizeof(tag))) {
        Serial.print("tag did not check ... ");
        return false;
    }

    return true;
}

void testCipher(AuthenticatedCipher *cipher, const struct TestVector *test)
{
    bool ok;
    TestVector testVector;
    memcpy_P(&testVector, test, sizeof(TestVector));
    test = &testVector;

    Serial.print(test->name);
    Serial.print(" ... ");

    ok  = testCipher_N(cipher, test, test->datasize);
    ok &= testCipher_N(cipher, test, 1);
    ok &= testCipher_N(cipher, test, 2);
    ok &= testCipher_N(cipher, test, 5);
    ok &= testCipher_N(cipher, test, 8);
    ok &= testCipher_N(cipher, test, 13);
    ok &= testCipher_N(cipher, test, 16);

    if (ok)
        Serial.println("Passed");
    else
        Serial.println("Failed");
}

bool initCipher(AuthenticatedCipher *cipher, byte* key, byte* iv, byte* authData, byte authDataLen) {
    
    cipher->clear();
    
    if (!cipher->setKey(key, cipher->keySize())) {
        Serial.print("setKey ");
        return false;
    }

    if (!cipher->setIV(iv, 16)) {
        Serial.print("setIV ");
        return false;
    }

    cipher->addAuthData(authData, authDataLen);

}

void encryptBuffer(AuthenticatedCipher *cipher, byte* outputBuffer, byte* buffer, byte bufferLen) {
  cipher->encrypt(outputBuffer, buffer, bufferLen);
}

void decryptBuffer(AuthenticatedCipher *cipher, byte* outputBuffer, byte* buffer, byte bufferLen) {
  cipher->decrypt(outputBuffer, buffer, bufferLen);
}

void computeTag(AuthenticatedCipher *cipher, byte* outputTag) {
  cipher->computeTag(outputTag, 16);
  Serial.print("Tag : ");
  printArray(outputTag, 16);
}

TransistorNoiseSource noise(A2);

void printEEPROM() {
  Serial.println("Contenu EEPROM");
    byte compteurLigne = 0x0;

    Serial.print("    ");
    for(byte i=0; i<32; i++) {
      printHex(i);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("----");
    for(byte i=0; i<32; i++) {
      Serial.print("---");
    }

    byte valeur;
    for(int i=0; i<E2END+1; i++) {
      if(i % 32 == 0) {
        Serial.println();
        printHex(compteurLigne);
        compteurLigne += 2;
        Serial.print("| ");
      }
      EEPROM.get(i, valeur);
      printHex(valeur);
      Serial.print(" ");
    }

    Serial.println();
}


RF24 radio(7, 8);

void setup()
{
    Serial.begin(115200);

    // Initialize the random number generator.
    RNG.begin("Init data random, mettre UUID ici");
    RNG.setAutoSaveTime(120); // 2 heures

    // Add the noise source to the list of sources known to RNG.
    RNG.addNoiseSource(noise);

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  radio.begin();
  radio.setChannel(0x0c);
  radio.setAutoAck(true);
  radio.setRetries(15, 1);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_LOW);

    Serial.println();

    // EAX<AES256> *eax256;
  
    Serial.println("State Sizes:");
    Serial.print("EAX<AES256> ... ");
    Serial.println(sizeof(*eax256));
    Serial.println();

    Serial.println("Test Vectors:");
    // eax256 = new EAX<AES256>();
    testCipher(eax256, &testVectorEAX1);
    // delete eax256;

    Serial.println();

    printEEPROM();
    
}

byte contenuAuth[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};
byte contenuACrypter[24] = {0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xa, 0xb,
                            0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
                            0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21};
byte testKey[32] = {0x23, 0x39, 0x52, 0xDE, 0xE4, 0xD5, 0xED, 0x5F,
                    0x9B, 0x9C, 0x6D, 0x6F, 0xF8, 0x0F, 0xF4, 0x78,
                    0x23, 0x39, 0x52, 0xDE, 0xE4, 0xD5, 0xED, 0x5F,
                    0x9B, 0x9C, 0x6D, 0x6F, 0xF8, 0x0F, 0xF4, 0x78};
byte testIv[16] = {0x62, 0xEC, 0x67, 0xF9, 0xC3, 0xA4, 0xA4, 0x07,
                   0xFC, 0xB2, 0xA8, 0xC4, 0x90, 0x31, 0xA8, 0xB4};

byte bufferTest[24];
byte bufferDecrypteTest[24];
byte bufferTag[16];
byte bufferDecrypteTag[16];

void testCrypter() {
  // Crypter le message

    Serial.println("Test encryption ");
    Serial.print("Auth text : ");
    printArray((byte*)&contenuAuth, sizeof(contenuAuth));
    printArray((byte*)&contenuACrypter, sizeof(contenuACrypter));
    for(byte i=0; i < 3; i++) {
      byte positionArray = 8*i;
      Serial.print("Cipher text : ");
      printArray((byte*)&contenuACrypter + positionArray, 8);
    }
    
    Serial.print("Key: ");
    printArray((byte*)&testKey, eax256->keySize());
    Serial.print("IV: ");
    printArray((byte*)&testIv, sizeof(testIv));
    initCipher(eax256, (byte*)&testKey, (byte*)&testIv, (byte*)&contenuAuth, sizeof(contenuAuth));

    for(byte i=0; i < 3; i++) {
      byte positionArray = 8*i;
      encryptBuffer(eax256, (byte*)&bufferTest + positionArray, (byte*)&contenuACrypter + positionArray, 8);
      Serial.print("Buffer crypte : ");
      printArray((byte*)&bufferTest + positionArray, 8);
    }

    computeTag(eax256, (byte*)&bufferTag);

    // Decrypter le message
    initCipher(eax256, (byte*)&testKey, (byte*)&testIv, (byte*)&contenuAuth, sizeof(contenuAuth));

    for(byte i=0; i < 3; i++) {
      byte positionArray = 8*i;
      decryptBuffer(eax256, (byte*)&bufferDecrypteTest + positionArray, (byte*)&bufferTest + positionArray, 8);
      Serial.print("Buffer decrypte : ");
      printArray((byte*)&bufferDecrypteTest + positionArray, 8);
    }

    // computeTag(eax256, (byte*)&bufferDecrypteTag);

    bool checkTag = eax256->checkTag(&bufferTag, sizeof(bufferTag));
    if(checkTag) {
      Serial.println("Tag OK");
    } else {
      Serial.println("Tag invalide");
    }
}

bool calibrating = false;
bool testComplete = false;
bool entropyPlein = false;

void loop()
{

  // Track changes to the calibration state on the noise source.
  bool newCalibrating = noise.calibrating();
  if (newCalibrating != calibrating) {
      calibrating = newCalibrating;
      if (calibrating)
          Serial.println("calibrating");
  }
    
  // Perform regular housekeeping on the random number generator.
  RNG.loop();

  // Generate output whenever 32 bytes of entropy have been accumulated.
  // The first time through, we wait for 48 bytes for a full entropy pool.
  if ( ! testComplete && RNG.available(sizeof(testIv)) ) {
      RNG.rand(testIv, sizeof(testIv));
      // printArray(testIv, sizeof(testIv));

      testCrypter();
      testComplete = true;
  }

  if( ! entropyPlein && RNG.available(48)) {
    Serial.println("Entropy plein");
    RNG.save();
    Serial.print("Sauvegarde de 48 bytes dans EEPROM a ");
    Serial.print(E2END + 1 - 48);
    Serial.println();

    printEEPROM();
    
    entropyPlein = true;
  }

}

void printArray(byte* liste, int len) {
  byte valeur = 0;
  for(int i=0; i<len; i++){
    valeur = liste[i];
    printHex(valeur);
  }

  Serial.println();  
}

void printHex(byte val) {
  if(val < 16) {
    Serial.print(val < 16 ? "0" : "");
  }
  Serial.print(val, HEX);
}
