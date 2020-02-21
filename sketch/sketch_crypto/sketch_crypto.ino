#include <Crypto.h>
#include <CryptoLW.h>
#include <EAX.h>
#include <AES.h>
#include <string.h>
#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

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

void setup()
{
    Serial.begin(115200);

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

}

void loop()
{
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
