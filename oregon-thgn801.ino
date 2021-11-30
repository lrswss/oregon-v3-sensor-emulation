/*
 * 
 * THGN801 emulation for Arduino Uno with FS1000A 433 RF-transmitter
 * 
 * (c) 2021 Lars Wessels <software@bytebox.org>
 *
 * Sketch to replace a (broken) THGN801 433MHz sensor sending temperature
 * and humidity readings using the Oregon V3 protocol. Tested with WMR200 
 * base station.
 * 
 * http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf
 *
 * Published under MIT license.
 * 
 */

#define RF433_TX_PIN 6  // output pin for 433MHz transmitter (e.g. FS1000A)
#define CHANNEL 1  // 1..10 for WMR200
#define THGN801_DATA_LEN 13
#define V3_PULSE_LENGHT_US 488  // data rate 1024Hz
#define V3_PULSE_SHORTEN_US 132  // 138us not reliable

#define DEBUG_PAYLOAD
#define DEBUG_LASTTX

// set THGN801's test values for temperature/humidity
// which are increased after each transmission
static float t_temp = -1.3;
static uint8_t t_hum = 25;

// lookup table to speed up CRC8
static uint8_t ccitt_table[256];


// calculate crc8 bytewise checksum using a CCITT lookup table
uint8_t crc_checksum_v3(uint8_t *payload, uint8_t len) {
    uint8_t crc = 0;
  
    // skip the V3 sync pattern '0xA'
    crc = ccitt_table[crc ^ (payload[3] & 0x0F)];
  
    // include bytes 4..[n-3]
    for (uint8_t i = 4; i < (len - 2); i++)
        crc = ccitt_table[crc ^ payload[i]];
  
    // nibbles need to be swapped (LSD)
    return ((crc & 0x0F) << 4) | ((crc & 0xF0) >> 4);
}


// create CCITT CRC8 lookup table
// based on http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html#ch4
void init_crc8_table() {
    uint8_t crc;
  
    for (uint16_t i = 0; i < 256; i++) { 
        crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);  // CCITT 0x07
        }
        ccitt_table[i] = crc;
    }
}


// calculate the Oregon V3 checksum
// based on https://github.com/merbanan/rtl_433/blob/master/src/devices/oregon_scientific.c
uint8_t oregon_checksum_v3(uint8_t *payload, int len) {
    uint8_t checksum;
  
    // remove sync byte '0xA' from first nibble of byte 4
    checksum = (payload[3] & 0xF);
  
    // bits 4..10
    for (uint8_t i = 4; i <= (len - 3); i++) {
      checksum += (payload[i] & 0xF) + ((payload[i] >> 4) & 0xF);
    }
  
    return ((checksum & 0x0F) << 4) | ((checksum & 0xF0) >> 4);
}


// send a single byte as series of manchester encoded RF pulses
// base station won't receive packets if timing is off
void manchester_encode_v3(uint8_t txByte, bool last) {
    static uint32_t txMicros = micros();
    uint8_t bitMask = 0; 

    // send 8 bits of data 
    for (uint8_t i = 0; i < 8; i++) {  

        // ensure equal distant bit pulses
        txMicros += (V3_PULSE_LENGHT_US * 2);
        if (txMicros - micros() < 0)
            delayMicroseconds((txMicros - micros()) * -1);
        else
            delayMicroseconds(txMicros - micros());

        // send bit 4..7 first followed by bit 0..3
        if (!bitMask) 
            bitMask = 0x10; // start with bit 4
        else if (bitMask == 0x80) 
            bitMask = 0x01; // jump from bit 7 to bit 0
        else
            bitMask <<= 1; // next bit
        
        //  high bit is encoded as as on-to-off and low bit as off-to-on transition
        digitalWrite(RF433_TX_PIN, ((txByte & bitMask) >= 1) ? HIGH : LOW);
        delayMicroseconds(V3_PULSE_LENGHT_US - V3_PULSE_SHORTEN_US);
        digitalWrite(RF433_TX_PIN, ((txByte & bitMask) >= 1) ? LOW : HIGH);
        
        // add extra delay after last low to high pulse
        // since there is no more data to come...
        if (((txByte & bitMask) == 0) && last)
            delayMicroseconds(V3_PULSE_LENGHT_US);
    }
}  


// returns payload for THGN801 outdoor sensor (temperature/humidity)
uint8_t *payload_thgn801(uint8_t rollingCode, float tempC, uint8_t hum) {
    static uint8_t payload[THGN801_DATA_LEN];
    uint16_t t10 = 0;

    // clear payload
    memset(payload, 0, THGN801_DATA_LEN);
  
    // set preamble with 1-bit pulses (24 bits) and sync nibble '0101'
    memset(payload, 0xFF, 3);  
    payload[3] = 0xA0;
  
    // nibbles 0..3: oregon sensor id (16 bits)
    payload[3] |= 0x0F; // THGN801 is 0xF824
    payload[4] = 0x82;
    payload[5] = 0x40;
  
    // nibble 4: seems to be channel 1..15 for THGN801 (WMR200 base station offers 1..10)
    payload[5] |= CHANNEL > 15 ? 15 : CHANNEL;
  
    // nibble 5..6 contains 16 bit rolling code (LSD), changes on sensor reset
    payload[6] = rollingCode;
  
    // nibble 7 is flag for battery status (4 bits)
    payload[7] = 0x80; // always ok (0x80), set to 0x40 for low battery

    // nibbles 8..13 encode sensor-specific data
    // THGN801: nibbles 8..11 temperature in degC as LSD with 0.1 precision (12 bits)
    t10 = abs(tempC * 10.001);  // .001 required to fix rounding issues
    payload[7] |= ((t10 % 10) & 0x0F);
    payload[8] = ((t10 / 10) % 10) << 4 | ((t10 / 100) & 0x0F);
    payload[9] = (tempC < 0.0) ? 0x80 : 0; // nibble 11 encodes temperature sign (0x80 => negative)    

    // THGN801: nibbles 12..13 encodes relative humidity (8 bits)
    payload[9] |= (hum % 10) & 0x0F;
    payload[10] = ((hum / 10) % 10) << 4;  // use of nibble 14 unknown

    // post preambles with checksums
    // sensor specific checksum for nibbles 0..14
    payload[11] = oregon_checksum_v3(payload, THGN801_DATA_LEN);

    // crc8 checksum for V3 protocol
    payload[12] = crc_checksum_v3(payload, THGN801_DATA_LEN);

    return payload;  
}


// send manchester encoded payload for a Oregon V3 sensor
void send_data_v3(uint8_t *payload, uint8_t len) {
#ifdef DEBUG_PAYLOAD
    Serial.print(millis());
    Serial.print(": V3 Payload [ ");
    for (uint8_t i = 0; i < len; i++) {
        if (payload[i] < 16)
            Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
#endif
    digitalWrite(RF433_TX_PIN, LOW);
    delay(1);    
    for (uint8_t i = 0; i < len; i++)
        manchester_encode_v3(payload[i], (i+1 == len));
    digitalWrite(RF433_TX_PIN, LOW);
}


void setup() {  
    pinMode(RF433_TX_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(250);   
    Serial.print(F("\n\nStarting Oregon V3 sensor emulator...\n\n"));
    
    randomSeed(analogRead(1)); // random seed for rolling codes
    init_crc8_table(); // create in memory cr8-ccitt lookup table
}


void loop() {
    static uint32_t seconds = 0;
    static uint32_t lastMillis = 0, lastTX = 0;
    static bool thgn801_pending = true;
    static uint8_t thgn801_rc = 0;

    if (thgn801_rc == 0) {
        thgn801_rc = random(0x01, 0xFE);
        Serial.print(millis());
        Serial.print(": Rolling code for THGN801 is ");
        Serial.println(((thgn801_rc & 0x0F) << 4) | ((thgn801_rc & 0xF0) >> 4));
    }
    
    if (millis() - lastMillis > 1000) {
        digitalWrite(LED_BUILTIN, LOW);
        lastMillis = millis();
        seconds++;
        Serial.print(millis());
        Serial.println(": Waiting for next TX...");
        thgn801_pending = true;
    }

    // Oregon V3 base station is expecting to receive
    // temperature/humidity readings every 53 secs.
    if (seconds % 53 == 0 && thgn801_pending) {
        thgn801_pending = false;
        digitalWrite(LED_BUILTIN, HIGH);

        Serial.print(millis());
        Serial.print(": THGN801 (Temperature: ");
        Serial.print(t_temp);
        Serial.print(" °C, Humidity: ");
        Serial.print(t_hum);
        Serial.print(" %");
#ifdef DEBUG_LASTTX
        if (lastTX > 0) {
            Serial.print(", Last TX ");
            Serial.print(millis() - lastTX);
            Serial.print(" ms ago");
            lastTX = millis();
        }
#endif
        Serial.println(")");

        // create and send off sensor payload
        send_data_v3(payload_thgn801(thgn801_rc, t_temp, t_hum), THGN801_DATA_LEN); 

        // increase readings for testing
        t_temp += 0.1;
        if (t_temp  > 45)
            t_temp = 19.0;
        t_hum += 1;
        if (t_hum > 95)
            t_hum = 30;
    }
}
