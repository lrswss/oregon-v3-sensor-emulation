/*
 * 
 * PCR800 emulation for Arduino Uno with FS1000A 433 RF-transmitter
 * 
 * (c) 2021 Lars Wessels <software@bytebox.org>
 *
 * Sketch to replace a PCR800 433MHz sensor sending precipitation readings 
 * using the Oregon V3 protocol. Tested with WMR200 base station and rtl_433.
 * 
 * References:
 * http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf
 * https://github.com/merbanan/rtl_433/blob/master/src/devices/oregon_scientific.c
 *
 * Published under MIT license.
 * 
 */

#define RF433_TX_PIN 6  // output pin for 433MHz transmitter (e.g. FS1000A)
#define V3_CHANNEL 0
#define V3_PAYLOAD_NIBBLES 18
#define V3_TX_INTERVAL_SECS 47
#define V3_TX_BYTES 15

#define V3_PULSE_LENGHT_US 488  // data rate 1024Hz
#define V3_PULSE_SHORTEN_US 134 // 138us not working reliably
#define V3_PULSE_TUNING 1.2  // increase up to 1.4 if base station doesn't pick up messages

#define DEBUG_PAYLOAD
#define DEBUG_LASTTX
#define DEBUG_INC

// set PCR800's precipitation readings (inches) which are increased 
// after each transmission for testing purposes if DEBUG_INC is set
static float p_total = 2.7;
static float p_hourly = 0.3;

// rolling code changes on every sensor reset
static uint8_t rcode;


// calculate a bitwise crc8 ccitt checksum
uint8_t crc8_checksum_v3(uint8_t *payload, uint8_t payloadNibbles) {
    uint8_t crc = 0x00;  // crc8 init value

    // iterate over nibbles with sensor payload including a trailing iteration
    for (uint8_t i = 7; i <= (payloadNibbles + 7); i++) {
        if (i < (payloadNibbles + 7)) {
          if ((i % 2) == 0)
              crc ^= payload[i/2] >> 4;
          else
              crc ^= (payload[i/2] & 0x0F);
        }
        for (uint8_t j = 0; j < 4; j++) {
            if (crc & 0x80) 
                crc = (crc << 1) ^ 0x07;
            else 
                crc <<= 1;    
        }
    }
    
    // nibbles need to be swapped (LSD)
    return ((crc & 0x0F) << 4) | ((crc & 0xF0) >> 4);
}


// calculate the Oregon V3 checksum
// based on https://github.com/merbanan/rtl_433/blob/master/src/devices/oregon_scientific.c
uint8_t oregon_checksum_v3(uint8_t *payload, int payloadNibbles) {
    uint8_t checksum;
  
    // remove sync byte '0xA' from first sensor data nibble
    checksum = (payload[3] & 0xF);

    // nibbles used for simple checksum (sum of nibbles)
    // 4 (sensor id) + 1 (V3_CHANNEL) + 2 (rolling code) + 1 (flag) + payloadNibbles
    for (uint8_t i = 4; i < ((28 + payloadNibbles*4) / 8); i++)
        checksum += (payload[i] >> 4) + (payload[i] & 0xF);
   
    if ((payloadNibbles % 2) == 0)
        checksum += payload[((28 + payloadNibbles*4)/8)] >> 4;

    return ((checksum & 0x0F) << 4) | ((checksum & 0xF0) >> 4);
}


// send a single byte as series of manchester encoded RF pulses
// base station won't receive packets if timing is off
// adjusting V3_PULSE_TUNING from 1.1 up to 1.4 should help
void manchester_encode_v3(uint8_t txByte) {
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
        delayMicroseconds((V3_PULSE_LENGHT_US * V3_PULSE_TUNING) - V3_PULSE_SHORTEN_US);
        digitalWrite(RF433_TX_PIN, ((txByte & bitMask) >= 1) ? LOW : HIGH);
    }
}  


// returns payload for PCR800 precipitation sensor (15 bytes)
uint8_t *payload_pcr800(uint8_t rollingCode, float pTotal, float pHourly) {
    static uint8_t payload[V3_TX_BYTES];
    uint32_t total;
    uint16_t hourly;
    uint8_t chksum, crc;
        
    // 6 nibbles preamble
    // 1 nibble sync
    // 4 nibbles sensor id
    // 1 nibble V3_CHANNEL 
    // 2 nibbles rolling code
    // 1 nibble flag (battery)
    // n nibbles sensor specific payload (PCR800 n=10, 40 bits)
    // 2 nibbles oregon checksum
    // 2 nibbles crc8 checksum
    memset(payload, 0, V3_TX_BYTES);

    // preamble with 1-bit pulses (6 nibbles) and 1 sync nibble '0101' (28 bits)
    memset(payload, 0xFF, 3); // 1..6
    payload[3] = 0xA0; // 7
  
    // nibbles 8..11: oregon sensor id (16 bits)
    payload[3] |= 0x02; // PCR800 is 0x2914
    payload[4] = 0x91;
    payload[5] = 0x40;
  
    // nibble 12: channel 0..15, PCR800 usually using channel 0 (8 bit)
    payload[5] |= V3_CHANNEL > 15 ? 15 : V3_CHANNEL;
  
    // nibble 13..14 is rolling code, changes on sensor reset (16 bits LSD) 
    payload[6] = rollingCode;
    
    // nibble 15 is flag for battery status (4 bits)
    payload[7] = 0x00; // set to 0x40 for low battery

    // nibbles 16..19 rain rate, 20..25 total precipitation (40 bits)
    hourly = pHourly * 100;
    total = pTotal * 1000;
    payload[8] = ((hourly / 10) % 10) << 4 | ((hourly / 100) % 10);
    payload[9] = ((hourly / 1000) % 10) << 4 | (total % 10);
    payload[10] = ((total / 10) % 10) << 4 | ((total / 100 ) % 10);
    payload[11] = ((total / 1000) % 10) << 4 | ((total / 10000) % 10);
    payload[12] = ((total / 100000) % 10) << 4;

    // checksum (8 bits) for nibbles 16..25 with sensor specific data (sum of nibbles)
    chksum = oregon_checksum_v3(payload, V3_PAYLOAD_NIBBLES);
    payload[12] |= (chksum & 0xf0) >> 4;
    payload[13] = (chksum & 0x0f) << 4;
    
    // crc8 checksum for V3 protocol (8 bits)
    crc = crc8_checksum_v3(payload, V3_PAYLOAD_NIBBLES);
    payload[13] |= (crc & 0xf0) >> 4;
    payload[14] = (crc & 0x0f) << 4;
 
    return payload;  
}


// send manchester encoded payload for a Oregon V3 sensor
void send_data_v3(uint8_t *payload, uint8_t len) {
    uint8_t i;
#ifdef DEBUG_PAYLOAD
    Serial.print(millis());
    Serial.print(": V3 Payload [ ");
    for (i = 0; i < len; i++) {
        if (payload[i] < 16)
            Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
#endif
    digitalWrite(RF433_TX_PIN, LOW);
    for (i = 0; i < len; i++)
        manchester_encode_v3(payload[i]);
        
    // need to add extra delay after last low to high 
    // pulse since there is no more data to come...
    if ((payload[i] & 0x08) == 0)
        delayMicroseconds(V3_PULSE_LENGHT_US);
      
    digitalWrite(RF433_TX_PIN, LOW);
}


void setup() {  
    pinMode(RF433_TX_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(250);   
    Serial.print(F("\n\nStarting Oregon V3 PCR800 emulator...\n\n"));
    
    randomSeed(analogRead(1)); // random seed for rolling code
    rcode = random(0x01, 0xFE);
    Serial.print(millis());
    Serial.print(": Rolling code: ");
    Serial.print(((rcode & 0x0F) << 4) | ((rcode & 0xF0) >> 4));
    Serial.print(", V3_CHANNEL: ");
    Serial.print(V3_CHANNEL);
    Serial.print(", RF433 packets: ");
    Serial.println(V3_TX_BYTES);
}


void loop() {
    static uint32_t seconds = 0;
    static uint32_t lastMillis = 0, lastTX = 0;
    static bool tx_pending = true;

    if (millis() - lastMillis > 1000) {
        digitalWrite(LED_BUILTIN, LOW);
        lastMillis = millis();
        seconds++;
        Serial.print(millis());
        Serial.println(": Waiting for next TX...");
        tx_pending = true;
    }

    // Oregon V3 base station is expecting to 
    // receive readings at fixed intervals
    if (seconds % V3_TX_INTERVAL_SECS == 0 && tx_pending) {
        tx_pending = false;
        digitalWrite(LED_BUILTIN, HIGH);

        Serial.print(millis());
        Serial.print(": PCR800 (Precipitation Total: ");
        Serial.print(p_total);
        Serial.print(" inches, Hourly: ");
        Serial.print(p_hourly);
        Serial.print(", inches");
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
        send_data_v3(payload_pcr800(rcode, p_total, p_hourly), V3_TX_BYTES); 
    
#ifdef DEBUG_INC
        // increase readings for testing
        p_hourly += 0.05;
        if (p_hourly > 3.0)
            p_hourly = 0.5;
        p_total += 0.2;
        if (p_total > 20)
            p_total = 10;
#endif
    }
}
