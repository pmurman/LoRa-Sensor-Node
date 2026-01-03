/*!
 * @author  Peter Murman
 * @date    7 Jan 2021, 23 Oct 2022
 * 
 * @brief   Simple LoRaWAN payload encoder.
 * 
 * @license Licensed under see license.txt
 * 
 * v1.0.0   - Initial version
 *
 * LoRaWAN payload encoding and transmission.
 * 
 * Supports:
 * - Variable length payloads using a Payload Data Encoding header (PDE) as part of the LoRaWAN FRMPayload:
 *    . Allows dynamic parameter passing (eg. no change = no parameter payload)
 *    . One-, two- or three-byte parameter value encoding
 *    . Up to 12 parameter values and 1 error value (max payload: 44 bytes)
 * - Variable length payloads using the LoRaWAN's FPort:
 *    . Supports 223 pre-defined payload encoding schemes (permitted FPort values: 1-223, minus 200-203)
 *    . Supports FPort encoding instead of sending PDE headers, reducing payload and time-on-air
 * - Error reporting
 * - FPort is set to 0 (noPayloadEncoding) when FRMPayload contains the data/error header
 *
 * FPort is currently part of the packet data (1st byte) to simplify uploading to the application server.
 *
 * Output formats:
 * (1) binary payload data (LoRa 'raw' packet)
 * (2) character sequence with base64 encoded data (base64 packet)
 */

#define LORA_HACK // needed for our LoRa 'gateway'
#include "sensor-app.h"
#include "payload.h"

Payload::Payload(void f(uint8_t *, const uint8_t), void g(void)) :
  upload_(f), addPktNr_(g), pde_(0U), error_(0U), pktFill_(packet_) {
}

/**
 * @brief Add parameter to payload header 
 * 
 * @param s Sensor parameter
 */
void Payload::addSensorPar(sensor_t s) {
  pde_ |= s.sz << ((s.idx / 3) * 8 + (s.idx % 3) * 2);
}

/**
 * @brief Add error to error queue
 * 
 * @param e Error/warning code
 */
void Payload::addError(uint32_t e) {
  error_ |= e | 0xc0;
}

/**
 * @brief Produce packet with PDE header and payload data, in case of pending error also error header, and pass to upload callback
 *
 * @param paramList Parameter list for this node
 */
void Payload::sendPacket(sensor_t *paramList) {
  if (!pde_)
    return; // no data

  addPktNr_();

  // set PDE header extension bit(s)
  uint32_t bytePid = 0x3F000000; // parameter ID bitmask for PDE[31:24]
  uint32_t extBits = 0x00808080; // associated EXT bits
  while (extBits) {
    if (pde_ & bytePid) { // parameter(s) in this byte?
      pde_ |= extBits; // set EXT bit(s) & done
      break;
    }
    bytePid >>= 8U;
    extBits >>= 8U;
  }

  uint32_t pid = pde_;
  if (error_) {
    *pktFill_++ = 0; // don't use FPort encoding
    // err hdr length encoding
    uint sz = 0;
    if (error_ > 0xffffff)
      sz = 3;
    else if (error_ > 0xffff)
      sz = 2;
    else if (error_ > 0xff)
      sz = 1;
    error_ |= sz << 4;

    // error hdr
    do {
      *pktFill_++ = error_;
      error_ >>= 8;
    }
    while (sz--);

    // data hdr
    while (pde_) {
      *pktFill_++ = pde_;
      pde_ >>= 8U;
    }
  }
  else // data only - use FPort encoding
    if (pde_ & 0x80) // multi-byte PDE 
      switch (pde_) {
    // multi-byte PDE to FPort mapping goes here
    // eg.
    //  case 0x0303:
    //    *pktFill_++ = 64;
    //    break;
        default:
          *pktFill_++ = 0; // no FPort encoding for this PDE
          while (pde_) {
            *pktFill_++ = pde_;
            pde_ >>= 8U;
          }
          break;
      }
    else {
#if defined LORA_HACK
      pktFill_++; // reserve space for device ID
#endif
      *pktFill_++ = pde_; // FPort = PDE for values 0x01-0x3f (1-63)
    }

  // shift-out parameter data of parameters with non-zero bitfields encoded in the PID
  for (uint i = First; i < End; i++) {
    int32_t data = paramList[i].raw;
    for (uint n = pid >> ((i / 3) * 8 + (i % 3) * 2) & 0b11; n > 0 ; n--) {
      *pktFill_++ = data & 0xff;
      data >>= 8U;
    }
  }

  out();
}

/**
 * @brief Transmit error directly 
 * 
 * @param e Error/warning code
 */
#if defined LORA_HACK
extern uint8_t devEui[8];
#endif
void Payload::sendError(uint32_t e) {
  static uint8_t error_[1 + 1 + PADDING]; // FPort + error header + padding bytes
  error_[0] = 0; // no FPort encoding
  error_[1] = (e & 0x0f) | 0x40; // no header after this one, no error data 
#if defined LORA_HACK
  error_[0] = devEui[2]; // misuse FPort (Oct22: while not used) for sending unique part of our lora DevEUIs to 'gateway'
#endif
  uint8_t skipFPort = *error_ == 0 ? 1 : 0;
  if (upload_ != nullptr)
    upload_(error_ + skipFPort, 2 - skipFPort);
}

void Payload::out(void) {
  // TODO: pass non-zero packet[0] value to LoRaWAN FPort
  // App payload decoder must then use this as PDE and use decoding table matching above encoding scheme for multi-byte PDE
#if defined LORA_HACK
  *packet_ = devEui[2];
#endif

  uint skipFPort = *packet_ == 0 ? 1 : 0;

  if (upload_ != nullptr)
    upload_(packet_ + skipFPort, pktFill_ - (packet_ + skipFPort));
  pde_ = 0U;
  error_ = 0U;
  pktFill_ = packet_;
}

char * Payload::convert2B64(uint8_t * raw, uint8_t * rawEnd) {
  if (rawEnd - raw > MAX_PAYLOAD)
    rawEnd = raw + MAX_PAYLOAD;

  uint padding = 0;
  // add empty octets until raw packet contains multiple of 3 octets
  while ((rawEnd - raw) % 3) {
    *rawEnd++ = 0;
    padding++;
  }

  static char packetB64[MAX_PAYLOAD * 4 + PADDING + 1]; // 4/3 x 3 (worst case every b64 chr is %-encoded + \0
  char *p = packetB64;

  for (uint8_t *r = raw; r < rawEnd; r += 3) {
    // calculate indexes in base64 chr table for each octet triplet
    uint8_t b4[4];
    b4[0] = (*r & 0xfc) >> 2;
    b4[1] = ((*r & 0x03) << 4) | ((*(r+1) & 0xf0) >> 4);
    b4[2] = ((*(r+1) & 0x0f) << 2) | ((*(r+2) & 0xc0) >> 6);
    b4[3] = *(r+2) & 0x3f;

    static const char base64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (auto b : b4)
      if (b < 62)
        *p++ = base64[b];
      else {
        // use url safe encoding for '+' and '/'
        static const char base16[] = "0123456789abcdef";
        *p++ = '%';
        *p++ = base16[base64[b] >> 4U];
        *p++ = base16[base64[b] & 0x0F];
      }
  }
  *p = '\0';
  while (padding--)
    *--p = '=';

  return packetB64;
}

/**
 * @brief A base64 converter test pattern enforcing %-encoding 
 * 
 * @returns ASCII packet with base64 encoded packet data
 * 
Three parameters are encoded in the raw packet, as follows (size in bytes):
PDE (1), p1 (2), p2 (3), p3 (1)

'+' = 0b111110
'/' = 0b111111

             |--PDE--|-------p1--------|
b64 sextets: 765432.107654.321076.543210
hex input:   000111.101111.111110.111111 = 1e ff bf
b64 code:    Hv+/

             |------------p2-----------|
b64 sextets: 765432.107654.321076.543210
hex input:   111110.111110.111111.111111 = fb ef ff
b64 code:    ++//

             |---p3--|xxxxxxxxxxxxxxxxxx
b64 sextets: 765432.107654.321076.543210
hex input:   111110.00                   = f8
b64 code:    /A==

expected decoded parameter values: p1=49151, p2=16773115, p3=248
*/
char *Payload::test() {
  const uint8_t test[] = { '\x1e', '\xff', '\xbf', '\xfb', '\xef', '\xff', '\xf8' };
  pktFill_ = packet_;

  for (auto byte : test)
    *pktFill_++ = byte;

  return convert2B64(packet_, pktFill_);
}
