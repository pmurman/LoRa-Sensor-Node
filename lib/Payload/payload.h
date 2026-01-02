/*!
 * @file payload.h
 */

#ifndef _PAYLOAD_H_
#define _PAYLOAD_H_

#define PADDING (3 - MAX_PAYLOAD % 3) % 3 // Base64 padding bytes needed after payload

class Payload {
  public:
              Payload(void (*transmit_upload_handler)(uint8_t *tx_packet, const uint8_t tx_packet_size), void (*send_packet_number)(void));
    void      addSensorPar(sensor_t sensor_parameter_def);
    void      addError(uint32_t error_warning_code);
    void      sendPacket(sensor_t *sensors_parameter_def);
    void      sendError(uint32_t error_warning_code);
    char      *convert2B64(uint8_t *packet_start, uint8_t *packet_plus_b64fillers_end);
    char      *test(void);

  private:
    void      (*upload_)(uint8_t *tx_packet, const uint8_t tx_packet_size);
    void      (*addPktNr_)(void);
    uint8_t   packet_[1 + MAX_PAYLOAD + PADDING]; // FPort[1] + payload + room for padding bytes
    uint32_t  pde_;
    uint32_t  error_;
    uint8_t   *pktFill_;
    void      out(void);
};
#endif