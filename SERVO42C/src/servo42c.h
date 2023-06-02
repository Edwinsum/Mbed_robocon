#include <mbed.h>

#define MKS42C_UART_TIMEOUT 500 // 500ms

class servo42c{
    public:
    Serial serial;
    time_t a, b;
    int id;
    char mks42c_buffer[128];
    CAN* CAN0;
    CANMessage Txmsg;
    CANMessage Rxmsg;

    private:
    void servo42c_init(int CAN_ID, CAN* _CAN, int hz);
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

    uint8_t sendPacket(uint32_t id, uint8_t packet[], int32_t len);
    uint8_t mks42c_read_uint8(uint8_t address, uint8_t* value); 
    uint8_t mks42c_read_uint16(uint8_t address, uint16_t* value); 
    uint8_t mks42c_read_uint32(uint8_t address, uint32_t* value);
    uint8_t mks42c_read_bytes(uint8_t address, uint8_t* data, uint8_t n); 
    uint16_t mks42c_read_encoder(uint8_t address);
    uint8_t mks42c_set_openloop_mode(uint8_t address);
    uint8_t mks42c_set_closedloop_mode(uint8_t address);
    uint8_t mks42c_set_velocity(uint8_t address, uint8_t dir, uint8_t speed);
    uint8_t mks42c_set_velocity_by_rpm(uint8_t address, uint8_t dir, uint16_t rpm);
    uint8_t mks42c_set_angle(uint8_t address, uint8_t dir, uint8_t rpm, double angle);
    uint8_t mks42c_stop_motor(uint8_t address);




};