#include <servo42c.h>
#include <mbed.h>

void servo42c::servo42c_init(int CAN_ID, CAN* _CAN, int hz){
  id = CAN_ID;
  CAN0 = _CAN;
  CAN0->frequency(hz);
  Rxmsg.format = CANExtended;
}

void servo42c::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) 
{
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

uint8_t servo42c::sendPacket(uint32_t id, uint8_t packet[], int32_t len)
{
  if (CAN0->write(CANMessage(id, (const char*)packet, sizeof(packet),CANData,CANExtended)))
  {   
    return 1;
  }
  else {
    //Serial.println("Error Sending Message...");
    return 0;
  }
}

uint8_t servo42c::mks42c_read_uint8(uint8_t address, uint8_t* value){
  int32_t send_index = 3;
  uint8_t buffer[3];
  uint8_t result = sendPacket(id|address, buffer, send_index);
  *value = Rxmsg.data[1];
  return result;
}

uint8_t servo42c::mks42c_read_uint16(uint8_t address, uint16_t* value){
  int32_t send_index = 4;
  uint8_t buffer[4];
  uint8_t result = sendPacket(id|address, buffer, send_index);
  *value = (Rxmsg.data[1] << 8) + Rxmsg.data[2];
  return result;
}

uint8_t servo42c::mks42c_read_uint32(uint8_t address, uint32_t* value){
  int32_t send_index = 6;
  uint8_t buffer[6];
  uint8_t result = sendPacket(id|address, buffer, send_index);
  *value = (Rxmsg.data[1] << 24) + (Rxmsg.data[2] << 16) + (Rxmsg.data[3] << 8) + Rxmsg.data[4];
  return result;
}

uint8_t servo42c::mks42c_read_bytes(uint8_t address, uint8_t* data, uint8_t n){
  uint8_t buffer[32];
  int i = 0;
  int MSB_idx = 0;
  float start, endT;
  
    while (true) {
      start = clock();
      endT = clock();
    for (;;) {
        while (!serial.writeable()) {
            if (endT - start >= MKS42C_UART_TIMEOUT) {
              serial.puts("TIMEOUT"); //
              return 0;
            }
        }
        buffer[MSB_idx+i] = serial.readable();
        if(i>0 && (buffer[MSB_idx+1] == address)){
          MSB_idx++;
          i=0;
        }
        if(buffer[MSB_idx] == address) i++;
        if (i == n){
          uint8_t rCHK = 0;
          for(int j=0; j<n-1; j++){
            rCHK += buffer[MSB_idx+j];
          }
          rCHK = rCHK & 0xFF;
          if(rCHK == buffer[MSB_idx+n-1]){
            serial.puts("(Valid) uint8 Result:");
            for(int k=0; k<n ;k++){
//              Serial.printf("%02x ", buffer[MSB_idx+k]);
              data[k] = buffer[MSB_idx+k];
            }
            serial.puts("");
            return 1;
          }
          else{
            serial.puts("(Invalid) uint8 Result:");
            for(int k=0; k<n ;k++){
//              Serial.printf("%02x ", buffer[MSB_idx+k]);
            }
            serial.puts("");
            do{
              MSB_idx--;
              i--;
            }while((buffer[MSB_idx] != address) && (i > 0));
//            MSB_idx = 0;
//            i == 0;
          }
        }
    }}
    return 0;
}
uint16_t servo42c::mks42c_read_encoder(uint8_t address){
   //send e0 30 10 //recieve e0 40 00 20
  serial.puts("Reading...");

  uint8_t func_code = 0x30;
  uint8_t tCHK = (address + func_code) & 0xFF;

  serial.putc(address); // address
  serial.putc(func_code); 
  serial.putc(tCHK);

  uint16_t value;
  mks42c_read_uint16(address, &value);
  printf("Encoder: %d\n", value);

  return value;
}

uint8_t servo42c::mks42c_set_openloop_mode(uint8_t address){
  //dont use during velocity mode
  //e0 82 xx tCHK //00-CR_OPEN;(Open loop mode) //02-CR_UART(Closed loop mode (UART interface))
  //Return e0 , result (uint8_t), rCHK.
  serial.puts("Set mode");

  uint8_t func_code = 0x82;
  uint8_t tCHK = (address + 0x82 + 0x00) & 0xFF;

  serial.putc(address); // address
  serial.putc(func_code); 
  serial.putc(0x00);
  serial.putc(tCHK);

  uint8_t value;
  mks42c_read_uint8(address, &value);
  printf("mks42c_set_openloop_mode: %d\n", value);

  return value;
}

uint8_t servo42c::mks42c_set_closedloop_mode(uint8_t address){
  //dont use during velocity mode
  //e0 82 xx tCHK //00-CR_OPEN;(Open loop mode) //02-CR_UART(Closed loop mode (UART interface))
  //Return e0 , result (uint8_t), rCHK.
  serial.puts("Set mode");

  uint8_t func_code = 0x82;
  uint8_t tCHK = (address + func_code + 0x02) & 0xFF;

  serial.putc(address); // address
  serial.putc(func_code); 
  serial.putc(0x02);
  serial.putc(tCHK);

  uint8_t value;
  mks42c_read_uint8(address, &value);
  printf("mks42c_set_closedloop_mode: %d\n", value);
  
  return value;
}

uint8_t servo42c::mks42c_set_velocity(uint8_t address, uint8_t dir, uint8_t speed){
  //command 31: e0 f6 xx tCHK
  //Vrpm = (speed × 3000)/(Mstep × 200)(RPM) 1.8°motor
  //Return e0 ,result (uint8_t),rCHK.
  serial.puts("Set Velocity");

  uint8_t func_code = 0xf6;
  uint8_t command_value = speed & 0b01111111;
  command_value += ((dir & 0b00000001) << 7);
//  Serial.printf("command_value: %0x\n", command_value);
  uint8_t tCHK = (address + func_code + command_value) & 0xFF;
  
  serial.putc(address); // address
  serial.putc(func_code); 
  serial.putc(command_value);
  serial.putc(tCHK);

  uint8_t value;
  mks42c_read_uint8(address, &value);
  printf("mks42c_set_velocity: %d\n", value);

  return value;
}

uint8_t servo42c::mks42c_set_velocity_by_rpm(uint8_t address, uint8_t dir, uint16_t rpm){
   int Mstep = 16;
  double speed = rpm*Mstep*20;
  speed = speed/3000.0;
  mks42c_set_velocity(address, dir, (uint8_t)speed);

  return 0;
}

uint8_t servo42c::mks42c_set_angle(uint8_t address, uint8_t dir, uint8_t rpm, double angle){
  //command 34: e0 fd xx xx xx xx xx tCHK
  //The third byte(xx) defines the direction and speed, such as command 9.
  //The last two bytes(xx xx xx xx ) define the number of pulses.
  //Return e0 ,result (uint8_t),rCHK.
  serial.puts("Set Velocity");

  uint8_t func_code = 0xfd;
  int Mstep = 16;
  double speed = rpm*Mstep*20;
  speed = speed/3000.0;
//  Serial.printf("speed: %f\n", speed);
  uint8_t command_speed_value = (uint8_t)speed & 0b01111111;
  command_speed_value += ((dir & 0b00000001) << 7);
//  Serial.printf("command_value: %0x\n", command_speed_value);
  uint32_t steps = Mstep*angle/1.8;
  uint32_t command_pulse_value = (uint32_t)steps;
  printf("command_pulse_value: %lu\n", steps);

  uint8_t command_pulse_value_1 = (command_pulse_value >> 24);
  uint8_t command_pulse_value_2 = command_pulse_value >> 16;
  uint8_t command_pulse_value_3 = command_pulse_value >> 8;
  uint8_t command_pulse_value_4 = command_pulse_value;

  uint8_t tCHK = (address + func_code + command_speed_value
    + command_pulse_value_1 + command_pulse_value_2
    + command_pulse_value_3 + command_pulse_value_4) & 0xFF;
  
  serial.putc(address); // address
  serial.putc(func_code); 
  serial.putc(command_speed_value);
  serial.putc(command_pulse_value_1);
  serial.putc(command_pulse_value_2);
  serial.putc(command_pulse_value_3);
  serial.putc(command_pulse_value_4);
  serial.putc(tCHK);

  uint8_t value;
  mks42c_read_uint8(address, &value);
  printf("mks42c_set_angle: %d\n", value);
  
  return value;
}

uint8_t servo42c::mks42c_stop_motor(uint8_t address){
  //command 32: e0 f7 tCHK
  //Return e0 and the result (uint8_t) and rCHK.
  serial.puts("Stop Motor");

  uint8_t func_code = 0xf7;
  uint8_t tCHK = (address + func_code) & 0xFF;

  serial.putc(address); // address
  serial.putc(func_code);
  serial.putc(tCHK);

  uint8_t value;
  mks42c_read_uint8(address, &value);
  printf("mks42c_stop_motor: %d\n", value);

  return value;
}


