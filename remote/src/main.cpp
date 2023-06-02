#include "Arduino.h"

#define SAMPLEFILTER_TAP_NUM 43

float max_y = 3560.0f;
float min_y = 325.0f;
float mid_y = 1920.0f;

float max_x = 3600.0f;
float min_x = 285.0f;
float mid_x = 1830.0f;

float max_w = 3560.0f;
float min_w = 300.0f;
float mid_w = 1840.0f;

float offset = 100.0f;

#define LX 36
#define LY 39
#define RX 35
#define front_up 27
#define front_down 25
#define back_up 26
#define back_down 33
#define both_up 5
#define both_down 19
#define lifting_up 18
#define lifting_down 23

int8_t start_byte = (int8_t)0xff;
int8_t zero_byte = (int8_t)0x9f;

static float filter_taps[SAMPLEFILTER_TAP_NUM] = {
  -0.018992483991563325,
  -0.005226506151104887,
  -0.005276138882158422,
  -0.004787697180733653,
  -0.003664860383909234,
  -0.001833572549223195,
  0.0007568031181954745,
  0.0041122917416025135,
  0.008255225082065276,
  0.013058105266784277,
  0.01850486575131265,
  0.02444252414626457,
  0.03068707455615802,
  0.03706575131387409,
  0.043386822071696404,
  0.04944183287789727,
  0.05499845528211098,
  0.05986855538934459,
  0.06385719698827888,
  0.06681698484580635,
  0.06864865017344841,
  0.06926837062158107,
  0.06864865017344841,
  0.06681698484580635,
  0.06385719698827888,
  0.05986855538934459,
  0.05499845528211098,
  0.04944183287789727,
  0.043386822071696404,
  0.03706575131387409,
  0.03068707455615802,
  0.02444252414626457,
  0.01850486575131265,
  0.013058105266784277,
  0.008255225082065276,
  0.0041122917416025135,
  0.0007568031181954745,
  -0.001833572549223195,
  -0.003664860383909234,
  -0.004787697180733653,
  -0.005276138882158422,
  -0.005226506151104887,
  -0.018992483991563325
};

typedef struct joystick_packet {
    int8_t start = start_byte;
    int8_t x;
    int8_t y;
    int8_t w;
    int8_t operation;
} joystick_packet;

typedef struct {
  float history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

joystick_packet msg;

SampleFilter x_filter;
SampleFilter y_filter;
SampleFilter w_filter;

float rescale(float org, float offset){
  return (float)(org - offset);
}

void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i){
    f->history[i] = 0;
  }
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, float input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM){
    f->last_index = 0;
  }
}

float SampleFilter_get(SampleFilter* f) {
  float acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}

float XtoPercentage(float x){
  float temp = 0.0f;
  
  if((x <= offset) & (x >= -offset)){
    return 0.0f;
  }

  if(x < 0.0f){
    temp = (x/min_x)*-100.0f;
  }

  if(x > 0.0f){
    temp = (x/max_x)*100.0f;
  }

  if(temp > 100.0f){
    temp = 100.0f;
  }
  else if(temp < -100.0f){
    temp = -100.0f;
  }
  
  return temp;
}

float YtoPercentage(float y){
  float temp = 0.0f;
  
  if((y <= offset) & (y >= -offset)){
    return 0.0f;
  }

  if(y < 0.0f){
    temp = (y/min_y)*-100.0f;
  }

  if(y > 0.0f){
    temp = (y/max_y)*100.0f;
  }

  if(temp > 100.0f){
    temp = 100.0f;
  }
  else if(temp < -100.0f){
    temp = -100.0f;
  }

  return temp;
}

float WtoPercentage(float w){
  float temp = 0.0f;
  
  if((w <= offset) & (w >= -offset)){
    return 0.0f;
  }

  if(w < 0.0f){
    temp = (w/min_w)*-100.0f;
  }

  if(w > 0.0f){
    temp = (w/max_w)*100.0f;
  }

  if(temp > 100.0f){
    temp = 100.0f;
  }
  else if(temp < -100.0f){
    temp = -100.0f;
  }

  return temp;
}

void simpling(){
  for(int i = 0; i < 100; i++){
    float y = analogRead(LY);
    float x = analogRead(LX);
    float w = analogRead(RX);

    SampleFilter_put(&x_filter, rescale(x, mid_x));
    SampleFilter_put(&y_filter, rescale(y, mid_y));
    SampleFilter_put(&w_filter, rescale(w, mid_w));

    delayMicroseconds(100);
  }
}

void preparePacket(){

  msg.x = (int8_t)XtoPercentage(SampleFilter_get(&x_filter));
  msg.y = (int8_t)YtoPercentage(SampleFilter_get(&y_filter));
  //msg.w = (int8_t)0x00;
  msg.w = (int8_t)WtoPercentage(SampleFilter_get(&w_filter));

  /*Serial.print("x = ");
  Serial.print(msg.x);
  Serial.print(", y = ");
  Serial.print(msg.y);
  Serial.print(", w = ");
  Serial.print(msg.w);
  Serial.print('\n');*/

  if(msg.x == 0x00){
    msg.x = zero_byte;
  }

  if(msg.y == 0x00){
    msg.y = zero_byte;
  }

  if(msg.w == 0x00){
    msg.w = zero_byte;
  }

  if(digitalRead(front_up) == 0){
    msg.operation = (int8_t)50;
    return;
  }
  
  if(digitalRead(front_down) == 0){
    msg.operation = (int8_t)51;
    return;
  }

  if(digitalRead(back_up) == 0){
    msg.operation = (int8_t)52;
    return;
  }

  if(digitalRead(back_down) == 0){
    msg.operation = (int8_t)53;
    return;
  }

  if(digitalRead(both_up) == 0){
    msg.operation = (int8_t)54;
    return;
  }

  if(digitalRead(both_down) == 0){
    msg.operation = (int8_t)55;
    return;
  }

  if(digitalRead(lifting_up) == 0){
    msg.operation = (int8_t)56;
    return;
  }

  if(digitalRead(lifting_down) == 0){
    msg.operation = (int8_t)57;
    return;
  }

  msg.operation = zero_byte;
}

void sendPacket(){
  //Serial.write((char*)&msg, sizeof(msg));

  Serial2.write((char*)&msg, sizeof(msg));
  Serial2.flush();
}

void setup() {
  max_x = rescale(max_x, mid_x);
  min_x = rescale(min_x, mid_x);
  
  max_y = rescale(max_y, mid_y);
  min_y = rescale(min_y, mid_y);

  max_w = rescale(max_w, mid_w);
  min_w = rescale(min_w, mid_w);

  SampleFilter_init(&x_filter);
  SampleFilter_init(&y_filter);
  SampleFilter_init(&w_filter);
  
  pinMode(LX, INPUT);
  pinMode(LY, INPUT);
  pinMode(RX, INPUT);
  pinMode(front_up, INPUT_PULLUP);
  pinMode(front_down, INPUT_PULLUP);
  pinMode(back_up, INPUT_PULLUP);
  pinMode(back_down, INPUT_PULLUP);
  pinMode(both_up, INPUT_PULLUP);
  pinMode(both_down, INPUT_PULLUP);
  pinMode(lifting_up, INPUT_PULLUP);
  pinMode(lifting_down, INPUT_PULLUP);

  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  /*Serial.print("LX: ");
  Serial.print(analogRead(LX));
  Serial.print(" LY: ");
  Serial.print(analogRead(LY));
  Serial.print(" RX: ");
  Serial.print(analogRead(RX));
  Serial.print(" Start: ");
  Serial.println(digitalRead(START));*/

  simpling();
  preparePacket();
  sendPacket();
}