// put your setup code here, to run once:
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MCP342x.h>
#include <math.h>
# include "driver/adc.h" 
# include "esp_adc_cal.h"

#define TPS      (0.5/60.0) // temperature change rate in degree / second
#define FAST_TPS (4.0/60.0) // fast cooling rate in degree / second

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Temperature measument using a thermistor
//   Device: SEMITEC JT103-050 https://akizukidenshi.com/goodsaffix/jt_thermistor.pdf
//   T0:   Nominal temperature (in kelvin)  (SEMITEC JT103-050: 25C = 25 + 273.15 Kelvin)
//   R_T0: Resistance at the nominal temperature (10.0KOhm)
//   B:    Thermistor B constant (SEMITEC JT103-050: B=3435 +- 1%)
//   T:    Temperature around the thermistor (in Kelvin)
//   R_T:  Thermistor resistence at T
//
//   R_T = R_T0 * exp(B * (1/T - 1/T0))
//   B * (1/T - 1/T0) = log(R_T/RT0)
//   1/T - 1/T0 = ln(R_T/RT0) / B
//   1/T = ln(R_T / RT0) / B + 1 / T0
//   T   = 1 / (ln(R_T / RT0) / B + 1 / T0)
//
//   Schematic: V (2.048V  +-0.08%) -- R (10K Ohm +-0.1%) -(Vt)- R_T (Thermistor) -- GND
//   Vt / R_T = V / (R + R_T) ; current flowing in R and R_T is the same.
//   Vt * (R + R_T) = V * R_T
//   R_T (V - Vt) = Vt * R
//   R_T = Vt * R / (V - Vt)
//   ADC: 2.048 V = 32768
#define K0     273.15
#define T0    (K0 + 25.0)     // Kelvin
#define R_T0 10000.0         // Ohm
#define B     3435.0 
#define V        2.048       // V
#define R    10000.0         // Ohm
double calc_temp(long adc) {
  double Vt  = (double) V * (double) adc / 32768.0; 
  double R_T = Vt * R / (V - Vt);
  double T   = (double) 1.0 / (log(R_T / R_T0) / B + 1.0 / T0);
  return T - K0;
}

 // 0x68 is the default address for all MCP342x devices
uint8_t adc_addr = 0x68;
MCP342x adc = MCP342x(adc_addr);

int    mode = 0;
double t1, t2, t3, t4;
float  dci = 0.0;
char   hci[2] = "I"; 

void disp_status() {
  char tmp_s[24];
  display.clearDisplay();
  //display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,  0);
  sprintf(tmp_s, "1 %5.2f  2 %5.2f", t1, t2); display.println(tmp_s);
  display.setCursor(10, 10);
  sprintf(tmp_s, "3 %5.2f  4 %5.2f", t3, t4); display.println(tmp_s);
  display.setCursor(10, 20);
  if      (mode == 0) sprintf(tmp_s, "%s idle    %5.2fA", hci, dci);
  else if (mode == 1) sprintf(tmp_s, "%s heating %5.2fA", hci, dci);
  else if (mode == 2) sprintf(tmp_s, "%s cooling %5.2fA", hci, dci);
  else if (mode == 3) sprintf(tmp_s, "%s holding %5.2fA", hci, dci);
  else if (mode == 4) sprintf(tmp_s, "%s fastcl %5.2fA", hci, dci);
  display.println(tmp_s);  
  display.display();      // Show initial text
}

#define PNP1   16 // ON at low, OFF at high
#define PNP2   19
#define NPN1   18
#define NPN2   17
#define SW1    33
#define SW2    25
#define SW3    26
#define SW4    27
#define DCI    36

void go_idle() {
  digitalWrite(PNP1, HIGH); // All OFF
  digitalWrite(PNP2, HIGH);
  digitalWrite(NPN1, LOW);
  digitalWrite(NPN2, LOW);
  strcpy(hci, "I");
}

void do_heat() {
  digitalWrite(PNP2, HIGH); // OFF
  digitalWrite(NPN1, LOW);  // OFF
  digitalWrite(PNP1, LOW);  // ON
  digitalWrite(NPN2, HIGH); // ON
  strcpy(hci, "H");
}

void do_cool() {
  digitalWrite(PNP1, HIGH); // OFF
  digitalWrite(NPN2, LOW);  // OFF
  digitalWrite(PNP2, LOW);  // ON
  digitalWrite(NPN1, HIGH); // ON
  strcpy(hci, "C");
}

void read_adc() {
  int i;
  long value[4] = {0, 0, 0, 0};
  char tmp_s[20];
  MCP342x::Channel chn[4] = {MCP342x::channel1, MCP342x::channel2, MCP342x::channel3, MCP342x::channel4};
  // Initiate a conversion; convertAndRead() will wait until it can be read
  for (i = 0; i < 4; i ++) {
    MCP342x::Config status;
    uint8_t err = adc.convertAndRead(chn[i], MCP342x::oneShot,
                           MCP342x::resolution16, // 16 bit value
                                     MCP342x::gain1,        // Gain = 0dB
             1000000, value[i], status);
    if (err) {
      Serial.print("ADC ERR, ch="); Serial.print(i);
      Serial.print(", err="); Serial.println(err);
      continue;
    }
  }
  for (i = 0; i < 4; i ++) {
    if (value[i] < 1000) value[i] = 1000; // Probably the thermistor is not connected.
  }
  t1 = calc_temp(value[3]);
  t2 = calc_temp(value[2]);
  t3 = calc_temp(value[1]);
  t4 = calc_temp(value[0]);
  Serial.print("ADC ");
  sprintf(tmp_s, "%6d  %5.2f C", value[0], t1); Serial.print(tmp_s);
  sprintf(tmp_s, "%6d  %5.2f C", value[1], t2); Serial.print(tmp_s);
  sprintf(tmp_s, "%6d  %5.2f C", value[2], t3); Serial.print(tmp_s);
  sprintf(tmp_s, "%6d  %5.2f C", value[3], t4); Serial.print(tmp_s);
}

esp_adc_cal_characteristics_t adcChar;
void dc_current() {
  uint32_t voltage;
  char tmp_s[20];
  esp_adc_cal_get_voltage(ADC_CHANNEL_0, &adcChar, &voltage);
  dci = (float) (voltage - 73) / 1024.0 * 1.1 / 0.22 * 1.25;  // 0.22ohm
  sprintf(tmp_s, "%5d %4.2fA", voltage, dci);
  Serial.print(tmp_s);
}

void setup() {
  // Serial
  Serial.begin(115200); 
  Wire.begin();

  // Reset ADC
  MCP342x::generalCallReset();

  //
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  // For PNP Transistors, HIGH means OFF
  pinMode(PNP1, INPUT_PULLUP);
  pinMode(PNP1, OUTPUT);
  digitalWrite(PNP1, HIGH);
  pinMode(PNP2, INPUT_PULLUP);
  pinMode(PNP2, OUTPUT);
  digitalWrite(PNP2, HIGH);
  // For NPN Transistors, LoW means OFF
  pinMode(NPN1, OUTPUT);
  digitalWrite(NPN1, LOW);
  pinMode(NPN2, OUTPUT);
  digitalWrite(NPN2, LOW);
  // ESP32 ADC for current measurement
  //adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_0);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adcChar);
}

double   start_t4;
uint32_t start_ms = 0;

void temp_ctrl() {
  if (mode == 0) return;

  uint32_t elapsed_ms = millis() - start_ms;
  double target_t4;
  if      (mode == 1) {
    double target_t4 = start_t4 + TPS * elapsed_ms / 1000.0;
    if (t4 > target_t4) go_idle();
    else                do_heat();
  }
  else if (mode == 2) {
    double target_t4 = start_t4 - TPS * elapsed_ms / 1000.0;
    if      (t4 > target_t4) do_cool();
    else if (t4 < target_t4) do_heat();
    else                     go_idle();
  }
  else if (mode == 4) {
    double target_t4 = start_t4 - FAST_TPS * elapsed_ms / 1000.0;
    if      (t4 > target_t4) do_cool();
    else if (t4 < target_t4) do_heat();
    else                     go_idle();
  }
  else if (mode == 3) {
    if      (t4 < start_t4 - 0.1) do_heat();
    else if (t4 > start_t4 + 0.1) do_cool();
    else                          go_idle();
  }
  Serial.print(" ");
  Serial.print(hci);
  Serial.print(" ");
  Serial.print(elapsed_ms);
}


void loop() {
  // put your main code here, to run repeatedly:
  read_adc();
  dc_current();

  int new_mode = -1;
  bool sw1Low = (digitalRead(SW1) == LOW);
  bool sw2Low = (digitalRead(SW2) == LOW);
  bool sw3Low = (digitalRead(SW3) == LOW);
  bool sw4Low = (digitalRead(SW4) == LOW);
  if      (sw3Low && sw4Low) new_mode = 4; // fast cool
  else if (sw1Low) new_mode = 1; // heat
  else if (sw2Low) new_mode = 2; // cool
  else if (sw3Low) new_mode = 3; // hold
  else if (sw4Low) new_mode = 0; // idle
  if (new_mode != -1) {
    mode     = new_mode;
    start_t4 = t4;
    start_ms = millis();
  }
  else {
    temp_ctrl();
  }
  if (t4 > 42.0 || t4 < 10.0) mode = 0;
  disp_status();
  Serial.println();
}
