
/*
  HIgh performance AM/FM RDS software-defined radio signal processing tuner with TEF668X marked F8602/F8605/F8705 and I2C Arduino Controller
  for modified NXP-GTK-MPX v1.2-Git / v1.2-Beta user interface ( uncomment your TEF option in "void setup"! )
  by RTVDXRO ( 22 06 2023 Rotor )
  ( for old "historical" versions visit rdi.boards.net ! )

Antenna Switch option:

  Ant A = Normal ( STD )

  Ant B = iMS enabled ( FM multipath suppression when the receiver is in a moving vehicle )

  Ant C = EQ enabled ( FM channel equalizer for improved field performance )

  Ant D = Both enabled

For LNA gain reduction option (F8605/F8705) enable " Show antenna input alignment " in NXP-GTK interface ! ( max 36db attenuation )
Audio output level is variable with optimised HI-BLEND on poor signals but don't expect to be some sort of HiFi / Hi-End ...

Great special thanks to Konrad Kosmatka, author of the original & brilliant version for Sony XDR-F1HD

  https://fmdx.pl/xdr-i2c/
  https://fmdx.pl/xdr-gtk/

  and also to ( in alphabetical order ):

  - ace919
  - Brian Beezley
  - dxhdtv
  - eggplant886
  - fmdxbp
  - Jan Kolar
  - Marcin Woloszczuk
  - Mihai Popa          https://github.com/stailus/tef6686_rds
  - makserge            https://github.com/makserge/tef6686_radio
  - Nicu Florica        http://nicuflorica.blogspot.com/2020/02/radio-cu-tef6686.html
  - ODJeetje
  - Olivier Guillaume
  - Przemyslaw Korpas
  - Sjef Verhoeven      https://github.com/PE5PVB
  - VoXiTPro            https://github.com/voxit1512/Tef6686

  and many other enthusiasts ...

  Tested on Arduino Nano V3.0 at 5V

  Feel free to add your own contribution to this nice project !
*/

#include <Arduino.h>
#include <Wire.h>
#include "DSP_INIT_F8602.h" // v102 - uncomment your option in "void setup"
#include "DSP_INIT_F8605.h" // v205 - uncomment your option in "void setup"

#define SERIAL_PORT_SPEED 115200
#define SERIAL_BUFFER_SIZE 16
char buff[SERIAL_BUFFER_SIZE];
#define TIMER_INTERVAL 133
#define DATA_TIMER_INTERVAL 0
#define AM_VOL_SCALE -1
#define RELAY_CW_PIN 5  // Change this to the pin number connected to your  CW relay
#define RELAY_CCW_PIN 6 // Change this to the pin number connected to your CCW relay

byte DSP_I2C = (0x65U);

uint32_t MODA_FREQ = 0;
uint32_t MODF_FREQ = 0;
uint32_t REG_FREQ;
uint32_t freq;
uint32_t timer = 0;      // Signal level reporting timer
uint32_t timer_DATA = 0; // RDS DATA reporting timer
uint8_t buff_pos = 0;

int16_t nDeemphasis, volume;
int16_t AudioLevel;
int16_t LevelOffset;
int16_t LevelAtt;
int16_t LevelAM;
int16_t AttFM;

int8_t current_filter = -1; // -1 = adaptive
int8_t current_set = -1;
int8_t AGC_tress;
int8_t RF1plus;
int8_t RF2plus;
int8_t Squelch;
int8_t Setsquelch;
int8_t Filter_AM = 16;
int8_t Filter_FM = 16;
int8_t radio_mode;
int8_t forced_mono;
int8_t mode;
int8_t scan_mode;

/* Scan */
uint8_t scan_filter = 0;
uint16_t scan_start = 0;
uint16_t scan_end = 0;
uint16_t scan_step = 0;
uint8_t AM_scan_filter = 0;
uint16_t AM_start_scan = 0;
uint16_t AM_scan_end = 0;
uint16_t AM_scan_step = 0;

/* Get_Quality_Status */
int8_t snr;
int16_t level;
uint16_t usn;
uint16_t wam;
int16_t offset;
uint16_t bandwidth;
uint16_t mod;
uint16_t status;

// XDR-GTK Antenna Switch option
static const uint8_t INIT_1[] PROGMEM = // Ant A
    {
        // Multipath Off & ChannelEqualizer Off
        0x05, 0x20, 0x14, 0x01, 0x00, 0x00,                                     // FM_Set_MphSuppression(1,0)
        0x05, 0x20, 0x16, 0x01, 0x00, 0x00,                                     // FM_Set_ChannelEqualizer(1,0)
        0x07, 0x20, 0x5A, 0x01, 0x00, 0x64, 0x00, 0x1E,                         // FM_Set_StBandBlend_Time          (1, 100, 30)
        0x0B, 0x20, 0x5B, 0x01, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, // FM_Set_StBandBlend_Gain          (1, 1000, 1000, 1000, 1000)
        0x0B, 0x20, 0x5C, 0x01, 0xFF, 0xB5, 0xFF, 0xDD, 0xFF, 0xE7, 0xFF, 0xE7, // FM Set_StBandBlend_Bias          (1, -75, -35, -25, -25)
        0x00};

static const uint8_t INIT_2[] PROGMEM = // Ant B
    {
        // Multipath On & ChannelEqualizer Off
        0x05, 0x20, 0x14, 0x01, 0x00, 0x01,                                     // FM_Set_MphSuppression(1,1)
        0x05, 0x20, 0x16, 0x01, 0x00, 0x00,                                     // FM_Set_ChannelEqualizer(1,0)
        0x07, 0x20, 0x5A, 0x01, 0x00, 0x64, 0x00, 0x1E,                         // FM_Set_StBandBlend_Time          (1, 100, 30)
        0x0B, 0x20, 0x5B, 0x01, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, // FM_Set_StBandBlend_Gain          (1, 1000, 1000, 1000, 1000)
        0x0B, 0x20, 0x5C, 0x01, 0xFF, 0xB5, 0xFF, 0xDD, 0xFF, 0xE7, 0xFF, 0xE7, // FM Set_StBandBlend_Bias          (1, -75, -35, -25, -25)
        0x00};

static const uint8_t INIT_3[] PROGMEM = // Ant C
    {
        // Multipath Off & ChannelEqualizer On
        0x05, 0x20, 0x14, 0x01, 0x00, 0x00,                                     // FM_Set_MphSuppression(1,0)
        0x05, 0x20, 0x16, 0x01, 0x00, 0x01,                                     // FM_Set_ChannelEqualizer(1,1)
        0x07, 0x20, 0x5A, 0x01, 0x00, 0x64, 0x00, 0x1E,                         // FM_Set_StBandBlend_Time          (1, 100, 30)
        0x0B, 0x20, 0x5B, 0x01, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, // FM_Set_StBandBlend_Gain          (1, 1000, 1000, 1000, 1000)
        0x0B, 0x20, 0x5C, 0x01, 0xFF, 0xB5, 0xFF, 0xDD, 0xFF, 0xE7, 0xFF, 0xE7, // FM Set_StBandBlend_Bias          (1, -75, -35, -25, -25)
        0x00};

static const uint8_t INIT_4[] PROGMEM = // Ant D
    {
        // Multipath On & ChannelEqualizer On
        0x05, 0x20, 0x14, 0x01, 0x00, 0x01,                                     // FM_Set_MphSuppression(1,1)
        0x05, 0x20, 0x16, 0x01, 0x00, 0x01,                                     // FM_Set_ChannelEqualizer(1,1)
        0x07, 0x20, 0x5A, 0x01, 0x00, 0x64, 0x00, 0x1E,                         // FM_Set_StBandBlend_Time          (1, 100, 30)
        0x0B, 0x20, 0x5B, 0x01, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, // FM_Set_StBandBlend_Gain          (1, 1500, 1500, 1500, 1500)
        0x0B, 0x20, 0x5C, 0x01, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0xFA, // FM Set_StBandBlend_Bias          (1, 250, 250, 250, 250)
        0x00};

const uint8_t AMFilterMap[] PROGMEM = {30, 40, 60, 80, 30, 40, 60, 80, 30, 40, 60, 80, 30, 40, 60, 80};
const uint16_t FMFilterMap[] PROGMEM = {560, 640, 720, 840, 960, 1140, 1320, 1500, 1680, 1840, 2000, 2180, 2360, 2540, 2860, 3120};

void Write(uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(DSP_I2C);
  for (int i = 0; i < len; i++)
    Wire.write(*buf++);
  Wire.endTransmission();
}

void Read(uint8_t *buf, uint8_t len)
{
  uint8_t lenrec = Wire.requestFrom(DSP_I2C, len);
  for (int i = 0; i < lenrec; i++)
    *buf++ = Wire.read();
}

void Set_Cmd(uint8_t mdl, uint8_t cmd, int len, ...)
{
  uint8_t buf[31];
  uint16_t temp;
  va_list vArgs;
  va_start(vArgs, len);
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  for (uint8_t i = 0; i < len; i++)
  {
    temp = va_arg(vArgs, uint16_t);
    buf[3 + i * 2] = (uint8_t)(temp >> 8);
    buf[4 + i * 2] = (uint8_t)temp;
  }
  va_end(vArgs);
  Write(buf, len * 2 + 3);
}

void Get_Cmd(uint8_t mdl, uint8_t cmd, int16_t *receive, int len)
{
  uint8_t buf[3];
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  Write(buf, 3);
  Read((uint8_t *)receive, 2 * len);
  for (uint8_t i = 0; i < len; i++)
  {
    uint16_t newval = (uint8_t)(receive[i] >> 8) | (((uint8_t)(receive[i])) << 8);
    receive[i] = newval;
  }
}

void dsp_write_data(const uint8_t *data)
{
  uint8_t *pa = (uint8_t *)data;
  uint8_t len, first;
  for (;;)
  {
    len = pgm_read_byte_near(pa++);
    first = pgm_read_byte_near(pa);
    if (!len)
      break;
    if (len == 2 && first == 0xff)
    {
      int delaytime = pgm_read_byte_near(++pa);
      delay(delaytime);
      pa++;
    }
    else
    {
      Wire.beginTransmission(DSP_I2C);
      for (int i = 0; i < len; i++)
        Wire.write(pgm_read_byte_near(pa++));
      Wire.endTransmission();
    }
  }
}

void scan(bool continous)
{

  uint32_t freq;

  if (scan_start > 6500)
  {
    scan_mode = 0;
  }
  else
  {
    scan_mode = 1;
    scan_start = AM_start_scan;
    scan_end = AM_scan_end;
    scan_step = AM_scan_step;
  }
  if (scan_mode == 0)
  {
    Set_Cmd(32, 1, 2, 1, scan_start);
  }
  if (scan_mode == 1)
  {
    Set_Cmd(33, 10, 2, scan_filter == -1 ? 1 : 0, pgm_read_byte_near(AMFilterMap + scan_filter));
    Set_Cmd(33, 1, 2, 1, scan_start);
  }
  do
  {
    Serial.print('U');
    for (freq = scan_start; freq <= scan_end; freq += scan_step)
    {
      Set_Cmd(scan_mode == 0 ? 32 : 33, 1, 2, 1, freq);
      Serial.print(scan_mode == 0 ? freq * 10 : freq, DEC);
      Serial.print('=');
      delay(10);

      int16_t uQuality[4] = {0};
      Get_Cmd(scan_mode == 0 ? 32 : 33, 128, uQuality, 4);
      Serial.print(uQuality[1] / 10, DEC);
      Serial.print(',');
    }
    Serial.print('\n');
  } while (continous && !Serial.available());
  //  Restore previous settings
  if (radio_mode == 0)
  {
    Set_Cmd(32, 1, 2, 1, REG_FREQ / 10);
  }
  else
  {
    Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
    Set_Cmd(33, 1, 2, 1, REG_FREQ);
  }
}

void serial_hex(uint8_t val)
{
  Serial.print((val >> 4) & 0xF, HEX);
  Serial.print(val & 0xF, HEX);
}

void get_DATA()
{
  int16_t uDATA_Data[8] = {0};
  Get_Cmd(32, 131, uDATA_Data, 8);
  if (bitRead(uDATA_Data[0], 15) == 1)
  {
    Serial.print('P');
    serial_hex(uDATA_Data[1] >> 8);
    serial_hex(uDATA_Data[1]);
    Serial.print('\n');
    Serial.print('R');
    serial_hex(uDATA_Data[2] >> 8);
    serial_hex(uDATA_Data[2]);
    serial_hex(uDATA_Data[3] >> 8);
    serial_hex(uDATA_Data[3]);
    serial_hex(uDATA_Data[4] >> 8);
    serial_hex(uDATA_Data[4]);
    serial_hex(uDATA_Data[5] >> 8);
    Serial.print('\n');
  }
}

void Set_AGC_tresshold(uint8_t val)
{
  if (val == 0)
  {
    Set_Cmd(32, 11, 1, 920);
    Set_Cmd(33, 11, 1, 1020);
  }
  if (val == 1)
  {
    Set_Cmd(32, 11, 1, 900);
    Set_Cmd(33, 11, 1, 1000);
  }
  if (val == 2)
  {
    Set_Cmd(32, 11, 1, 890);
    Set_Cmd(33, 11, 1, 990);
  }
  if (val == 3)
  {
    Set_Cmd(32, 11, 1, 880);
    Set_Cmd(33, 11, 1, 980);
  }
  if (val == 4)
  {
    Set_Cmd(32, 11, 1, 870);
    Set_Cmd(33, 11, 1, 970);
  }
  if (val == 5)
  {
    Set_Cmd(32, 11, 1, 840);
    Set_Cmd(33, 11, 1, 940);
  }
  else
  {
    Set_Cmd(32, 11, 1, 890); // GPIO not present in this version ...
    Set_Cmd(33, 11, 1, 990);
  }
}

void Set_LevelOffset(uint8_t val, uint8_t val1)
{
  if (val == 1 && val1 == 0)
  {
    Set_Cmd(32, 39, 1, 60);
    Set_Cmd(33, 39, 1, 60);
  }
  else if (val == 0 && val1 == 1)
  {
    Set_Cmd(32, 39, 1, 90);
    Set_Cmd(33, 39, 1, 90);
  }
  else if (val == 1 && val1 == 1)
  {
    Set_Cmd(32, 39, 1, 150);
    Set_Cmd(33, 39, 1, 150);
  }
  else
  {
    Set_Cmd(32, 39, 1, 0);
    Set_Cmd(33, 39, 1, 0);
  }
}

void Set_Deemphasis(uint8_t val)
{
  if (val == 0)
  {
    Set_Cmd(32, 31, 1, 750); // 75 uS
    Set_Cmd(32, 85, 1, 0);
  }
  else if (val == 1)
  {
    Set_Cmd(32, 31, 1, 500); // 50 uS
    Set_Cmd(32, 85, 1, 0);
  }
  else if (val == 2)
  {
    Set_Cmd(32, 31, 1, 0); //  0 uS (FLAT)
    Set_Cmd(32, 85, 1, 0);
  }
  else if (val == 3)
  {
    Set_Cmd(32, 31, 1, 0);
    Set_Cmd(32, 85, 1, 1); //  MPX
  }
  else if (val == 4)
  {
    Set_Cmd(32, 31, 1, 500);
    Set_Cmd(32, 85, 1, 1); //  MPX+ ( or Digital option not present in this sketch ! )
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(40);
  int16_t uState;
  Get_Cmd(64, 128, &uState, 1);
  if (uState < 2)
    // dsp_write_data(DSP_INIT_F8602); // uncomment your option
    dsp_write_data(DSP_INIT_F8605); //
  else if (uState > 2)
  {
    Set_Cmd(64, 1, 1, 1);
  }

  while (true)
  {
    if (Serial.available() > 0)
    {
      if (Serial.read() == 'x')
      {
        while (!Serial.available())
          ;
        if (Serial.read() == '\n')
          break;
      }
    }
  }
  delay(20);
  while (Serial.available() > 0)
    Serial.read(); // clear the serial buffer
  Serial.print("\nOK\n");
  if (AM_VOL_SCALE != -1)
  {
    int SetVolScale = map(AM_VOL_SCALE, 0, 100, -120, 60);
    Set_Cmd(33, 80, 1, SetVolScale);
  }
}

void loop()
{
  // check signal level and 19kHz subcarrier every TIMER_INTERVAL
  if ((millis() - timer) >= TIMER_INTERVAL)
  {
    Serial.print("S");
    if (radio_mode == 0)
    {
      int16_t uStatus;
      Get_Cmd(32, 133, &uStatus, 1);
      if (forced_mono == 1)
      {
        Serial.print('M');
      }
      else
      {
        char stereo = (uStatus & (1 << 15)) ? 's' : 'm';
        Serial.print(stereo);
      }
    }
    else
    {
      Serial.print('m');
    }

    int16_t uQuality[4] = {0};
    if (radio_mode == 0)
    {
      Get_Cmd(32, 128, uQuality, 6);
    }
    else
    {
      Get_Cmd(33, 128, uQuality, 4);
    }
    if (uQuality[1] > 1200)
    {
      uQuality[1] = 1200;
    }
    Serial.print(uQuality[1] / 10, DEC);
    Serial.print(',');
    if ((uQuality[2] > 1000) && (radio_mode == 0))
    {
      uQuality[2] = 1000;
    }
    if (radio_mode == 1)
    {
      uQuality[2] = uQuality[2] / 5;
    }
    Serial.print(uQuality[2] / 10, DEC);
    Serial.print(',');
    Serial.print(uQuality[3] / 10, DEC);
    Serial.print('\n');
    timer = millis();
  }

  if (radio_mode == 0)
  {
    if ((millis() - timer_DATA) >= DATA_TIMER_INTERVAL)
    {
      get_DATA();
      timer_DATA = millis();
    }
  }

  if (Serial.available() > 0)
  {
    buff[buff_pos] = Serial.read();
    if (buff[buff_pos] != '\n' && buff_pos != SERIAL_BUFFER_SIZE - 1)
      buff_pos++;
    else
    {
      buff[buff_pos] = 0x00;
      buff_pos = 0;

      switch (buff[0])
      {

      case 'x':
        Serial.println("OK");
        break;

      case 'Y': // Audio volume scaler
        volume = atoi(buff + 1);
        if (volume == 0)
        {
          Set_Cmd(48, 10, 1, 0); // 0 dB
          Set_Cmd(48, 11, 1, 0); // unmute
          Set_Cmd(48, 12, 1, 0);
        }
        else
        {
          int SetVolume = map(volume, 0, 100, -600, 30);
          Set_Cmd(48, 10, 1, SetVolume);
          // Set_Cmd(48, 10, 1, 20);  // +2 dB
          Set_Cmd(48, 11, 1, 0); // unmute
          Set_Cmd(48, 12, 1, 0);
        }
        Serial.print("Y");
        Serial.println(volume);
        break;

      case 'S':
        if (buff[1] == 'a')
        {
          scan_start = (atol(buff + 2) + 5) / 10;
          AM_start_scan = atol(buff + 2);
        }
        else if (buff[1] == 'b')
        {
          scan_end = (atol(buff + 2) + 5) / 10;
          AM_scan_end = atol(buff + 2);
        }
        else if (buff[1] == 'c')
        {
          scan_step = (atol(buff + 2) + 5) / 10;
          AM_scan_step = atol(buff + 2);
        }
        else if (buff[1] == 'f')
        {
          scan_filter = atol(buff + 2);
        }
        else if (scan_start > 0 && scan_end > 0 && scan_step > 0 && scan_filter >= 0)
        {
          if (buff[1] == 'm')
            scan(true); // Multiple (continous) scan
          else
            scan(false); // Single scan
        }
        break;

      case 'B':
        forced_mono = atol(buff + 1);
        if (forced_mono == 1)
          Set_Cmd(32, 66, 1, 2, forced_mono == 1 ? 2 : 0, 0);
        else
          Set_Cmd(32, 66, 1, 0, forced_mono == 1 ? 2 : 0, 180);
        Serial.print('B');
        Serial.print(forced_mono ? '1' : '0');
        Serial.print('\n');
        break;

      case 'M':
        mode = atol(buff + 1);
        if (mode == 0)
        {
          if (MODF_FREQ == 0)
          {
            MODF_FREQ = 87600;
          }
          Set_Cmd(32, 1, 2, 1, MODF_FREQ / 10);
          Serial.println("M0");
          Serial.print('T');
          Serial.println(MODF_FREQ);
          if (Filter_AM != 16)
          {
            Filter_AM = 15;
          }
          if (Filter_FM == 16)
          {
            Filter_FM = -1;
          }
          current_filter = Filter_FM;
          Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
          radio_mode = 0;
        }
        else
        {
          if (MODA_FREQ == 0)
          {
            MODA_FREQ = 855;
          }
          Set_Cmd(33, 1, 2, 1, MODA_FREQ);
          Serial.println("M1");
          Serial.print('T');
          Serial.println(MODA_FREQ);
          Filter_FM = current_filter;
          if (Filter_FM != 16)
          {
            Filter_FM = current_filter;
          }
          if (Filter_AM == 16)
          {
            Filter_AM = 15;
          }
          current_filter = Filter_AM;
          Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
          radio_mode = 1;
        }
        Serial.print('F');
        Serial.println(current_filter);
        Serial.print('V');
        Serial.println(LevelOffset);
        break;

      case 'T': // Tune to a frequency
        freq = atol(buff + 1);
        REG_FREQ = freq;
        if ((REG_FREQ >= 65000) && (REG_FREQ <= 108000))
        {
          Set_Cmd(32, 1, 2, 1, REG_FREQ / 10);
          Serial.println("M0");
          Serial.print('T');
          Serial.println(REG_FREQ);
          if (radio_mode == 1)
          {
            if (Filter_AM != 16)
            {
              Filter_AM = 16;
            }
            if (Filter_FM == 16)
            {
              Filter_FM = -1;
            }
            current_filter = Filter_FM;
            Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
          }
          radio_mode = 0;
          MODF_FREQ = REG_FREQ;
        }
        else if ((REG_FREQ >= 144) && (REG_FREQ <= 26999))
        {
          Set_Cmd(33, 1, 2, 1, REG_FREQ);
          Serial.println("M1");
          Serial.print('T');
          Serial.println(REG_FREQ);
          if (radio_mode == 0)
          {
            if (Filter_FM != 16)
            {
              Filter_FM = -1;
            }
            if (Filter_AM == 16)
            {
              Filter_AM = 16;
            }
            current_filter = Filter_AM;
            Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
          }
          radio_mode = 1;
          MODA_FREQ = REG_FREQ;
        }
        Serial.print('F');
        Serial.println(current_filter);
        Serial.print('V');
        Serial.println(LevelOffset);
        break;

      case 'F': // Change FIR filters
        current_filter = atoi(buff + 1);
        if (radio_mode == 0)
        {
          Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
        }
        else
        {
          Set_Cmd(33, 10, 2, 0, pgm_read_byte_near(AMFilterMap + current_filter));
        }
        Serial.print('F');
        Serial.println(buff + 1);
        break;

      case 'V':
        LevelOffset = atoi(buff + 1);
        LevelAM = map(LevelOffset, 0, 127, 0, 360);
        if (radio_mode == 1)
        {
          Set_Cmd(33, 12, 1, LevelAM);
        } // Set AM LNA gain reduction ( enable "Show antenna input alignment" in NXP-GTK  )
        else
          LevelAtt = atoi(buff + 1);
        AttFM = map(LevelAtt, 0, 127, 0, 360);
        if (radio_mode == 0)
        {
          Set_Cmd(32, 12, 1, AttFM);
        } // Set FM LNA gain reduction ( enable "Show antenna input alignment" in NXP-GTK  )
        Serial.print('V');
        Serial.println(buff + 1);
        break;

      case 'Q': // Audio volume scaler
        Squelch = atoi(buff + 1);
        if (Squelch != -1 && AM_VOL_SCALE == -1)
        {
          Setsquelch = map(Squelch, 0, 100, -120, 60);
          Set_Cmd(33, 80, 1, Setsquelch);
          Serial.print("Q");
          Serial.println(Squelch);
        }
        break;

      case 'D': // Change the de-emphasis
        nDeemphasis = atoi(buff + 1);
        Serial.print("D");
        Set_Deemphasis(nDeemphasis);
        Serial.println(nDeemphasis);
        break;

      case 'G':
        Serial.print('G');
        RF1plus = buff[1] - 48;
        RF2plus = buff[2] - 48;
        Set_LevelOffset(RF1plus, RF2plus);
        Serial.print(RF1plus);
        Serial.println(RF2plus);
        break;

      case 'A': // AGC  (check what levelstepping does)
        Serial.print('A');
        AGC_tress = atoi(buff + 1);
        Set_AGC_tresshold(AGC_tress);
        Serial.println(AGC_tress);
        break;

      case 'C':
        if (buff[1] == '0')
        {
          digitalWrite(RELAY_CW_PIN, LOW);
          digitalWrite(RELAY_CCW_PIN, LOW);
          Serial.print("Relays OFF\n");
        }
        else if (buff[1] == '1')
        {
          digitalWrite(RELAY_CW_PIN, HIGH);
          digitalWrite(RELAY_CCW_PIN, LOW);
          Serial.print("Relay CW\n");
        }
        else if (buff[1] == '2')
        {
          digitalWrite(RELAY_CW_PIN, LOW);
          digitalWrite(RELAY_CCW_PIN, HIGH);
          Serial.print("Relay CCW\n");
        }
        Serial.print('C');
        Serial.println(buff + 1);
        break;

      case 'Z':
        Serial.print('Z');
        if (buff[1] == '0')
        {
          dsp_write_data(INIT_1);
          Serial.print("0");
        }
        else if (buff[1] == '1')
        {
          dsp_write_data(INIT_2);
          Serial.print("1");
        }
        else if (buff[1] == '2')
        {
          dsp_write_data(INIT_3);
          Serial.print("2");
        }
        else if (buff[1] == '3')
        {
          dsp_write_data(INIT_4);
          Serial.print("3");
        }
        else
        {
          Serial.print("4");
        }
        Serial.print('\n');

        Set_AGC_tresshold(AGC_tress);
        Set_LevelOffset(RF1plus, RF2plus);
        Set_Deemphasis(nDeemphasis);
        break;

      case 'X': // shutdown
        Set_Cmd(64, 1, 1, 1);
        TWCR = 0; // Release SDA and SCL lines used by hardware I2C
        Serial.println("X");
        delay(10);
        asm("jmp 0");
        break;
      }
    }
  }
}