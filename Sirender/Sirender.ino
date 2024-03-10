/**
 * Sirender
 *  緊急車両サイレン検知
 *  1st author : @yagikjp  2024.03.02
 *
 * サイレン仕様参考情報
 *  https://www.city.hekinan.lg.jp/material/files/group/47/inu.pdf
 *  救急車：960Hz 0.65S, 770Hz 0.65S
 *  救急車：850Hz 0.65S, 680Hz 0.65S
 *  消防車：850Hz ~ 300Hz
 *  パトカー：870Hz 8S
 */

#include <M5StickCPlus.h>
#include <driver/i2s.h>
#include <string.h>
#include "AXP192.h"
#include "arduinoFFT.h"

#define PIN_CLK            0
#define PIN_DATA          34

#define FFTsamples       512
#define SAMPLING_RATE  11025

#define NUM_RING         200

typedef struct type_analyze {
  unsigned long   ms;
  int             ambulance_960;
  int             ambulance_770;
  int             ambulance_850;
  int             ambulance_680;
  int             fire_850;
  int             fire_300;
  int             police_870;
} Analyze;

enum IS_SIREN {
  IS_NOT_SIREN,
  IS_AMBULANCE,
  IS_FIRE_ENGINE,
  IS_POLICE_CAR
};

unsigned long head  = 0;
int           idx   = 0;
Analyze       ring_analyze[NUM_RING];

int16_t       adcBuffer [FFTsamples] = {0};
double        vReal     [FFTsamples];          // サンプリングデータ
double        vImag     [FFTsamples];
ArduinoFFT<double>    FFT = ArduinoFFT<double>(vReal, vImag, FFTsamples, SAMPLING_RATE);


//  FFTグラフ描画
void drawFFTresult() {
  int   y, y0, f, t;

  M5.Lcd.fillScreen(WHITE);
  for(int n = 2; n < FFTsamples / 2; n++) {
    y = map(vReal[n], 0, 10000, 0, M5.Lcd.height());
    if(2 < n) {
      if(y0 < y) {
        f = y0;
        t = y;
      } else {
        f = y;
        t = y0;
      }
      for(int i = f; i <= t; i++) {
        M5.Lcd.drawPixel((n * M5.Lcd.width() * 2) / FFTsamples, M5.Lcd.height() - i, BLACK);
      }
    }
    y0 = y;
  }
}

//  I2S初期化
void i2sInit() {
  i2s_config_t i2s_config = {
      .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate          = SAMPLING_RATE,
      .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,    // is fixed at 12bit, stereo, MSB
        .channel_format       = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
      .communication_format = I2S_COMM_FORMAT_I2S,
#endif
      .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count        = 2,
      .dma_buf_len          = 128,
  };

  i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
  pin_config.mck_io_num   = I2S_PIN_NO_CHANGE;
#endif

  pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num    = PIN_CLK;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num  = PIN_DATA;

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, SAMPLING_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

//  ピーク周波数取得
double getPeak(double *data, int n, double cycle) {
  int     i, j = -1;
  double  max = -1, hz = 0;

  for(i = 2; i < n / 2; i++) {
    if(max < data[i]) {
      j   = i;
      max = data[i];
    }
  }

  if(0 < j) {
    hz = cycle / n * j;
  }

  return  hz;
}

//  ピーク有無チェック
bool check_peak_sub(double *data, int n, double cycle, double hz) {
  bool    ret = false;
  int     i = (int)(hz / cycle * n + 0.5);

//  printf("%d ", i);

  if((2 * data[i - 2] < data[i]) && (data[i] > 2 * data[i + 2])) {
    ret = true;
  }

  return  ret;
}

//  サイレン波長チェック
void check_peak(double *data, int n, double cycle, Analyze *analyze) {
  if(check_peak_sub(data, n, cycle, 960))   analyze->ambulance_960 = 1;
  if(check_peak_sub(data, n, cycle, 770))   analyze->ambulance_770 = 1;
  if(check_peak_sub(data, n, cycle, 850))   analyze->ambulance_850 = 1;
  if(check_peak_sub(data, n, cycle, 680))   analyze->ambulance_680 = 1;
  if(check_peak_sub(data, n, cycle, 850))   analyze->fire_850      = 1;
  if(check_peak_sub(data, n, cycle, 300))   analyze->fire_300      = 1;
  if(check_peak_sub(data, n, cycle, 870))   analyze->police_870    = 1;
}

//  サイレン判別
int check_siren() {
  int     ret = IS_NOT_SIREN;

  if((NUM_RING < head) || (1500 < millis() - ring_analyze[0].ms)) {
    unsigned long   h = head, ms;
    int             i, j, n = 0;
    int             s960 = 0, s770 = 0, x960 = 0;
    int             s850 = 0, s680 = 0, x850 = 0;

    i = h % NUM_RING;
    ms = ring_analyze[i].ms;
    for(j = 0; j < NUM_RING; j++, h--, n++) {
      i = h % NUM_RING;
      if(1300 < ms - ring_analyze[i].ms) {
        break;
      }
      s960 = s960 +  ring_analyze[i].ambulance_960;
      s770 = s770 +  ring_analyze[i].ambulance_770;
      x960 = x960 + (ring_analyze[i].ambulance_960 & ring_analyze[i].ambulance_770);

      s850 = s850 +  ring_analyze[i].ambulance_850;
      s680 = s680 +  ring_analyze[i].ambulance_680;
      x960 = x960 + (ring_analyze[i].ambulance_850 & ring_analyze[i].ambulance_680);
    }

//    printf(" %2d %2d %2d %2d ", s960, s770, x960, n);
    if((n * 0.4 < s960) && (s960 < n * 0.8)
    && (n * 0.4 < s770) && (s770 < n * 0.8)
    && (x960 < n * 0.5)) {
      ret = IS_AMBULANCE;
//      printf(" ambulance\t");
    }
    else
    if((n * 0.4 < s850) && (s850 < n * 0.8)
    && (n * 0.4 < s680) && (s680 < n * 0.8)
    && (x850 < n * 0.5)) {
      ret = IS_AMBULANCE;
//      printf(" ambulance\t");
    }
  }

  return  ret;
}

void setup() {
//  void begin(bool LCDEnable=true, bool PowerEnable=true, bool SerialEnable=true);
  M5.begin(true, true, true);
  M5.Lcd.setRotation(1);

  i2sInit();      //  マイク初期化
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  //  FFT窓関数

  delay(1000);    //  マイク初期化待ち
}

void loop() {
  static unsigned long  ms_disp = 0, ms_not_usb = 0;
  static int            siren, n, disp_mode = -1;
  float                 vin;
  size_t                bytesread;

  idx = head % NUM_RING;
  memset(&ring_analyze[idx], 0, sizeof(Analyze));
  ring_analyze[idx].ms = millis();
  i2s_read(I2S_NUM_0, (char *)adcBuffer, FFTsamples * 2, &bytesread, portMAX_DELAY);

  for(n = 0; n < FFTsamples; n++) {
    vReal[n] = adcBuffer[n];
  }
  memset(vImag, 0, sizeof(double) * FFTsamples);

  FFT.compute(FFTDirection::Forward);   //  FFT
  FFT.complexToMagnitude();   //  実数変換

  check_peak(vReal, FFTsamples, SAMPLING_RATE, &ring_analyze[idx]);

//  printf("%lu\t", ring_analyze[idx].ms);
//  printf("%d " , ring_analyze[idx].ambulance_960);
//  printf("%d " , ring_analyze[idx].ambulance_770);
//  printf("%d " , ring_analyze[idx].ambulance_850);
//  printf("%d " , ring_analyze[idx].ambulance_680);
//  printf("%d " , ring_analyze[idx].fire_850);
//  printf("%d " , ring_analyze[idx].fire_300);
//  printf("%d " , ring_analyze[idx].police_870);

  siren = check_siren();
  switch(siren) {
    case IS_AMBULANCE:
      ms_disp = millis();
      if(disp_mode != siren) {
        disp_mode = siren;
        M5.Lcd.fillScreen(RED);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setTextColor(BLACK, RED);
        M5.Lcd.setCursor(10, 60);
        M5.Lcd.println("AMBULANCE");
      }
      break;

    default:
      if(3000 < millis() - ms_disp) {
        if(disp_mode != siren) {
          disp_mode = siren;
          M5.Lcd.fillScreen(BLACK);
          M5.Lcd.setTextSize(4);
          M5.Lcd.setTextColor(BLUE, BLACK);
          M5.Lcd.setCursor(0, 60);
          M5.Lcd.println(" Sirender");
        }
      }
  }

  vin = M5.Axp.GetVBusVoltage();
  if(vin < 3) {
    if(ms_not_usb == 0) {
      ms_not_usb = millis();

      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(10, 60);
      M5.Lcd.println("Power OFF");
    } else if(3000 < millis() - ms_not_usb) {
      M5.Axp.PowerOff();
    }
  } else if(0 < ms_not_usb) {
    ms_disp     =  0;
    disp_mode   = -1;
    ms_not_usb  = 0;
  }

//  printf("\n");
//  drawFFTresult();

  head++;
  delay(1);
}
