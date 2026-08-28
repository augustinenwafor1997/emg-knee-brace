/*
  Brace EMG — ESP32-S3 dual MyoWare sampler
  ---------------------------------------------------------------
  Reads two MyoWare RAW outputs (QUAD + HAM) at a hardware-timed
  1000 Hz and streams them to the Raspberry Pi over USB serial as
  newline-delimited ASCII:  "seq,micros,quad,ham\n"

  Timing is owned by a hardware timer + ISR, NOT by the Pi, so the
  sample rate is exact regardless of what Linux is doing. Each line
  carries a monotonic sample index (seq) — the reliable time base,
  since seq * 1 ms is glitch-free and gaps in seq are the only true
  measure of dropped samples. micros() is included only as a
  diagnostic (it can jump by whole seconds on some ESP32 cores).

  ---------------------------------------------------------------
  WIRING  (do this exactly — the 3.3 V rule is not optional)
  ---------------------------------------------------------------
    MyoWare (QUAD)  RAW  -> ESP32  QUAD_PIN
    MyoWare (HAM)   RAW  -> ESP32  HAM_PIN
    MyoWare  GND         -> ESP32  GND   (common ground REQUIRED)
    MyoWare  Vin/+       -> 3.3 V  (from its battery power shield)

    >>> Power each MyoWare at 3.3 V, NOT 5 V. <<<
    The RAW output is centered at Vcc/2 and swings toward the rails.
    At 5 V it can exceed 3.3 V and damage the ESP32 ADC input.
    At 3.3 V it stays inside the ESP32's 0-3.3 V ADC range.

    Sample the RAW pad (unprocessed amplified EMG). Confirm the
    silkscreen on your MyoWare revision — it is labeled RAW on both
    MyoWare 1.0 and 2.0.

  ---------------------------------------------------------------
  ARDUINO IDE SETTINGS  (ESP32-S3)
  ---------------------------------------------------------------
    Board:              "ESP32S3 Dev Module"
    USB CDC On Boot:    "Enabled"   <-- REQUIRED so Serial = USB CDC
    Core:               Arduino-ESP32 v3.x (new timer API used below)

    On a classic ESP32 (no native USB): the code still works over the
    UART bridge. Set QUAD_PIN/HAM_PIN to ADC1 pins (e.g. 34/35) below.
*/

// -------------------- Config --------------------
#define QUAD_PIN   1     // ESP32-S3: GPIO1  (ADC1_CH0).  Classic ESP32: use 34
#define HAM_PIN    2     // ESP32-S3: GPIO2  (ADC1_CH1).  Classic ESP32: use 35

static const uint32_t SAMPLE_HZ = 1000;      // per-channel sample rate
static const uint32_t SERIAL_BAUD = 921600;  // ignored by native USB CDC; used by UART bridge
static const int      QUEUE_LEN   = 5000;     // ~5 s of backup — absorbs multi-second host stalls (GC, OS hiccups)
// ------------------------------------------------

struct __attribute__((packed)) Sample {
  uint32_t seq;    // monotonic sample index — the reliable time base (seq * 1 ms)
  uint32_t t;      // micros() at read time — diagnostic cross-check only (it can glitch)
  uint16_t quad;   // 12-bit ADC count, QUAD (thigh extensor)
  uint16_t ham;    // 12-bit ADC count, HAM  (hamstring flexor)
};

hw_timer_t       *timer = NULL;
SemaphoreHandle_t timerSemaphore;
QueueHandle_t     dataQueue;

// ISR: fires every 1/SAMPLE_HZ second, just releases the ADC task.
void IRAM_ATTR onTimer() {
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(timerSemaphore, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

// Core 1: precise acquisition. Reads both channels back-to-back
// (~tens of microseconds apart) so inter-channel skew is negligible.
void ADC_Task(void *pv) {
  Sample s;
  uint32_t seq = 0;
  for (;;) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
      s.seq  = seq++;
      s.t    = micros();
      s.quad = (uint16_t)analogRead(QUAD_PIN);
      s.ham  = (uint16_t)analogRead(HAM_PIN);
      // Drop rather than block if the Pi stalls — preserves timing integrity.
      xQueueSend(dataQueue, &s, 0);
    }
  }
}

// Core 0: streams the queue out as ASCII lines. Newline framing lets
// the Pi resync instantly if a byte is ever lost.
void Serial_Task(void *pv) {
  Sample s;
  char line[48];
  for (;;) {
    if (xQueueReceive(dataQueue, &s, portMAX_DELAY) == pdTRUE) {
      int n = snprintf(line, sizeof(line), "%lu,%lu,%u,%u\n",
                       (unsigned long)s.seq, (unsigned long)s.t, s.quad, s.ham);
      if (n > 0) Serial.write((const uint8_t *)line, n);
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  analogReadResolution(12);            // 0..4095, matches existing dataset
  analogSetAttenuation(ADC_11db);      // full 0-3.3 V input range

  timerSemaphore = xSemaphoreCreateBinary();
  dataQueue      = xQueueCreate(QUEUE_LEN, sizeof(Sample));

  xTaskCreatePinnedToCore(ADC_Task,    "ADC_Task",    4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Serial_Task, "Serial_Task", 4096, NULL, 2, NULL, 0);

  // Hardware timer: 1 MHz tick base, alarm every (1e6 / SAMPLE_HZ) ticks, auto-reload.
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / SAMPLE_HZ, true, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));     // all work happens in the tasks
}
