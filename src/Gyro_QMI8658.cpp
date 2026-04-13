#include "Gyro_QMI8658.h"
#include "PWR_Key.h"
#include "BAT_Driver.h"

IMUdata Accel;
IMUdata Gyro;

uint8_t Device_addr ; // default for SD0/SA0 low, 0x6A if high
acc_scale_t acc_scale = ACC_RANGE_2G;
gyro_scale_t gyro_scale = GYR_RANGE_64DPS;
acc_odr_t acc_odr = acc_odr_norm_30;    // 30 Hz — sufficient for step detection (200ms debounce)
gyro_odr_t gyro_odr = gyro_odr_norm_30;
sensor_state_t sensor_state = sensor_default;
lpf_t acc_lpf;

float accelScales, gyroScales;
uint8_t readings[12];
uint32_t reading_timestamp_us; // timestamp in arduino micros() time
static bool s_imuReady = false;

// Define variables
int stepCount = 0;           // Variable to store step count
float distanceTraveled = 0;  // Distance in meters
float caloriesBurned = 0;    // Calories
static float s_dbgMag = 0.0f;
static float s_dbgEnv = 0.0f;
static float s_dbgJerk = 0.0f;
static bool s_haveValidAccelSample = false;
static uint8_t s_zeroVectorStreak = 0;
static unsigned long s_lastImuRecoverMs = 0;
static volatile bool s_shakeDetected = false;

static void logImuConfigRegisters(const char *tag) {
  if (!s_imuReady) return;
  uint8_t ctrl1 = QMI8658_receive(QMI8658_CTRL1);
  uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
  uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
  uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
  uint8_t ctrl6 = QMI8658_receive(QMI8658_CTRL6);
  uint8_t ctrl7 = QMI8658_receive(QMI8658_CTRL7);
  printf("QMI8658 %s CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL5=0x%02X CTRL6=0x%02X CTRL7=0x%02X\r\n",
         tag ? tag : "cfg", ctrl1, ctrl2, ctrl3, ctrl5, ctrl6, ctrl7);
}

static bool shouldLogImuDebug() {
  // Keep verbose IMU telemetry while charging/debugging but suppress it on
  // battery to reduce CPU/serial overhead.
  return BAT_Is_Charging();
}

bool detectStep() {
  static float magBaseline = 1.0f;       // tracks gravity baseline
  static float motionEnvelope = 0.0f;    // smoothed high-pass magnitude
  static bool overThreshold = false;     // edge detector state
  static unsigned long lastStepMs = 0;
  static float lastX = 0.0f, lastY = 0.0f, lastZ = 0.0f;
  static bool initialized = false;

  // Tuned more sensitive for lighter arm swing and shorter indoor steps.
  const unsigned long debounceMs = 220;
  const float envelopeThreshold = 0.035f; // |mag - baseline| in g
  const float jerkThreshold = 0.10f;      // sum |dx|+|dy|+|dz| per sample

  float mag = sqrtf(Accel.x * Accel.x + Accel.y * Accel.y + Accel.z * Accel.z);

  if (!initialized) {
    lastX = Accel.x;
    lastY = Accel.y;
    lastZ = Accel.z;
    magBaseline = (mag > 0.05f) ? mag : 1.0f;
    initialized = true;
    return false;
  }

  // Treat near-zero vectors as invalid samples (bus/config fault or stale data)
  // and avoid startup false-positive steps when magnitude collapses from 1g to 0.
  if (mag < 0.05f) {
    s_dbgMag = mag;
    s_dbgJerk = 0.0f;
    overThreshold = false;
    return false;
  }

  float jerk = fabsf(Accel.x - lastX) + fabsf(Accel.y - lastY) + fabsf(Accel.z - lastZ);
  lastX = Accel.x;
  lastY = Accel.y;
  lastZ = Accel.z;

  // Slow baseline tracks gravity/orientation drift, not steps.
  magBaseline = (0.98f * magBaseline) + (0.02f * mag);
  float hp = fabsf(mag - magBaseline);

  // Keep some smoothing, but let the envelope react faster to lighter steps.
  motionEnvelope = (0.65f * motionEnvelope) + (0.35f * hp);
  s_dbgMag = mag;
  s_dbgEnv = motionEnvelope;
  s_dbgJerk = jerk;

  bool nowOver = motionEnvelope > envelopeThreshold;
  bool jerkHit = jerk > jerkThreshold &&
                 motionEnvelope > (envelopeThreshold * 0.35f) &&
                 mag > 0.65f && mag < 2.80f;
  unsigned long now = millis();
  bool stepped = false;
  if (((nowOver && !overThreshold) || jerkHit) && (now - lastStepMs > debounceMs)) {
    lastStepMs = now;
    stepped = true;
  }
  overThreshold = nowOver;
  return stepped;
}

static bool detectShake() {
  static float accelEnvelope = 0.0f;
  static float jerkEnvelope = 0.0f;
  static unsigned long lastShakeMs = 0;
  static bool initialized = false;

  const float accelHighPass = fabsf(s_dbgMag - 1.0f);
  const unsigned long now = millis();
  const unsigned long shakeCooldownMs = 3000;

  if (!initialized) {
    initialized = true;
    return false;
  }

  accelEnvelope = (0.35f * accelEnvelope) + (0.65f * accelHighPass);
  jerkEnvelope = (0.30f * jerkEnvelope) + (0.70f * s_dbgJerk);

  const bool moved = accelEnvelope > 0.08f;
  const bool jerked = jerkEnvelope > 0.16f;
  const bool strongAccel = accelEnvelope > 0.16f;

  if (((moved && jerked) || strongAccel) &&
      (now - lastShakeMs >= shakeCooldownMs)) {
    lastShakeMs = now;
    return true;
  }

  return false;
}

static bool detectMotionActivity() {
  static float motionEnvelope = 0.0f;
  static unsigned long lastMotionMs = 0;

  const unsigned long now = millis();
  const float activitySignal = (s_dbgEnv * 1.6f) + (s_dbgJerk * 0.55f);

  motionEnvelope = (0.60f * motionEnvelope) + (0.40f * activitySignal);

  if (motionEnvelope > 0.07f && (now - lastMotionMs) >= 300UL) {
    lastMotionMs = now;
    return true;
  }

  return false;
}

/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 */
void QMI8658_Init(void)
{
    uint8_t whoAmI = 0;
    uint8_t revision = 0;
    s_imuReady = false;

    // Probe both possible I2C addresses and validate WHO_AM_I.
    const uint8_t candidates[] = { QMI8658_L_SLAVE_ADDRESS, QMI8658_H_SLAVE_ADDRESS };
    for (size_t i = 0; i < sizeof(candidates); i++) {
        uint8_t addr = candidates[i];
        if (I2C_Read(addr, QMI8658_WHO_AM_I, &whoAmI, 1) == ESP_OK &&
            whoAmI != 0x00 && whoAmI != 0xFF) {
            Device_addr = addr;
            I2C_Read(Device_addr, QMI8658_REVISION_ID, &revision, 1);
            s_imuReady = true;
            break;
        }
    }

    if (!s_imuReady) {
        printf("QMI8658 not detected at 0x%02X or 0x%02X\r\n",
               QMI8658_L_SLAVE_ADDRESS, QMI8658_H_SLAVE_ADDRESS);
        return;
    }

    printf("QMI8658 detected at 0x%02X (WHO_AM_I=0x%02X, REV=0x%02X)\r\n",
           Device_addr, whoAmI, revision);

    s_haveValidAccelSample = false;
    s_zeroVectorStreak = 0;
    s_lastImuRecoverMs = 0;

    setState(sensor_running);             

    setAccScale(acc_scale);            
    setAccODR(acc_odr);                    
    setAccLPF(LPF_MODE_0);                  
    switch (acc_scale) {                
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case ACC_RANGE_2G:  accelScales = 2.0 / 32768.0; break;
        case ACC_RANGE_4G:  accelScales = 4.0 / 32768.0; break;
        case ACC_RANGE_8G:  accelScales = 8.0 / 32768.0; break;
        case ACC_RANGE_16G: accelScales = 16.0 / 32768.0; break;
    }

    setGyroScale(gyro_scale);              
    setGyroODR(gyro_odr);                       
    setGyroLPF(LPF_MODE_3);                
    switch (gyro_scale) {                  
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case GYR_RANGE_16DPS: gyroScales = 16.0 / 32768.0; break;
        case GYR_RANGE_32DPS: gyroScales = 32.0 / 32768.0; break;
        case GYR_RANGE_64DPS: gyroScales = 64.0 / 32768.0; break;
        case GYR_RANGE_128DPS: gyroScales = 128.0 / 32768.0; break;
        case GYR_RANGE_256DPS: gyroScales = 256.0 / 32768.0; break;
        case GYR_RANGE_512DPS: gyroScales = 512.0 / 32768.0; break;
        case GYR_RANGE_1024DPS: gyroScales = 1024.0 / 32768.0; break;
    }

    logImuConfigRegisters("init");
}

void QMI8658_Loop(void)
{
  if (!s_imuReady) {
    static unsigned long lastMissingLog = 0;
    if (millis() - lastMissingLog >= 5000UL) {
      lastMissingLog = millis();
      Serial.println("[IMU] not ready - no sensor data");
    }
    return;
  }

  bool displayAwake = PWR_IsDisplayAwake();

  // Keep accelerometer in normal 30 Hz mode in both awake/sleep states.
  // Low-power ODR modes proved unreliable for step detection on this hardware.
  static bool odrForced = false;
  if (!odrForced) {
    setAccODR(acc_odr_norm_30);
    odrForced = true;
    Serial.println(">> IMU ODR forced: 30Hz normal (step reliability)");
  }

  // Throttle IMU reads when display is off, but keep cadence high enough to
  // capture short acceleration peaks used by threshold-based step detection.
  if (!displayAwake) {
    static unsigned long lastSleepRead = 0;
    if (millis() - lastSleepRead < 100) return;  // 10Hz when sleeping
    lastSleepRead = millis();
  }

  getAccelerometer();
  getGyroscope();

  // static unsigned long lastImuDebugLog = 0;
  // if (shouldLogImuDebug() && millis() - lastImuDebugLog >= 5000UL) {
  //   lastImuDebugLog = millis();
  //   Serial.printf("[IMU] ax=%.3f ay=%.3f az=%.3f mag=%.3f env=%.3f jerk=%.3f steps=%d awake=%d\n",
  //                 Accel.x, Accel.y, Accel.z, s_dbgMag, s_dbgEnv, s_dbgJerk,
  //                 stepCount, displayAwake ? 1 : 0);
  // }

  if (!s_haveValidAccelSample) return;

  if (detectMotionActivity()) {
    PWR_UpdateActivity();
  }

  if (detectStep()) {
    // Keep step counting independent of display wake policy.
    // Waking/resetting display idle on every step keeps the screen on while walking
    // and is a major battery drain.
  }

  if (!s_shakeDetected && detectShake()) {
    s_shakeDetected = true;
  }
}

bool QMI8658_ConsumeShakeEvent(void)
{
  bool detected = s_shakeDetected;
  s_shakeDetected = false;
  return detected;
}

/**
 * Transmit one uint8_t of data to QMI8658.
 * @param addr address of data to be written
 * @param data the data to be written
 */
void QMI8658_transmit(uint8_t addr, uint8_t data)
{
    I2C_Write(Device_addr, addr, &data, 1);
}

/**
 * Receive one uint8_t of data from QMI8658.
 * @param addr address of data to be read
 * @return the uint8_t of data that was read
 */
uint8_t QMI8658_receive(uint8_t addr)
{
    uint8_t retval;
    I2C_Read(Device_addr, addr, &retval, 1);
    return retval;
}

/**
 * Writes data to CTRL9 (command register) and waits for ACK.
 * @param command the command to be executed
 */
void QMI8658_CTRL9_Write(uint8_t command)
{
    // transmit command
    QMI8658_transmit(QMI8658_CTRL9, command);

    // wait for command to be done
    while (((QMI8658_receive(QMI8658_STATUSINT)) & 0x80) == 0x00);
}

/**
 * Set output data rate (ODR) of accelerometer.
 * @param odr acc_odr_t variable representing new data rate
 */
void setAccODR(acc_odr_t odr)
{
    if (sensor_state != sensor_default)                     // If the device is not in the default state
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK;                        // clear previous setting
        ctrl2 |= odr;                                       // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_odr = odr;
}

/**
 * Set output data rate (ODR) of gyro.
 * @param odr gyro_odr_t variable representing new data rate
 */
void setGyroODR(gyro_odr_t odr)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GODR_MASK; // clear previous setting
    ctrl3 |= odr; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_odr = odr;
}

/**
 * Set scale of accelerometer output.
 * @param scale acc_scale_t variable representing new scale
 */
void setAccScale(acc_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
    ctrl2 &= ~QMI8658_ASCALE_MASK; // clear previous setting
    ctrl2 |= scale << QMI8658_ASCALE_OFFSET; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_scale = scale;
}

/**
 * Set scale of gyro output.
 * @param scale gyro_scale_t variable representing new scale
 */
void setGyroScale(gyro_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GSCALE_MASK; // clear previous setting
    ctrl3 |= scale << QMI8658_GSCALE_OFFSET; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_scale = scale;
}

/**
 * Set new low-pass filter value for accelerometer
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setAccLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
    ctrl5 &= ~QMI8658_ALPF_MASK;
    ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
    ctrl5 |= 0x01; // turn on acc low pass filter
    QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
    acc_lpf = lpf;
}

/**
 * Set new low-pass filter value for gyro
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setGyroLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
    ctrl5 &= ~QMI8658_GLPF_MASK;
    ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
    ctrl5 |= 0x10; // turn on gyro low pass filter
    QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
}

/**
 * Set new state of QMI8658.
 * @param state new state to transition to
 */
void setState(sensor_state_t state)
{
    uint8_t ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= ~(0x80 | 0x01);
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // disable syncSample mode
        // NOTE: This board's QMI8658 variant can report all-zero accel data
        // when gyro is disabled in CTRL7 (0x41). Keep both enabled (0x43).
        QMI8658_transmit(QMI8658_CTRL7, 0x43);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        break;
    case sensor_power_down:
        // disable high speed internal clock,
        // acc and gyro powered down
        QMI8658_transmit(QMI8658_CTRL7, 0x00);

        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // disable 2MHz oscillator
        ctrl1|= 0x01;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        break;
    case sensor_locking:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= ~(0x80 | 0x01);
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // enable syncSample mode
        QMI8658_transmit(QMI8658_CTRL7, 0x83);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);

        // disable internal AHB clock gating:
        QMI8658_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658_CTRL9_Write(0x12);
        // re-enable clock gating
        QMI8658_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658_CTRL9_Write(0x12);
        break;
    default:
        break;
    }
    sensor_state = state;
}


void getAccelerometer(void)
{

  uint8_t buf[6];
	esp_err_t ret = I2C_Read(Device_addr, QMI8658_AX_L, buf, 6);
	if(ret != ESP_OK) {
		printf("QMI8658: Accelerometer read failure (addr=0x%02X)\r\n", Device_addr);
    return;
  }
	else{
    bool rawAllZero = true;
    for (size_t i = 0; i < 6; i++) {
      if (buf[i] != 0) {
        rawAllZero = false;
        break;
      }
    }
    if (rawAllZero) {
      static unsigned long lastRawZeroLog = 0;
      if (millis() - lastRawZeroLog >= 5000UL) {
        lastRawZeroLog = millis();
        uint8_t ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        uint8_t ctrl7 = QMI8658_receive(QMI8658_CTRL7);
        Serial.printf("[IMU] raw accel bytes all zero (CTRL1=0x%02X CTRL2=0x%02X CTRL7=0x%02X)\n",
                      ctrl1, ctrl2, ctrl7);
      }
    }
    Accel.x = (float)((int16_t)((buf[1]<<8) | (buf[0])));
    Accel.y = (float)((int16_t)((buf[3]<<8) | (buf[2])));
    Accel.z = (float)((int16_t)((buf[5]<<8) | (buf[4])));
    Accel.x = Accel.x * accelScales;
    Accel.y = Accel.y * accelScales;
    Accel.z = Accel.z * accelScales;

    float absSum = fabsf(Accel.x) + fabsf(Accel.y) + fabsf(Accel.z);
    if (absSum < 0.01f) {
      if (s_zeroVectorStreak < 255) s_zeroVectorStreak++;
      unsigned long now = millis();
      if (s_zeroVectorStreak >= 25 && (now - s_lastImuRecoverMs) >= 5000UL) {
        s_lastImuRecoverMs = now;
        Serial.printf(">> IMU zero-vector streak=%u; reinitializing I2C + IMU\n", s_zeroVectorStreak);
        I2C_Init();
        QMI8658_Init();
        return;
      }
    } else {
      s_zeroVectorStreak = 0;
      s_haveValidAccelSample = true;
    }
  }

}
void getGyroscope(void)
{
  uint8_t buf[6];
	esp_err_t ret = I2C_Read(Device_addr, QMI8658_GX_L, buf, 6);
	if(ret != ESP_OK)
		printf("QMI8658 : Gyroscope read failure\r\n");
	else{
    Gyro.x = (float)((int16_t)((buf[1]<<8) | (buf[0])));
    Gyro.y = (float)((int16_t)((buf[3]<<8) | (buf[2])));
    Gyro.z = (float)((int16_t)((buf[5]<<8) | (buf[4])));
    Gyro.x = Gyro.x * gyroScales;
    Gyro.y = Gyro.y * gyroScales;
    Gyro.z = Gyro.z * gyroScales;
  }
}










