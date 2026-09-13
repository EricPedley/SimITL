/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"
#include "common/crc.h"

#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor_impl.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "timer_def.h"

const timerHardware_t timerHardware[1]; // unused

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "config/config_eeprom_impl.h"
#include "scheduler/scheduler.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"
#include "cli/cli.h"

#include "dyad.h"

uint32_t SystemCoreClock;
uint64_t micros_passed;
int64_t sleep_timer;

static struct timespec start_time;
static double simRate = 1.0;
static bool workerRunning = true;


int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void)
{
    return 0;
}

void timerInit(void)
{
    printf("[timer]Init...\n");
}

void timerStart(TIM_TypeDef *tim)
{
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    //while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return clockCycles;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros;
}
uint32_t getCycleCounter(void)
{
    return (uint32_t) (micros64() & 0xFFFFFFFF);
}


// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y)
{
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}


// PWM part
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    // NOOP
}

static void pwmWriteMotor(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = value - idlePulse;
    }
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    // NOOP
}

static void pwmCompleteMotorUpdate(void)
{
    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }
}

void pwmWriteServo(uint8_t index, float value)
{
    servosPwm[index] = value;
}

static const motorVTable_t vTable = {
  .postInit = motorPostInitNull,
  .convertExternalToMotor = pwmConvertFromExternal,
  .convertMotorToExternal = pwmConvertToExternal,
  .enable = pwmEnableMotors,
  .disable = pwmDisableMotors,
  .isMotorEnabled = pwmIsMotorEnabled,
  .decodeTelemetry = motorDecodeTelemetryNull,
  .write = pwmWriteMotor,
  .writeInt = pwmWriteMotorInt,
  .updateComplete = pwmCompleteMotorUpdate,
  .shutdown = pwmShutdownPulsesForAllMotors,
  .requestTelemetry = NULL,
  .isMotorIdle = NULL,
  .getMotorIO = NULL,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t _idlePulse)
{
  UNUSED(motorConfig);

  if (!device) {
      return false;
  }
  device->vTable = &vTable;
  const uint8_t motorCount = device->count;
  pwmMotorCount = device->count;
  printf("Initialized motor count %d\n", motorCount);

  idlePulse = _idlePulse;

  for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
      pwmMotors[motorIndex].enabled = true;
  }

#ifdef USE_DSHOT_TELEMETRY
  // The simulated ESCs always provide eRPM telemetry, like bidirectional
  // DSHOT hardware would. Config validation clears dev.useDshotTelemetry
  // when the configured motor protocol is not a DSHOT one, so turn it back
  // on here, before initDshotTelemetry() and rpmFilterInit() run during
  // init().
  motorConfigMutable()->dev.useDshotTelemetry = DSHOT_TELEMETRY_ON;
  useDshotTelemetry = true;
  dshotMotorCount = motorCount;
#endif

  return true;
}

#ifdef USE_DSHOT
// Fake DSHOT motor device so a dshot motor_pwm_protocol still drives the
// simulated motors: map the dshot command/throttle range (DSHOT_MIN_THROTTLE
// .. DSHOT_MAX_THROTTLE) onto the same normalized 0..1000 range that the PWM
// write path leaves in motorsPwm[].
static void pwmWriteDshotMotor(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = (value < DSHOT_MIN_THROTTLE) ? 0
            : (int16_t)((value - DSHOT_MIN_THROTTLE) * (1000.0f / DSHOT_RANGE));
    }
}

static void pwmWriteDshotMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteDshotMotor(index, (float)value);
}

static const motorVTable_t dshotVTable = {
  .postInit = motorPostInitNull,
  .convertExternalToMotor = dshotConvertFromExternal,
  .convertMotorToExternal = dshotConvertToExternal,
  .enable = pwmEnableMotors,
  .disable = pwmDisableMotors,
  .isMotorEnabled = pwmIsMotorEnabled,
  .decodeTelemetry = motorDecodeTelemetryNull,
  .write = pwmWriteDshotMotor,
  .writeInt = pwmWriteDshotMotorInt,
  .updateComplete = pwmCompleteMotorUpdate,
  .shutdown = pwmShutdownPulsesForAllMotors,
  .requestTelemetry = NULL,
  .isMotorIdle = NULL,
  .getMotorIO = NULL,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig)
{
    UNUSED(motorDevConfig);

    if (!device) {
        return false;
    }
    device->vTable = &dshotVTable;
    pwmMotorCount = device->count;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {
        pwmMotors[motorIndex].enabled = true;
    }

#ifdef USE_DSHOT_TELEMETRY
    // The simulated ESCs always provide eRPM telemetry, like bidirectional
    // DSHOT hardware would; enable it even if dshot_bidir is off in the
    // loaded config.
    motorConfigMutable()->dev.useDshotTelemetry = DSHOT_TELEMETRY_ON;
    useDshotTelemetry = true;
    dshotMotorCount = pwmMotorCount;
#endif

    return true;
}
#endif // USE_DSHOT

// Stub out the DSHOT command machinery (beacons, spin direction, ESC info,
// extended telemetry enable). There is no real ESC to send commands to, but
// dshotStreamingCommandsAreEnabled() must report true: core.c refuses to
// clear ARMING_DISABLED_BOOT_GRACE_TIME for DSHOT protocols otherwise (the
// real implementation waits for motorEnableTimeMs, which is stuck at 0 in
// the sim since motorEnable() runs at millis()==0).
void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType)
{
    UNUSED(index);
    UNUSED(motorCount);
    UNUSED(command);
    UNUSED(commandType);
}

void dshotSetPidLoopTime(uint32_t pidLoopTime)
{
    UNUSED(pidLoopTime);
}

bool dshotCommandQueueEmpty(void)
{
    return true;
}

bool dshotCommandIsProcessing(void)
{
    return false;
}

uint8_t dshotCommandGetCurrent(unsigned index)
{
    UNUSED(index);
    return 0;
}

bool dshotCommandOutputIsEnabled(unsigned motorCount)
{
    UNUSED(motorCount);
    return true;
}

bool dshotStreamingCommandsAreEnabled(void)
{
    return true;
}

#ifdef USE_DSHOT_TELEMETRY
// Simulated ESC telemetry: mirrors what pwmTelemetryDecode() does on real
// hardware after receiving the ESC's telemetry frame. The frame carries the
// commutation period (us per electrical revolution) encoded as
// (period & 0x1ff) << (period >> 9); see dshot_decode_eRPM_telemetry_value().
// The raw frame is decoded by updateDshotTelemetry() in the filter task.
dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;

static uint16_t sitlDshotEncodeErpm(float erpm)
{
    if (!(erpm > 0.0f)) {
        return 0x0fff; // decodes to zero eRPM
    }
    uint32_t period = (uint32_t)lrintf(60000000.0f / erpm);
    if (period < 1) {
        period = 1;
    }
    if (period > 0xffff) {
        // dshot_decode_eRPM_telemetry_value() reconstructs the period into a
        // uint16_t, so frames with period > 65535us (eRPM < ~915) decode as
        // invalid and would leave a stale value. Report the stopped sentinel
        // like a real ESC does.
        return 0x0fff;
    }
    uint32_t shift = 0;
    while (period > 0x1ff && shift < 0x7f) {
        period >>= 1;
        shift++;
    }
    return (uint16_t)((shift << 9) | (period & 0x1ff));
}

void sitlDshotTelemetryFeedErpm(uint8_t motorIndex, float erpm)
{
    if (motorIndex < MAX_SUPPORTED_MOTORS) {
        dshotTelemetryState.motorState[motorIndex].rawValue = sitlDshotEncodeErpm(erpm);
    }
}

void sitlDshotTelemetryFeedDone(void)
{
    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    dshotTelemetryState.readCount++;
}
#endif // USE_DSHOT_TELEMETRY

// ADC part
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;

char * EEPROM_FILENAME = 0;

// Mirrors the private configRecord_t layout used by
// betaflight/src/main/config/config_eeprom.c: the image starts with a 2-byte
// header {eepromConfigVersion, 0xBE magic}, followed by these records, a
// zero-size terminator and a stored CRC word chosen so that the CRC over the
// whole image (including itself) is the fixed residue value.
typedef struct {
    uint16_t size;      // includes this 6-byte header
    uint16_t pgn;
    uint8_t  version;
    uint8_t  flags;
    uint8_t  pg[];
} PG_PACKED sitlConfigRecord_t;

// loadEEPROM() reports failure when a parameter group compiled into this
// build has no record in the image, which makes init() reset the entire
// configuration to defaults. Whenever the compiled PG set grows (e.g.
// USE_DSHOT adds PG_ESC_SENSOR_CONFIG and USE_RPM_FILTER adds
// PG_RPM_FILTER_CONFIG), any eeprom file written by an older build would
// silently lose every setting - including mixerConfig.yaw_motors_reversed
// and the motor protocol, which changes closed-loop behavior. Appending a
// record carrying each missing group's reset defaults lets the normal load
// path keep the stored configuration, matching what a real FC does when a
// new parameter group is introduced.
static void sitlEepromAppendMissingPgRecords(void)
{
    if (eepromData[1] != 0xBE) {
        return; // empty or foreign image; the normal load path handles it
    }

    // locate the zero-size terminator record
    uint8_t *terminator = NULL;
    uint8_t *p = eepromData + 2;
    while (p + sizeof(sitlConfigRecord_t) <= ARRAYEND(eepromData)) {
        const sitlConfigRecord_t *rec = (const sitlConfigRecord_t *)p;
        if (rec->size == 0) {
            terminator = p;
            break;
        }
        if (rec->size < sizeof(sitlConfigRecord_t)
            || p + rec->size > ARRAYEND(eepromData)) {
            return; // malformed image; leave it for the normal validity checks
        }
        p += rec->size;
    }
    if (!terminator) {
        return;
    }

    uint8_t *appendAt = terminator;
    int appended = 0;
    PG_FOREACH(reg) {
        const uint16_t pgn = pgN(reg);
        bool present = false;
        for (const uint8_t *q = eepromData + 2; q < terminator; ) {
            const sitlConfigRecord_t *rec = (const sitlConfigRecord_t *)q;
            if (rec->pgn == pgn && !(rec->flags & 0x3)) {
                present = true;
                break;
            }
            q += rec->size;
        }
        if (present) {
            continue;
        }
        const uint16_t recordSize = sizeof(sitlConfigRecord_t) + pgSize(reg);
        if (appendAt + recordSize + 4 > ARRAYEND(eepromData)) {
            break; // no room; let the normal load path report the failure
        }
        pgReset(reg); // populate reg->address with the group's reset defaults
        sitlConfigRecord_t *rec = (sitlConfigRecord_t *)appendAt;
        rec->size = recordSize;
        rec->pgn = pgn;
        rec->version = pgVersion(reg);
        rec->flags = 0;
        memcpy(rec->pg, reg->address, pgSize(reg));
        fprintf(stderr, "[FLASH_Unlock] appended record for pgn %d (version %d, size %d)\n",
                (int)pgn, (int)pgVersion(reg), (int)pgSize(reg));
        appendAt += recordSize;
        appended++;
    }
    if (!appended) {
        return;
    }

    // terminator + stored CRC (inverted, big-endian CRC16 over everything
    // before it), matching writeSettingsToEEPROM() in config_eeprom.c.
    appendAt[0] = 0;
    appendAt[1] = 0;
    uint16_t crc = 0xFFFF;
    crc = crc16_ccitt_update(crc, eepromData, (appendAt + 2) - eepromData);
    const uint16_t storedCrc = ~(((crc & 0xFF) << 8) | (crc >> 8));
    memcpy(appendAt + 2, &storedCrc, sizeof(storedCrc));
    printf("[FLASH_Unlock] appended %d missing PG record(s) to '%s'\n",
           appended, EEPROM_FILENAME);
}

bool loadEEPROMFromFile(void)
{
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");

        // simitl can just restart without closing the fileDesc...
        fclose(eepromFd);
        //return false;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME, "rb+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n",
                   EEPROM_FILENAME,
                   lSize,
                   sizeof(eepromData));
            sitlEepromAppendMissingPgRecords();
        } else {
            fprintf(stderr,
                    "[FLASH_Unlock] failed to load '%s, %ld < %ld'\n",
                    EEPROM_FILENAME,
                    n,
                    lSize);
            return false;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "wb+")) == NULL) {
            fprintf(stderr,  "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return false;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
              return false;
        }
    }
    return true;
}

void configUnlock(void)
{
    loadEEPROMFromFile();
}

void configLock(void)
{
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        int wrote = fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s size: %d'\n", EEPROM_FILENAME, wrote);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t), "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    if ((address >= (uintptr_t)eepromData) && (address + sizeof(uint32_t) <= (uintptr_t)ARRAYEND(eepromData))) {
        memcpy((void*)address, buffer, sizeof(config_streamer_buffer_type_t));
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)address, *((uint32_t*)address));
    } else {
        printf("[FLASH_ProgramWord]%p out of range!\n", (void*)address);
    }
    return CONFIG_RESULT_SUCCESS;
}


void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    printf("IOConfigGPIO\n");
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    return (IO_GPIO(io)->IDR & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BSRR = IO_Pin(io) << (hi ? 0 : 16);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BSRR = IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BRR = IO_Pin(io);
}

void IOInitGlobal(void)
{
    // NOOP
}

IO_t IOGetByTag(ioTag_t tag)
{
    UNUSED(tag);
    return NULL;
}

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = { .id = MCU_TYPE_SIMULATOR, .name = "SIMULATOR" };
    return &info;
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);
    if (IO_GPIO(io)->ODR & mask) {
        mask <<= 16; // bit is set, shift mask to reset half
    }
    IO_GPIO(io)->BSRR = mask;
}


void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind\n");
}

void debugInit(void)
{
    printf("debugInit\n");
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}


void systemInit(void) {
  printf("[system] Init...\n");
  SystemCoreClock = 500 * 1000000;  // fake 500MHz
  micros_passed = 0U;
  sleep_timer = 0;
  cliMode = false;
}

void systemReset(void) {
  printf("[system] Reset!\n");

  //reinits BF
  init();
  // exit(0);
}

void systemResetToBootloader(bootloaderRequestType_e requestType){
  printf("[system] ResetToBootloader!\n");
  // exit(1);
}

uint64_t micros64(void)
{
  return micros_passed;
}

uint32_t micros(void) {
  return micros_passed & 0xFFFFFFFF;
}

uint32_t millis(void) {
  return (micros_passed / 1000) & 0xFFFFFFFF;
}

void microsleep(uint32_t usec) {
  sleep_timer = usec;
}

void delayMicroseconds(uint32_t usec) {
  microsleep(usec);
}

void delayMicroseconds_real(uint32_t us)
{
    microsleep(us);
}

void delay(uint32_t ms) {
  microsleep(ms * 1000);
}

#ifdef _WIN32
#include <string.h>

char *strsep(char **stringp, const char *delim) {
    char *rv = *stringp;
    if (rv) {
        *stringp += strcspn(*stringp, delim);
        if (**stringp)
            *(*stringp)++ = '\0';
        else
            *stringp = 0; }
    return rv;
}
#endif

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);
}

bool useDshotTelemetry = false;