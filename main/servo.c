/*
 * Servo driver - RC hobby servo (SG90 / MG90S class) that works a panel switch.
 *
 * WHAT IT IS FOR
 * --------------
 * A concealed panel switch (86-type, the mechanism inside the wall box) is a
 * mechanical rocker/toggle that needs a real push with real torque. A small
 * servo can throw it, but three things have to be right or it will either not
 * move the switch or it will brown out the board:
 *
 *   1. POWER. A 9 g servo draws 100-200 mA idle but 0.7-1.5 A when it starts
 *      moving or stalls against the switch. That must come from the 5V rail on
 *      its own branch (see SHOPPING-AND-WIRING.md), never from 3V3 and never
 *      through the ESP32's 5V pin header. This driver only produces the
 *      control signal; it cannot make the supply keep up.
 *
 *   2. TORQUE / TRAVEL. Panel switches need tens of degrees of travel and
 *      enough force to snap over-centre. CONFIG_SERVO_ANGLE_* must be set to
 *      the servo's real endpoints measured on the bench, otherwise the servo
 *      buzzes against a mechanical stop and draws stall current forever.
 *
 *   3. SOFT START. Commanding a full 0->180 jump makes the servo slam to the
 *      limit at full speed. Ramping in small steps spreads the current over
 *      time, which matters a lot on this board - see DIAGNOSIS-POWER.md.
 *
 * HOW THE SIGNAL IS MADE
 * ----------------------
 * Standard RC PWM: 50 Hz, 1.0-2.0 ms pulse, 1.5 ms = centre. The ESP32-S3
 * LEDC peripheral generates this in hardware, so the pulse stays accurate with
 * WiFi running and no CPU time is spent once a position is set.
 *
 * The servo's signal input is high-impedance and only needs logic levels, so
 * the 3.3V GPIO drives it directly - there is no driver IC to add, and no level
 * shifter needed.
 */

#include "app_internal.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_SERVO_ENABLE

static const char *TAG = "servo";

/* Resolution of the LEDC timer. 1 MHz means 1 tick = 1 us, which is what makes
 * the pulse maths below read the same as the numbers in a servo datasheet. */
#define SERVO_TIMER_RES_HZ  1000000
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0

/* Where the last commanded position is remembered across resets.
 *
 * Why this matters more than it looks: the servo physically holds the panel
 * switch in one of two states, but the firmware would otherwise always park at
 * the centre angle on boot. After a brownout reset - which this board does
 * constantly - that would mean the arm swinging away from wherever the switch
 * actually is, every single boot. Remembering the position means a reset leaves
 * the switch alone instead of flapping it.
 */
#define SERVO_NVS_NAMESPACE "servo"
#define SERVO_NVS_KEY_ANGLE "angle"

static bool s_ready;
static int s_angle;              /* last commanded position, degrees */

/* ------------------------------------------------------------------------ */
/* Angle <-> pulse width                                                     */
/* ------------------------------------------------------------------------ */

static int angle_to_us(int deg)
{
    if (deg < CONFIG_SERVO_ANGLE_MIN_DEG) {
        deg = CONFIG_SERVO_ANGLE_MIN_DEG;
    }
    if (deg > CONFIG_SERVO_ANGLE_MAX_DEG) {
        deg = CONFIG_SERVO_ANGLE_MAX_DEG;
    }

    const int span = CONFIG_SERVO_ANGLE_MAX_DEG - CONFIG_SERVO_ANGLE_MIN_DEG;
    const int us_span = CONFIG_SERVO_PULSE_MAX_US - CONFIG_SERVO_PULSE_MIN_US;

    if (span <= 0) {
        return CONFIG_SERVO_PULSE_MIN_US;
    }

    /*
     * The angle range is mapped onto the pulse range linearly. Note that the
     * endpoints are the *servo's* safe endpoints, not 0 and 180: many servos
     * cannot reach a true 180 without buzzing, and the mechanical stop is
     * found by testing, not by assuming.
     */
    return CONFIG_SERVO_PULSE_MIN_US + (us_span * (deg - CONFIG_SERVO_ANGLE_MIN_DEG)) / span;
}

static esp_err_t write_us(int us)
{
    uint32_t duty = (uint32_t)us;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

static esp_err_t write_angle(int deg, bool remember);

/* ------------------------------------------------------------------------ */
/* Position memory (NVS)                                                     */
/* ------------------------------------------------------------------------ */

/*
 * Position memory, made deliberately fail-safe.
 *
 * HISTORY - why this looks paranoid. Three separate attempts to have the
 * position survive a reset all ended in a brownout boot loop, in this order:
 *
 *   1. nvs_flash_init() + nvs_flash_erase() when the partition looked bad.
 *      A flash erase during early boot on this supply browns out, which
 *      interrupts the erase, so the next boot saw the same bad partition and
 *      tried again - a genuine infinite loop, hundreds of resets.
 *
 *   2. nvs_flash_init() alone, with a clean, all-0xFF partition. Still
 *      browned out at the same point (~372 ms, inside servo_init).
 *
 *   3. Restricting writes to intentional moves only (no write on the power-up
 *      restore). Still browned out, because the read path also initialises NVS.
 *
 * With NVS calls compiled out entirely the same firmware is rock stable
 * (verified: boot completes, `servo test mode: position 90 deg` every 5 s).
 *
 * So: nothing here may run during startup, and every failure must degrade to
 * "no position memory" and carry on. The switch still works; only the
 * remembering is lost.
 *
 * FOLLOW-UP: nvs_flash_init() being fatal at boot on this board is worth
 * understanding on its own - it is a flash operation on a supply that already
 * cannot hold 3V3 through a WiFi calibration. Fixing the supply (see
 * SHOPPING-2-POWER-FIX.md) may well make all of this work as written.
 */
static bool nvs_ready(void)
{
    static int s_state = 0;      /* 0 = untried, 1 = usable, -1 = unusable */

    if (s_state != 0) {
        return s_state > 0;
    }

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not usable (%s) - the switch position will not be "
                      "remembered across resets", esp_err_to_name(err));
        s_state = -1;
        return false;
    }

    s_state = 1;
    return true;
}

/* Angle stored from a previous run, or `fallback` when there is nothing usable.
 *
 * NEVER called during servo_init() - see the history above. It is exposed as a
 * separate function so the boot path can simply not use it, and so the caller
 * decides when a flash read is affordable. */
int servo_recall_angle(int fallback)
{
    if (!nvs_ready()) {
        return fallback;
    }

    nvs_handle_t h;
    if (nvs_open(SERVO_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return fallback;    /* nothing stored yet - the normal first-boot case */
    }

    int32_t v = fallback;
    esp_err_t err = nvs_get_i32(h, SERVO_NVS_KEY_ANGLE, &v);
    nvs_close(h);

    if (err != ESP_OK) {
        return fallback;
    }
    if (v < CONFIG_SERVO_ANGLE_MIN_DEG || v > CONFIG_SERVO_ANGLE_MAX_DEG) {
        ESP_LOGW(TAG, "stored position %d is outside %d..%d - ignoring it",
                 (int)v, CONFIG_SERVO_ANGLE_MIN_DEG, CONFIG_SERVO_ANGLE_MAX_DEG);
        return fallback;
    }
    return (int)v;
}

static void remember_angle(int deg)
{
    if (!nvs_ready()) {
        return;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(SERVO_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(h, SERVO_NVS_KEY_ANGLE, deg);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not store the position: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

/* Returns the stored angle, or fallback when there is nothing usable stored.
 *
 * The stored value is range-checked on the way out: if the Kconfig endpoints
 * were changed between builds, an old value could sit outside them, and driving
 * a servo past its mechanical stop is how gears get stripped. */

static esp_err_t write_angle(int deg, bool remember)
{
    esp_err_t err = write_us(angle_to_us(deg));
    if (err == ESP_OK) {
        s_angle = deg;
        /*
         * Only persist on an intentional move, never on the power-up restore.
         *
         * Remembering the restore would mean a flash write on EVERY boot - and
         * this board boots several times a second while it is browning out. A
         * write interrupted by a power dip corrupts the NVS header, which then
         * makes nvs_flash_init() fail for every subsequent boot. That is
         * exactly how the partition on this board got broken.
         *
         * Restoring costs nothing to skip: the value being written back is the
         * one that was just read from NVS.
         */
        if (remember) {
            remember_angle(deg);
        }
    }
    return err;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,   /* 16384 counts at 1 MHz = 16.4 ms */
        .timer_num       = SERVO_TIMER,
        .freq_hz         = CONFIG_SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch = {
        .gpio_num   = CONFIG_SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * Park at centre. Deliberately NOT restoring the remembered position here.
     *
     * Reading it would mean calling nvs_flash_init() during boot, and on this
     * board that browns out inside servo_init (see the history above). Until
     * the supply is fixed, the arm centres on every boot and the position is
     * only remembered within a run.
     *
     * The centre angle is the furthest either way from both endpoints, so this
     * can never drive the arm into a mechanical stop on power-up.
     *
     * To get the old behaviour back once the supply is sorted, replace this
     * with:
     *     const int start = servo_recall_angle(centre);
     * (servo_recall_angle() is already written and range-checks the value.)
     */
    const int centre = (CONFIG_SERVO_ANGLE_MIN_DEG + CONFIG_SERVO_ANGLE_MAX_DEG) / 2;
    const int start = centre;

    err = write_angle(start, false);   /* never write to flash on boot */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not set the initial position: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "servo on GPIO%d: %d Hz, %d..%d us = %d..%d deg, centred at %d deg",
             CONFIG_SERVO_GPIO, CONFIG_SERVO_FREQ_HZ,
             CONFIG_SERVO_PULSE_MIN_US, CONFIG_SERVO_PULSE_MAX_US,
             CONFIG_SERVO_ANGLE_MIN_DEG, CONFIG_SERVO_ANGLE_MAX_DEG, s_angle);
    return ESP_OK;
}

int servo_get_angle(void)
{
    return s_angle;
}

esp_err_t servo_move_to(int deg)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return write_angle(deg, true);     /* intentional move: persist it */
}

/*
 * Ramped move.
 *
 * Steps in CONFIG_SERVO_STEP_DEG increments, pausing CONFIG_SERVO_STEP_MS
 * between them. The servo's *speed* is set by the pulse train, so a smooth
 * ramp is produced by walking the command rather than by changing frequency.
 *
 * Why bother: a single 0->180 command makes the servo accelerate to full speed
 * instantly, which is the moment it draws its peak current. Ramping keeps that
 * peak lower and shorter, and it also stops the arm from hammering the panel
 * switch's mechanical stop.
 */
esp_err_t servo_move_smooth(int deg)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (deg < CONFIG_SERVO_ANGLE_MIN_DEG) { deg = CONFIG_SERVO_ANGLE_MIN_DEG; }
    if (deg > CONFIG_SERVO_ANGLE_MAX_DEG) { deg = CONFIG_SERVO_ANGLE_MAX_DEG; }

    const int step = (CONFIG_SERVO_STEP_DEG > 0) ? CONFIG_SERVO_STEP_DEG : 1;
    int current = s_angle;

    while (current != deg) {
        int delta = deg - current;
        if (delta > step) {
            current += step;
        } else if (delta < -step) {
            current -= step;
        } else {
            current = deg;
        }
        esp_err_t err = write_angle(current, (current == deg));
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SERVO_STEP_MS));
    }
    return ESP_OK;
}

/*
 * Throw the switch. Position A and B come from Kconfig because they depend on
 * how the servo is mechanically mounted - there is no way for the firmware to
 * know where the switch's two stable states are.
 */
esp_err_t servo_toggle(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Whichever end we are nearer to, go to the other one. Using the midpoint
     * as the divider makes a repeated toggle land the same way every time. */
    const int mid = (CONFIG_SERVO_POS_A_DEG + CONFIG_SERVO_POS_B_DEG) / 2;
    const int target = (s_angle < mid) ? CONFIG_SERVO_POS_B_DEG : CONFIG_SERVO_POS_A_DEG;

    ESP_LOGI(TAG, "toggle: %d -> %d deg", s_angle, target);
    return servo_move_smooth(target);
}

esp_err_t servo_to_a(void)
{
    return s_ready ? servo_move_smooth(CONFIG_SERVO_POS_A_DEG) : ESP_ERR_INVALID_STATE;
}

esp_err_t servo_to_b(void)
{
    return s_ready ? servo_move_smooth(CONFIG_SERVO_POS_B_DEG) : ESP_ERR_INVALID_STATE;
}

/* Release the servo: stop sending pulses so it stops holding torque.
 *
 * Worth having: a servo that is holding against a spring-loaded panel switch
 * draws current continuously, and on a supply this marginal that standing load
 * is the difference between the board surviving a WiFi burst and not.
 */
esp_err_t servo_release(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Duty 0 removes the pulse train entirely (no 20 ms frame). */
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, 0);
    if (err == ESP_OK) {
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "released (no pulses - no holding torque, no standing current)");
    }
    return err;
}

#else  /* !CONFIG_SERVO_ENABLE */

esp_err_t servo_init(void)      { return ESP_OK; }
int       servo_get_angle(void) { return 0; }
esp_err_t servo_move_to(int deg)     { (void)deg; return ESP_OK; }
esp_err_t servo_move_smooth(int deg) { (void)deg; return ESP_OK; }
esp_err_t servo_toggle(void)    { return ESP_OK; }
esp_err_t servo_to_a(void)      { return ESP_OK; }
esp_err_t servo_to_b(void)      { return ESP_OK; }
esp_err_t servo_release(void)   { return ESP_OK; }

#endif
