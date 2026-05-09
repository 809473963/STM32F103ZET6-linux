#include "gripper42_tb6600.h"

#include <string.h>

#include "delay.h"
#include "stm32f10x_flash.h"

#define GRIPPER42_PUL_GPIO GPIOA
#define GRIPPER42_PUL_PIN GPIO_Pin_1
#define GRIPPER42_DIR_GPIO GPIOA
#define GRIPPER42_DIR_PIN GPIO_Pin_2
#define GRIPPER42_EN_GPIO GPIOA
#define GRIPPER42_EN_PIN GPIO_Pin_3

#define GRIPPER42_EN_ACTIVE_LOW 1

#define GRIPPER42_FLASH_ADDR 0x0807F800UL
#define GRIPPER42_CFG_MAGIC 0x47525034UL
#define GRIPPER42_CFG_VERSION 1U

#define GRIPPER42_DEFAULT_MIN_STEPS (-2000)
#define GRIPPER42_DEFAULT_MAX_STEPS (2000)
#define GRIPPER42_DEFAULT_MARGIN_STEPS 100U
#define GRIPPER42_DEFAULT_SPEED_STEPS_S 500U
#define GRIPPER42_MIN_SPEED_STEPS_S 80U
#define GRIPPER42_MAX_SPEED_STEPS_S 1500U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t zero_valid;
    int32_t min_steps;
    int32_t max_steps;
    uint16_t margin_steps;
    uint16_t speed_steps_s;
    uint32_t checksum;
} Gripper42FlashCfg_t;

static struct {
    Gripper42State_t state;
    uint8_t zero_valid;
    int32_t pos_steps;
    int32_t target_steps;
    int32_t min_steps;
    int32_t max_steps;
    uint16_t margin_steps;
    uint16_t speed_steps_s;
    uint16_t step_period_ms;
    uint16_t step_elapsed_ms;
    uint8_t moving_dir;
    uint8_t enabled;
} g_gripper;

static uint32_t cfg_checksum(const Gripper42FlashCfg_t *cfg)
{
    uint32_t sum = 0U;
    const uint32_t *p = (const uint32_t *)cfg;
    uint32_t words = (uint32_t)(sizeof(Gripper42FlashCfg_t) / 4U);
    uint32_t i;
    for (i = 0U; i < (words - 1U); i++) {
        sum ^= p[i];
        sum = (sum << 5) | (sum >> 27);
    }
    return sum ^ 0xA5A55A5AU;
}

static void en_set(uint8_t on)
{
#if GRIPPER42_EN_ACTIVE_LOW
    GPIO_WriteBit(GRIPPER42_EN_GPIO, GRIPPER42_EN_PIN, on ? Bit_RESET : Bit_SET);
#else
    GPIO_WriteBit(GRIPPER42_EN_GPIO, GRIPPER42_EN_PIN, on ? Bit_SET : Bit_RESET);
#endif
    g_gripper.enabled = on ? 1U : 0U;
}

static void dir_set(uint8_t dir_open)
{
    GPIO_WriteBit(GRIPPER42_DIR_GPIO, GRIPPER42_DIR_PIN, dir_open ? Bit_SET : Bit_RESET);
}

static void pulse_step_once(void)
{
    GPIO_SetBits(GRIPPER42_PUL_GPIO, GRIPPER42_PUL_PIN);
    delay_us(8);
    GPIO_ResetBits(GRIPPER42_PUL_GPIO, GRIPPER42_PUL_PIN);
    delay_us(8);
}

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void apply_speed(uint16_t speed_steps_s)
{
    g_gripper.speed_steps_s = clamp_u16(speed_steps_s, GRIPPER42_MIN_SPEED_STEPS_S, GRIPPER42_MAX_SPEED_STEPS_S);
    g_gripper.step_period_ms = (uint16_t)(1000U / g_gripper.speed_steps_s);
    if (g_gripper.step_period_ms == 0U) {
        g_gripper.step_period_ms = 1U;
    }
}

static void load_defaults(void)
{
    g_gripper.state = GRIPPER42_STATE_UNHOMED;
    g_gripper.zero_valid = 0U;
    g_gripper.pos_steps = 0;
    g_gripper.target_steps = 0;
    g_gripper.min_steps = GRIPPER42_DEFAULT_MIN_STEPS;
    g_gripper.max_steps = GRIPPER42_DEFAULT_MAX_STEPS;
    g_gripper.margin_steps = GRIPPER42_DEFAULT_MARGIN_STEPS;
    apply_speed(GRIPPER42_DEFAULT_SPEED_STEPS_S);
    g_gripper.step_elapsed_ms = 0U;
    g_gripper.moving_dir = 0U;
}

static uint8_t load_cfg_from_flash(void)
{
    const Gripper42FlashCfg_t *cfg = (const Gripper42FlashCfg_t *)GRIPPER42_FLASH_ADDR;
    if (cfg->magic != GRIPPER42_CFG_MAGIC || cfg->version != GRIPPER42_CFG_VERSION) {
        return 0U;
    }
    if (cfg->checksum != cfg_checksum(cfg)) {
        return 0U;
    }
    g_gripper.zero_valid = (cfg->zero_valid != 0U) ? 1U : 0U;
    g_gripper.min_steps = cfg->min_steps;
    g_gripper.max_steps = cfg->max_steps;
    g_gripper.margin_steps = cfg->margin_steps;
    apply_speed(cfg->speed_steps_s);
    return 1U;
}

static uint8_t save_cfg_to_flash(void)
{
    FLASH_Status st;
    uint32_t addr = GRIPPER42_FLASH_ADDR;
    uint32_t i;
    Gripper42FlashCfg_t cfg;
    const uint32_t *src = (const uint32_t *)&cfg;
    uint32_t words = (uint32_t)(sizeof(Gripper42FlashCfg_t) / 4U);

    cfg.magic = GRIPPER42_CFG_MAGIC;
    cfg.version = GRIPPER42_CFG_VERSION;
    cfg.zero_valid = g_gripper.zero_valid ? 1U : 0U;
    cfg.min_steps = g_gripper.min_steps;
    cfg.max_steps = g_gripper.max_steps;
    cfg.margin_steps = g_gripper.margin_steps;
    cfg.speed_steps_s = g_gripper.speed_steps_s;
    cfg.checksum = cfg_checksum(&cfg);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    st = FLASH_ErasePage(GRIPPER42_FLASH_ADDR);
    if (st != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0U;
    }

    for (i = 0U; i < words; i++) {
        st = FLASH_ProgramWord(addr, src[i]);
        if (st != FLASH_COMPLETE) {
            FLASH_Lock();
            return 0U;
        }
        addr += 4U;
    }
    FLASH_Lock();
    return 1U;
}

void Gripper42_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GRIPPER42_PUL_PIN | GRIPPER42_DIR_PIN | GRIPPER42_EN_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_ResetBits(GRIPPER42_PUL_GPIO, GRIPPER42_PUL_PIN);
    GPIO_ResetBits(GRIPPER42_DIR_GPIO, GRIPPER42_DIR_PIN);

    load_defaults();
    (void)load_cfg_from_flash();
    g_gripper.state = GRIPPER42_STATE_UNHOMED;
    en_set(0U);
}

void Gripper42_TickMs(uint16_t elapsed_ms)
{
    int32_t safe_min;
    int32_t safe_max;

    if (g_gripper.state != GRIPPER42_STATE_READY) {
        return;
    }
    if (g_gripper.pos_steps == g_gripper.target_steps) {
        en_set(0U);
        return;
    }

    g_gripper.step_elapsed_ms = (uint16_t)(g_gripper.step_elapsed_ms + elapsed_ms);
    if (g_gripper.step_elapsed_ms < g_gripper.step_period_ms) {
        return;
    }
    g_gripper.step_elapsed_ms = 0U;

    safe_min = g_gripper.min_steps + (int32_t)g_gripper.margin_steps;
    safe_max = g_gripper.max_steps - (int32_t)g_gripper.margin_steps;
    g_gripper.target_steps = clamp_i32(g_gripper.target_steps, safe_min, safe_max);

    g_gripper.moving_dir = (g_gripper.target_steps > g_gripper.pos_steps) ? 1U : 0U;
    dir_set(g_gripper.moving_dir);
    en_set(1U);
    pulse_step_once();
    g_gripper.pos_steps += (g_gripper.moving_dir ? 1 : -1);
}

void Gripper42_Jog(uint8_t dir, uint8_t speed_pct)
{
    uint16_t speed = GRIPPER42_MIN_SPEED_STEPS_S;
    int32_t safe_min = g_gripper.min_steps + (int32_t)g_gripper.margin_steps;
    int32_t safe_max = g_gripper.max_steps - (int32_t)g_gripper.margin_steps;

    if (g_gripper.state != GRIPPER42_STATE_READY) {
        return;
    }
    speed = (uint16_t)(GRIPPER42_MIN_SPEED_STEPS_S +
                       (((uint32_t)clamp_u16(speed_pct, 1U, 100U)) *
                        (GRIPPER42_MAX_SPEED_STEPS_S - GRIPPER42_MIN_SPEED_STEPS_S)) / 100U);
    apply_speed(speed);

    if (dir == 2U) {
        g_gripper.target_steps = g_gripper.pos_steps;
        en_set(0U);
        return;
    }

    if (dir == 1U) {
        g_gripper.target_steps = safe_max;
    } else {
        g_gripper.target_steps = safe_min;
    }
}

void Gripper42_Stop(void)
{
    g_gripper.target_steps = g_gripper.pos_steps;
    en_set(0U);
}

void Gripper42_SetOpenPercent(uint8_t open_pct)
{
    int32_t safe_min;
    int32_t safe_max;
    int32_t span;

    if (g_gripper.state != GRIPPER42_STATE_READY) {
        return;
    }

    safe_min = g_gripper.min_steps + (int32_t)g_gripper.margin_steps;
    safe_max = g_gripper.max_steps - (int32_t)g_gripper.margin_steps;
    if (safe_max <= safe_min) {
        g_gripper.state = GRIPPER42_STATE_FAULT;
        return;
    }

    if (open_pct > 100U) open_pct = 100U;
    span = safe_max - safe_min;
    g_gripper.target_steps = safe_min + (int32_t)(((uint32_t)span * open_pct) / 100U);
}

uint8_t Gripper42_SetZero(void)
{
    g_gripper.pos_steps = 0;
    g_gripper.target_steps = 0;
    g_gripper.zero_valid = 1U;
    g_gripper.state = GRIPPER42_STATE_READY;
    en_set(0U);
    return 1U;
}

uint8_t Gripper42_SaveConfig(void)
{
    return save_cfg_to_flash();
}

uint8_t Gripper42_TrustLastZero(void)
{
    if (!g_gripper.zero_valid) {
        return 0U;
    }
    g_gripper.pos_steps = 0;
    g_gripper.target_steps = 0;
    g_gripper.state = GRIPPER42_STATE_READY;
    return 1U;
}

void Gripper42_GetStatus(Gripper42Status_t *out)
{
    if (out == 0) {
        return;
    }
    out->state = (uint8_t)g_gripper.state;
    out->zero_valid = g_gripper.zero_valid;
    out->pos_steps = g_gripper.pos_steps;
    out->target_steps = g_gripper.target_steps;
    out->min_steps = g_gripper.min_steps;
    out->max_steps = g_gripper.max_steps;
    out->margin_steps = g_gripper.margin_steps;
    out->run_speed_steps_s = g_gripper.speed_steps_s;
}
