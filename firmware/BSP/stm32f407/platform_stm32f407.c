/**
 * @file    platform_stm32f407.c
 * @brief   STM32F407VET6 implementation of the platform seam declared in
 *          `platform_api.h`.
 *
 * @details
 * This is the only file in the project that touches a register.  It implements
 * every non-static function of `platform_api.h` and mirrors the behaviour of the
 * reference implementation `firmware/BSP/host/platform_host.c` exactly: same
 * status codes for the same inputs, same refusal to write an out-parameter on
 * failure, same latching semantics for the emergency kill, same reporting-once
 * behaviour for the watchdog reset cause.
 *
 * ---------------------------------------------------------------------------
 * # Pin and peripheral map
 * ---------------------------------------------------------------------------
 *
 * | Function                  | Pin(s)        | Peripheral / mode                   |
 * |---------------------------|---------------|-------------------------------------|
 * | Joint PWM                 | PA8           | TIM1_CH1, AF1, edge-aligned 20 kHz  |
 * | Gripper PWM               | PE11          | TIM1_CH2, AF1, edge-aligned 20 kHz  |
 * | Joint DIR                 | PE0           | GPIO output, push-pull              |
 * | Joint driver enable       | PE1           | GPIO output, push-pull              |
 * | Gripper DIR               | PE2           | GPIO output, push-pull              |
 * | Gripper driver enable     | PE3           | GPIO output, push-pull              |
 * | Joint encoder A/B         | PA0 / PA1     | TIM2_CH1/CH2, AF1, X4 quadrature    |
 * | Gripper encoder A/B       | PB6 / PB7     | TIM4_CH1/CH2, AF2, X4 quadrature    |
 * | Force (load cell)         | PA4           | ADC1_IN4                            |
 * | Joint current             | PA5           | ADC1_IN5                            |
 * | Gripper current           | PA2           | ADC1_IN2                            |
 * | Supply rail (24 V divider)| PA3           | ADC1_IN3                            |
 * | ADC trigger               | -             | TIM3 TRGO at 500 Hz, DMA2 Stream0   |
 * | Millisecond time base     | -             | TIM5 free-running, 1 kHz, 32-bit    |
 * | RS485 DE release timer    | -             | TIM7 one-pulse, 1 us resolution     |
 * | Debug console             | PA9 / PA10    | USART1, AF7, 115200 8N1             |
 * | Vision / host link        | PD5 / PD6     | USART2, AF7, 921600 8N1             |
 * | RS485 (Modbus RTU)        | PB10 / PB11   | USART3, AF7, 115200 8N1             |
 * | RS485 driver enable (DE)  | PD12          | GPIO output, push-pull              |
 * | CAN1                      | PD0 / PD1     | CAN1, AF9, 500 kbit/s               |
 * | E-stop loop (NC)          | PE4           | GPIO input, pull-down               |
 * | Joint limit min           | PE5           | GPIO input, pull-up                 |
 * | Joint limit max           | PE6           | GPIO input, pull-up                 |
 * | Gripper limit open        | PE7           | GPIO input, pull-up                 |
 * | Gripper limit closed      | PE8           | GPIO input, pull-up                 |
 * | Gate-driver nFAULT        | PE9           | GPIO input, pull-up, active low     |
 *
 * Everything that could be moved onto one GPIO port was: the six discrete inputs
 * are PE4..PE9, which makes `tp_platform_io_read_all()` a single `GPIOE->IDR`
 * read and one shift, and the four direction/enable outputs are PE0..PE3 on the
 * same port for the same reason.
 *
 * ## Conflict 1, resolved: PA0/PA1 are both TIM2_CH1/CH2 and ADC1_IN0/IN1
 *
 * `bsp_types.h` documents the force channel as `PA0, ADC1_IN0` and the joint
 * current as `PA1, ADC1_IN1`.  Those two pins are *also* the joint encoder's
 * quadrature inputs.  A pin has exactly one alternate function, so one of the two
 * uses has to move, and the encoder cannot: TIM2 in encoder mode needs those two
 * specific channels, whereas any ADC input can measure anything.
 *
 * The resolution is therefore to move the **analogue** channels, keeping the
 * signal that depends on a specific peripheral where it is:
 *
 * | Channel                  | `bsp_types.h` (stale) | This port    | Why            |
 * |--------------------------|-----------------------|--------------|----------------|
 * | `TP_ADC_FORCE`           | PA0, ADC1_IN0         | PA4, IN4     | free, same ADC |
 * | `TP_ADC_CURRENT_JOINT`   | PA1, ADC1_IN1         | PA5, IN5     | PA1 is TIM2_CH2|
 * | `TP_ADC_CURRENT_GRIPPER` | PA2, ADC1_IN2         | PA2, IN2     | unchanged      |
 * | `TP_ADC_VBUS`            | PA3, ADC1_IN3         | PA3, IN3     | unchanged      |
 *
 * PA2 and PA3 stay where `bsp_types.h` says because USART2 was *also* moved (to
 * PD5/PD6, as this port's pin map requires), so PA2/PA3 are free.  Only the two
 * pins that genuinely collide were moved.  The DMA rank order is unchanged and
 * still matches the `tp_adc_ch_t` enum order, so nothing above the seam sees
 * this; `s_adc_channel[TP_ADC_COUNT]` is the single translation table.
 *
 * ## Conflict 2, resolved: PA9 is both TIM1_CH2 and USART1_TX
 *
 * The port brief asks for `PA9 = TIM1_CH2 = gripper PWM` *and*
 * `USART1 (PA9/PA10) = debug`.  PA9 cannot be both.  The debug console cannot
 * move without giving up `USART1`, which `bsp_types.h` names explicitly for
 * `TP_BUS_DEBUG_UART`, so the **PWM** moved:
 *
 *   - Joint PWM stays on **PA8 = TIM1_CH1**, as specified.
 *   - Gripper PWM moves to **PE11 = TIM1_CH2**, the alternate pin for the very
 *     same channel and the very same AF1.
 *
 * The timer-channel contract in `bsp_types.h` ("TIM1_CH1 PWM" / "TIM1_CH2 PWM")
 * is preserved exactly; only a pin changed, and it changed to a pin that is not
 * otherwise used.  PE11 also sits in the same port as the DIR/EN outputs, so the
 * change costs no additional GPIO port clock.
 *
 * ## Pins deliberately left unused
 *
 * PE9 is TIM1_CH1's alternate pin, but TIM1_CH1 is routed to PA8, so PE9 is free
 * for use as the gate-driver nFAULT input.  PA0 is also the WKUP pin; the pull-up
 * fitted for the encoder means the robot cannot be woken from Standby by a
 * floating encoder line, which is why Standby is not used at all in this design.
 *
 * ---------------------------------------------------------------------------
 * # Why the PWM is edge-aligned and not centre-aligned
 * ---------------------------------------------------------------------------
 *
 * TIM1 is clocked from APB2's timer kernel at 168 MHz.  At a 20 kHz switching
 * frequency an edge-aligned period is 168e6/20000 = **8400 counts**, i.e. 12.6
 * bits of duty resolution and 0.119 per-mille per count.  A centre-aligned period
 * spends half its counts counting up and half counting down, so the same 20 kHz
 * would halve the resolution to 4200 counts (12.0 bits, 0.238 per-mille per
 * count).  Centre alignment is the right answer when it buys lower harmonic
 * content or when it is needed for complementary outputs with dead time; this
 * board drives a single-ended `PWM + DIR` gate driver with no complementary
 * outputs, no dead-time generator and no ADC trigger taken from TIM1 (the ADC
 * trigger comes from TIM3), so centre alignment would cost resolution and buy
 * nothing.  Edge alignment also fixes the duty-update instant at one point per
 * period, which makes the current-loop sampling delay deterministic.
 *
 * ---------------------------------------------------------------------------
 * # Register-level peripherals, and why
 * ---------------------------------------------------------------------------
 *
 * `stm32f4xx_hal_can.c` and `stm32f4xx_hal_iwdg.c` do not exist on the machine
 * this port was written on (see `firmware/Drivers/third_party/README.md`), so
 * **CAN1 and the IWDG are driven by direct register access**.  Their arithmetic
 * is shown in full at the point of use rather than hidden behind a library call.
 *
 * The three USARTs are *also* register-level, and that is a design choice rather
 * than a limitation: the HAL's UART state machine assumes one owner per port and
 * would have to be bypassed for the DMA ring anyway, `HAL_UART_Transmit_DMA`
 * would have to be told about every ring wrap, and the `__HAL_LOCK` guards add
 * branches to the ISR path.  Direct registers keep the whole receive path
 * auditable in this one file.  The HAL is still used for RCC, GPIO, TIM (PWM,
 * encoder, base) and ADC, where it does not get in the way.
 *
 * ---------------------------------------------------------------------------
 * # The millisecond time base
 * ---------------------------------------------------------------------------
 *
 * `platform_api.h` says the target time base is "the FreeRTOS tick".  This port
 * uses a free-running TIM5 instead, because:
 *
 *   1. The documented wrap period is 2^32 ms (~49.7 days).  A 32-bit timer with
 *      `ARR = 0xFFFFFFFF` gives exactly that; a 32-bit tick count computes it.
 *   2. The same code is correct in both the bare-metal and the FreeRTOS build,
 *      with no `#if` in the platform and no behavioural difference between them.
 *   3. An RTOS tick *stops* while interrupts are masked, so a tick-derived clock
 *      silently loses time inside any critical section.  A hardware counter does
 *      not, which matters because the control loop's timestamps feed the
 *      derivative term and the stall detector.
 *
 * The FreeRTOS build still uses this timer; nothing in the platform calls an
 * RTOS API, which is also what keeps every ISR here legal at any priority.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "platform_stm32f407.h"

#if defined(TP_PLATFORM_STM32F407)

#include <string.h>

#include "stm32f4xx_hal.h"

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "ring_buffer.h"

/* =========================================================================
 * Internal constants
 * ========================================================================= */

/** @brief Software ring capacity per bus direction, bytes (power of two). */
#define TP_STM32_BUS_RING_BYTES 512u

/** @brief Circular RX DMA scratch size per UART, bytes (power of two). */
#define TP_STM32_RX_DMA_BYTES 256u

/** @brief Software CAN receive FIFO depth, in frames. */
#define TP_STM32_CAN_RX_FRAMES 16u

/** @brief Power-of-two factor of TIM2/TIM4 in encoder mode: X4 decoding. */
#define TP_STM32_ENCODER_X4 4

/** @brief PWM period in TIM1 counts: 168 MHz / 20 kHz. */
#define TP_STM32_PWM_ARR 8400u

/** @brief PWM switching frequency, Hz, for the boot log. */
#define TP_STM32_PWM_HZ 20000u

/** @brief Nominal LSI frequency, Hz.  See the IWDG arithmetic below. */
#define TP_STM32_LSI_HZ 32000u

/* -------------------------------------------------------------------------
 * CAN1 bit timing, APB1 = 42 MHz, target = 500 kbit/s (TP_CAN_BITRATE_BPS)
 *
 *   tq            = (BRP + 1) / 42 MHz
 *   bit time      = tq * (1 + BS1 + BS2)      where BS1 = TS1 + 1, BS2 = TS2 + 1
 *   sample point  = (1 + BS1) / (1 + BS1 + BS2)
 *   BUS (bxCAN) limits: BS1 in [1,16] -> TS1 in [0,15]
 *                       BS2 in [1,8]  -> TS2 in [0,7]
 *                       BRP in [0,1023]
 *
 * A bit time of exactly 1/500000 = 2 us needs
 *
 *       (BRP + 1) * (1 + BS1 + BS2) = 2 us * 42 MHz = 84 clock cycles.
 *
 * So the divisor product must be 84, with (1 + BS1 + BS2) <= 25 because of the
 * BS1/BS2 fields.  Factor pairs of 84 and the best sample point each admits:
 *
 *   n = BRP+1   N = 1+BS1+BS2   best (BS1, BS2)   sample point
 *   ---------   -------------   ---------------   ------------
 *      4            21           (16, 4)           17/21 = 80.95 %
 *      6            14           (11, 2)           12/14 = 85.71 %   <-- chosen
 *      7            12           (9,  2)           10/12 = 83.33 %
 *     12             7           (4,  2)            5/7  = 71.43 %
 *     14             6           (3,  2)            4/6  = 66.67 %
 *     21             4           (2,  1)            3/4  = 75.00 %
 *     28             3           (1,  1)            2/3  = 66.67 %
 *     84             1           impossible (N must be >= 3)
 *
 * Nothing in the list reaches the 87.5 % ideal, and nothing can: an exact 87.5 %
 * requires N to be divisible by 8, and 84 is not.  The closest achievable is
 * N = 14 (which is 84/6), giving BS1 = 11, BS2 = 2 and a sample point of
 *
 *       12 / 14 = 85.714 %, i.e. 1.786 % of a bit time early = 35.7 ns at 500
 *       kbit/s.
 *
 * so the chosen solution is the *only* one that is both exactly 500 000 bit/s
 * and within 2 % of the ideal sample point:
 *
 *       BRP = 5  (prescaler 6)   TS1 = 10 (BS1 = 11)   TS2 = 1 (BS2 = 2)
 *       SJW = 0  (SJW = 1 tq)
 *       BTR = 0x5 | (10 << 16) | (1 << 20) = 0x001A0005
 *       bit rate = 42 MHz / (6 * 14) = 500 000 bit/s exactly, 0 ppm error
 *
 * (The brief's suggested "prescaler 5, BS1 15, BS2 2" would give
 * 42e6 / (5 * 18) = 466 667 bit/s — a 6.7 % error, far outside the 0.5 %
 * that CAN bit-timing tolerance allows — which is why it is not used.)
 * ------------------------------------------------------------------------- */
#define TP_STM32_CAN_BRP 5u  /**< BRP field: prescaler = BRP + 1 = 6. */
#define TP_STM32_CAN_TS1 10u /**< TS1 field: BS1 = TS1 + 1 = 11 tq.     */
#define TP_STM32_CAN_TS2 1u  /**< TS2 field: BS2 = TS2 + 1 = 2 tq.      */
#define TP_STM32_CAN_SJW 0u  /**< SJW field: SJW = 1 tq.                */

/** @brief Index of the discrete-input port bit that carries `TP_IO_ESTOP`. */
#define TP_STM32_IO_SHIFT 4u

/** @brief Mask of the six contiguous discrete inputs (PE4..PE9). */
#define TP_STM32_IO_MASK 0x3Fu

/** @brief UART error counters are split per bus; index with @ref tp_bus_t. */
#define TP_STM32_BUS_NONE 0xFFu

/* =========================================================================
 * Types
 * ========================================================================= */

/** @brief One UART plus the ring buffers and DMA stream it owns. */
typedef struct {
    USART_TypeDef      *regs;       /**< USART1/2/3.                       */
    DMA_Stream_TypeDef *tx_dma;     /**< Memory -> USART_DR stream.       */
    DMA_Stream_TypeDef *rx_dma;     /**< USART_DR -> memory stream.       */
    uint32_t            pclk_hz;    /**< Peripheral clock feeding the BRR.*/
    uint32_t            baud;       /**< Requested baud rate.             */
    uint32_t            brr;        /**< BRR value actually programmed.   */
    uint32_t            de_hold_us; /**< RS485 guard before DE release.   */
    uint8_t             rx_scratch[TP_STM32_RX_DMA_BYTES]; /**< Circular RX. */
    uint8_t             tx_storage[TP_STM32_BUS_RING_BYTES];
    uint8_t             rx_storage[TP_STM32_BUS_RING_BYTES];
    tp_ring_buffer_t    tx;         /**< Producer: tasks. Consumer: DMA IRQ.*/
    tp_ring_buffer_t    rx;         /**< Producer: USART IRQ. Consumer: tasks.*/
    volatile uint32_t   rx_pos;     /**< Bytes already harvested from RX DMA.*/
    volatile uint32_t   tx_len;     /**< Bytes in the in-flight TX transfer.*/
    volatile uint8_t    tx_active;  /**< Non-zero while TX DMA is running. */
    volatile uint8_t    de_asserted;/**< RS485 DE pin level.              */
    volatile uint8_t    in_use;     /**< Set once initialised.            */
    uint32_t            errors;     /**< Parity/framing/noise/overrun count.*/
} tp_stm32_uart_t;

/** @brief One received CAN frame, stored de-framed. */
typedef struct {
    uint32_t id;        /**< Standard identifier, 11 bits.                */
    uint8_t  dlc;       /**< Data length code.                            */
    uint8_t  data[8];   /**< Payload, zero-padded past @ref dlc.          */
} tp_stm32_can_frame_t;

/** @brief CAN1 state, including the software receive FIFO. */
typedef struct {
    tp_stm32_can_frame_t rx[TP_STM32_CAN_RX_FRAMES]; /**< SPSC ring of frames. */
    volatile uint32_t    rx_head;   /**< Frames ever pushed.                 */
    volatile uint32_t    rx_tail;   /**< Frames ever popped.                 */
    volatile uint32_t    rx_dropped;/**< Frames lost to a full software ring.*/
    volatile uint32_t    bus_off;   /**< Latched bus-off indication.         */
    uint32_t             btr;       /**< Bit-timing register value.          */
    uint32_t             bitrate;   /**< Achieved bit rate, bit/s.           */
    uint32_t             sample_tenths; /**< Sample point, tenths of a percent.*/
    uint8_t              initialised;
} tp_stm32_can_t;

/* =========================================================================
 * Module state
 * ========================================================================= */

static tp_stm32_uart_t  s_uart[TP_STM32_UART_COUNT];
static tp_stm32_can_t   s_can;

static TIM_HandleTypeDef s_htim1; /**< PWM, 20 kHz.                          */
static TIM_HandleTypeDef s_htim2; /**< Joint encoder.                        */
static TIM_HandleTypeDef s_htim4; /**< Gripper encoder.                      */
static TIM_HandleTypeDef s_htim3; /**< ADC trigger, 500 Hz.                  */
static ADC_HandleTypeDef s_hadc1; /**< Scan of four channels.                */

/** @brief ADC DMA destination: 16 complete scans of @ref TP_ADC_COUNT channels. */
static uint16_t s_adc_dma[TP_ADC_DMA_BUFFER_LEN * TP_ADC_COUNT]
    __attribute__((aligned(4)));

/** @brief Which ADC channel each DMA rank samples, index = @ref tp_adc_ch_t. */
static const uint32_t s_adc_channel[TP_ADC_COUNT] = {
    ADC_CHANNEL_4, /**< TP_ADC_FORCE            -> PA4, ADC1_IN4 */
    ADC_CHANNEL_5, /**< TP_ADC_CURRENT_JOINT    -> PA5, ADC1_IN5 */
    ADC_CHANNEL_2, /**< TP_ADC_CURRENT_GRIPPER  -> PA2, ADC1_IN2 */
    ADC_CHANNEL_3  /**< TP_ADC_VBUS             -> PA3, ADC1_IN3 */
};

static volatile uint16_t s_adc_latest[TP_ADC_COUNT]; /**< Newest raw counts. */
static volatile uint8_t  s_adc_valid[TP_ADC_COUNT];  /**< Whether above is real. */
static volatile uint8_t  s_adc_block_ready;          /**< A half-buffer completed. */
static volatile uint32_t s_adc_block_base;           /**< First scan of that half. */
static volatile uint32_t s_adc_blocks;               /**< Half-buffers completed. */
static volatile uint32_t s_adc_dma_errors;           /**< DMA transfer errors. */

static volatile float   s_pwm[TP_MOTOR_COUNT];       /**< Last commanded duty. */
static volatile uint8_t s_enabled[TP_MOTOR_COUNT];   /**< Software enable state. */
static volatile uint8_t s_killed;                    /**< Emergency kill latch. */

static volatile uint8_t  s_initialised;              /**< Full platform bring-up done. */
static volatile uint8_t  s_time_ok;                  /**< TIM5/DWT usable. */
static volatile uint8_t  s_dwt_ok;                   /**< Cycle counter counts. */
static volatile uint32_t s_critical_depth;           /**< Nesting diagnostics. */

static uint32_t s_reset_flags;                       /**< `RCC->CSR` at boot. */

/** @brief TIM7 prescaler, computed once at bring-up for the DE-release timer. */
static uint32_t s_tim7_psc;

static uint8_t  s_iwdg_armed;
static uint32_t s_iwdg_timeout_ms;
static uint32_t s_iwdg_div;
static uint32_t s_iwdg_reload;

/* =========================================================================
 * Small register helpers
 * ========================================================================= */

/**
 * @brief Timer kernel clock for a given APB peripheral clock.
 *
 * On STM32F4 the timer kernel clock is the APB clock when the APB prescaler is
 * 1, and **twice** the APB clock otherwise.  With PCLK1 = 42 MHz and
 * PCLK2 = 84 MHz against HCLK = 168 MHz, TIM2/3/4/5/7 run at 84 MHz and TIM1
 * runs at 168 MHz.  Deriving this rather than hard-coding it means a change to
 * the prescalers in `system_stm32f4xx.c` cannot silently double every period.
 *
 * @param[in] pclk_hz  `HAL_RCC_GetPCLK1Freq()` or `HAL_RCC_GetPCLK2Freq()`.
 * @return Timer kernel clock, Hz.
 */
static uint32_t timer_kernel_clock(uint32_t pclk_hz)
{
    return (pclk_hz == HAL_RCC_GetHCLKFreq()) ? pclk_hz : (pclk_hz * 2u);
}

/**
 * @brief Set one BSRR bit-pair on a port without disturbing any other pin.
 *
 * Writes go through BSRR rather than ODR so that setting a pin is a single
 * atomic 32-bit store: a read-modify-write of ODR would be a race against any
 * interrupt that also touches the same port, which is exactly how a "spurious
 * motor enable" bug is born.
 *
 * @param[in] port  GPIOA..GPIOE.
 * @param[in] pin   0..15.
 * @param[in] level Non-zero for high.
 */
static void gpio_write(GPIO_TypeDef *port, uint32_t pin, int level)
{
    if (level != 0) {
        port->BSRR = (uint32_t)1u << pin;
    } else {
        port->BSRR = (uint32_t)1u << (pin + 16u);
    }
}

/** @brief Write the joint (0) or gripper (1) direction pin. */
static void motor_dir_write(int ch, int level)
{
    gpio_write(GPIOE, (ch == 0) ? 0u : 2u, level); /* PE0 / PE2 */
}

/**
 * @brief Drive the joint (0) or gripper (1) gate-driver enable pin.
 *
 * The pin follows the *effective* state, not the requested one: while the kill
 * latch is set the pin stays low even if a caller asks for enable.  The software
 * state (`s_enabled`) still records the request, so `tp_platform_motor_is_enabled`
 * reports exactly what the host backend would report, and a subsequent
 * @ref tp_platform_emergency_release restores the pin.
 */
static void motor_enable_apply(int ch)
{
    int level = ((s_enabled[ch] != 0u) && (s_killed == 0u)) ? 1 : 0;

    gpio_write(GPIOE, (ch == 0) ? 1u : 3u, level); /* PE1 / PE3 */
}

/** @brief Assert or release the RS485 transceiver's driver-enable pin (PD12). */
static void rs485_de_write(int level)
{
    gpio_write(GPIOD, 12u, level);
}

/**
 * @brief Write a motor's PWM compare register and direction pin.
 *
 * @param[in] ch        @ref TP_MOTOR_JOINT or @ref TP_MOTOR_GRIPPER.
 * @param[in] permille  Signed duty, already clamped and finite.
 */
static void pwm_apply(int ch, float permille)
{
    float    magnitude = TP_FABS(permille);
    uint32_t ccr;

    /* 1000 per-mille maps to a full period: ccr = |duty| * ARR / 1000. */
    ccr = (uint32_t)((magnitude * (float)TP_STM32_PWM_ARR / 1000.0f) + 0.5f);
    if (ccr > TP_STM32_PWM_ARR) {
        ccr = TP_STM32_PWM_ARR;
    }

    /* The sign selects the H-bridge leg, i.e. the DIR pin; the magnitude is the
     * duty on the single PWM output. */
    motor_dir_write(ch, (permille >= 0.0f) ? 1 : 0);

    if (ch == 0) {
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, ccr);
    } else {
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, ccr);
    }
}

/* =========================================================================
 * DMA stream configuration (register level, shared by ADC and all three UARTs)
 * ========================================================================= */

/** @brief DMA data size encodings for `DMA_SxCR.PSIZE`/`MSIZE`. */
#define TP_DMA_SIZE_BYTE 0u
#define TP_DMA_SIZE_HALF 1u

/**
 * @brief Translate a (controller, stream) pair into its register block.
 *
 * @param[in] dma    1 or 2.
 * @param[in] stream 0..7.
 * @return Stream instance, or NULL when the pair does not exist.
 */
static DMA_Stream_TypeDef *dma_stream_instance(int dma, int stream)
{
    if ((stream < 0) || (stream > 7)) {
        return NULL;
    }

#define TP_DMA_CASE(n)                                                        \
    case n:                                                                   \
        return (dma == 1) ? DMA1_Stream##n : DMA2_Stream##n

    switch (stream) {
    TP_DMA_CASE(0);
    TP_DMA_CASE(1);
    TP_DMA_CASE(2);
    TP_DMA_CASE(3);
    TP_DMA_CASE(4);
    TP_DMA_CASE(5);
    TP_DMA_CASE(6);
    TP_DMA_CASE(7);
    default:
        break;
    }
#undef TP_DMA_CASE

    return NULL;
}

/**
 * @brief Program one DMA stream, leaving it disabled.
 *
 * Deliberately a hand-written equivalent of `HAL_DMA_Init` plus
 * `HAL_DMA_Start_IT`: the DMA is shared between the ADC and six UART streams, and
 * the interrupt flags are cleared by this file's own handler (see
 * @ref tp_stm32_dma_irq_handler).  Mixing that with the HAL's DMA state machine,
 * which also clears the same flags, would be a race for no benefit.
 *
 * @param[in] stream    Stream instance.
 * @param[in] channel   DMA request channel, 0..7.
 * @param[in] mem_to_peri Non-zero for memory -> peripheral, 0 for the reverse.
 * @param[in] circular  Non-zero for circular mode.
 * @param[in] mem_inc   Non-zero to increment the memory address.
 * @param[in] psize     @ref TP_DMA_SIZE_BYTE or @ref TP_DMA_SIZE_HALF.
 * @param[in] msize     @ref TP_DMA_SIZE_BYTE or @ref TP_DMA_SIZE_HALF.
 * @param[in] peri_addr Peripheral register address.
 * @param[in] mem_addr  Memory buffer address.
 * @param[in] count     Number of transfers.
 * @param[in] irq_mask  `DMA_IT_*` bits to enable in CR.
 */
static void dma_stream_config(DMA_Stream_TypeDef *stream, uint32_t channel,
                              int mem_to_peri, int circular, int mem_inc,
                              uint32_t psize, uint32_t msize,
                              uint32_t peri_addr, uint32_t mem_addr,
                              uint32_t count, uint32_t irq_mask)
{
    uint32_t cr;

    if (stream == NULL) {
        return;
    }

    /* CHSEL must not change while the stream is enabled. */
    stream->CR &= ~DMA_SxCR_EN;
    while ((stream->CR & DMA_SxCR_EN) != 0u) {
        /* Wait for the stream to acknowledge the disable.  Bounded in practice:
         * the hardware clears EN within a few AHB cycles. */
    }

    cr = ((channel << DMA_SxCR_CHSEL_Pos) & DMA_SxCR_CHSEL)
       | ((psize << DMA_SxCR_PSIZE_Pos) & DMA_SxCR_PSIZE)
       | ((msize << DMA_SxCR_MSIZE_Pos) & DMA_SxCR_MSIZE)
       | DMA_SxCR_PL_1 /* High priority: these streams all feed real-time paths. */
       | (mem_inc ? DMA_SxCR_MINC : 0u)
       | (circular ? DMA_SxCR_CIRC : 0u)
       | (mem_to_peri ? DMA_SxCR_DIR_0 : 0u)
       | (irq_mask & (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));

    stream->CR = cr;
    stream->FCR = 0u; /* Direct mode: no FIFO, as the HAL does for byte streams. */
    stream->PAR = peri_addr;
    stream->M0AR = mem_addr;
    stream->NDTR = count;
}

/* =========================================================================
 * Interrupt flag helpers
 * ========================================================================= */

/**
 * @brief Read and clear the interrupt flags of one DMA stream.
 *
 * @param[in]  dma     1 or 2.
 * @param[in]  stream  0..7.
 * @param[out] flags   Receives the pre-clear interrupt status.
 */
static void dma_flags_clear(int dma, int stream, uint32_t *flags)
{
    uint32_t value;

    if (dma == 1) {
        if (stream < 4) {
            uint32_t shift = (uint32_t)stream * 6u;
            value = (DMA1->LISR >> shift) & 0x3Fu;
            DMA1->LIFCR = value << shift;
        } else {
            uint32_t shift = (uint32_t)(stream - 4) * 6u;
            value = (DMA1->HISR >> shift) & 0x3Fu;
            DMA1->HIFCR = value << shift;
        }
    } else {
        if (stream < 4) {
            uint32_t shift = (uint32_t)stream * 6u;
            value = (DMA2->LISR >> shift) & 0x3Fu;
            DMA2->LIFCR = value << shift;
        } else {
            uint32_t shift = (uint32_t)(stream - 4) * 6u;
            value = (DMA2->HISR >> shift) & 0x3Fu;
            DMA2->HIFCR = value << shift;
        }
    }
    *flags = value;
}

/* Flag bit positions inside the 6-bit per-stream group: FEIF, DMEIF, TEIF, HTIF,
 * TCIF are bits 0, 2, 3, 4, 5 of the group (bit 1 is reserved). */
#define TP_DMA_GRP_FEIF  (1u << 0)
#define TP_DMA_GRP_DMEIF (1u << 2)
#define TP_DMA_GRP_TEIF  (1u << 3)
#define TP_DMA_GRP_HTIF  (1u << 4)
#define TP_DMA_GRP_TCIF  (1u << 5)

/* =========================================================================
 * ADC
 * ========================================================================= */

/**
 * @brief Publish the last complete scan of a finished DMA half-buffer.
 *
 * Called from the DMA2 Stream0 interrupt.  Only indices inside the half that has
 * *finished* are read, so no half-word can be torn by a concurrent DMA write.
 *
 * @param[in] base_scan Index of the first scan of the completed half.
 */
static void adc_publish_half(uint32_t base_scan)
{
    const uint32_t half = TP_ADC_DMA_BUFFER_LEN / 2u;
    const uint32_t last = base_scan + half - 1u;
    uint32_t i;

    for (i = 0u; i < (uint32_t)TP_ADC_COUNT; ++i) {
        s_adc_latest[i] = s_adc_dma[(last * (uint32_t)TP_ADC_COUNT) + i];
        s_adc_valid[i] = 1u;
    }
    s_adc_block_base = base_scan;
    s_adc_block_ready = 1u;
    s_adc_blocks++;
}

/* =========================================================================
 * USART: transmit path
 * ========================================================================= */

/**
 * @brief Cancel any pending RS485 driver-enable release.
 *
 * Called when new bytes are queued so that a multi-chunk frame cannot have DE
 * dropped in a gap between chunks: the transceiver must stay asserted for the
 * whole frame, otherwise another node can grab the pair mid-frame.
 */
static void rs485_de_hold(void)
{
    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7->DIER &= ~TIM_DIER_UIE;
    TIM7->SR = 0u;
    if (s_uart[TP_STM32_UART_RS485].de_asserted == 0u) {
        rs485_de_write(1);
        s_uart[TP_STM32_UART_RS485].de_asserted = 1u;
    }
}

/**
 * @brief Arm the one-pulse TIM7 that releases RS485 DE once the line is idle.
 *
 * # Why a timer and not the USART's TC interrupt
 *
 * The obvious implementation is to enable `TCIE` when the DMA finishes and drop
 * DE in the TC interrupt.  It does not work reliably here: `TC` is a sticky flag
 * whose only software clear is "read SR, then write DR", and writing DR with an
 * idle transmitter **transmits a byte**.  A stale TC left over from the previous
 * frame would therefore fire the interrupt immediately and truncate the current
 * one.  A one-shot timer sidesteps the flag entirely.  The delay is four
 * character times, which comfortably covers the shift register plus the transmit
 * register plus the transmitter's start delay, and is still far shorter than the
 * Modbus RTU 3.5-character inter-frame gap that follows it.
 */
static void rs485_de_arm_release(void)
{
    tp_stm32_uart_t *u = &s_uart[TP_STM32_UART_RS485];
    uint32_t ticks = u->de_hold_us;

    if (ticks == 0u) {
        ticks = 1u;
    }
    if (ticks > 65535u) {
        ticks = 65535u;
    }

    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7->DIER &= ~TIM_DIER_UIE;
    TIM7->PSC = s_tim7_psc;
    TIM7->ARR = ticks;
    TIM7->CNT = 0u;
    TIM7->EGR = TIM_EGR_UG;  /* Load PSC into its shadow register immediately. */
    TIM7->SR = 0u;
    /* URS: only a counter overflow generates an update event, so the UG above
     * does not re-arm the interrupt we are about to enable. */
    TIM7->CR1 = TIM_CR1_URS | TIM_CR1_OPM;
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Start the next TX DMA transfer if the ring has bytes and the line is idle.
 *
 * Restarts from `ring.tail`, sending the longest contiguous run so that no copy
 * into a bounce buffer is needed.  The transfer length is remembered so that the
 * completion interrupt can discard exactly those bytes.
 *
 * @param[in] idx  Index into @ref s_uart.
 * @return Bytes handed to the DMA, 0 when there was nothing to send.
 */
static uint32_t uart_tx_kick(int idx)
{
    tp_stm32_uart_t *u = &s_uart[idx];
    uint32_t count;
    uint32_t start;
    uint32_t contiguous;

    if ((u->in_use == 0u) || (u->tx_active != 0u)) {
        return 0u;
    }

    count = tp_ring_buffer_count(&u->tx);
    if (count == 0u) {
        return 0u;
    }

    start = u->tx.tail & u->tx.mask;
    contiguous = u->tx.capacity - start;
    if (contiguous > count) {
        contiguous = count;
    }

    /* M0AR and NDTR may only be written while the stream is disabled. */
    u->tx_dma->CR &= ~DMA_SxCR_EN;
    while ((u->tx_dma->CR & DMA_SxCR_EN) != 0u) {
        /* bounded: the hardware clears EN within a few cycles */
    }
    u->tx_dma->PAR = (uint32_t)&u->regs->DR;
    u->tx_dma->M0AR = (uint32_t)&u->tx.data[start];
    u->tx_dma->NDTR = contiguous;
    u->tx_len = contiguous;
    u->tx_active = 1u;
    u->tx_dma->CR |= DMA_SxCR_EN;

    return contiguous;
}

/**
 * @brief Handle the DMA transfer-complete of a UART transmitter.
 *
 * @param[in] idx  Index into @ref s_uart.
 */
static void uart_tx_complete(int idx)
{
    tp_stm32_uart_t *u = &s_uart[idx];

    if (u->tx_active == 0u) {
        return;
    }
    u->tx_active = 0u;
    (void)tp_ring_buffer_discard(&u->tx, u->tx_len);
    u->tx_len = 0u;
    u->tx_dma->CR &= ~DMA_SxCR_EN;
    while ((u->tx_dma->CR & DMA_SxCR_EN) != 0u) {
        /* bounded */
    }

    if (tp_ring_buffer_count(&u->tx) > 0u) {
        /* More of the same frame (or the next one) is queued: keep DE asserted
         * and keep the DMA running so there is no gap on the pair. */
        (void)uart_tx_kick(idx);
    } else if (idx == TP_STM32_UART_RS485) {
        rs485_de_arm_release();
    }
}

/* =========================================================================
 * USART: receive path
 * ========================================================================= */

/**
 * @brief Map a @ref tp_bus_t to the USART table index.
 * @param[in] bus  Bus selector.
 * @return Index into @ref s_uart, or @ref TP_STM32_BUS_NONE when @p bus is not a
 *         UART.
 */
static uint32_t bus_to_uart(tp_bus_t bus)
{
    switch (bus) {
    case TP_BUS_DEBUG_UART:
        return TP_STM32_UART_DEBUG;
    case TP_BUS_VISION_UART:
        return TP_STM32_UART_VISION;
    case TP_BUS_RS485:
        return TP_STM32_UART_RS485;
    default:
        return TP_STM32_BUS_NONE;
    }
}

uint32_t tp_stm32_uart_isr_rx(tp_bus_t bus)
{
    uint32_t idx = bus_to_uart(bus);
    tp_stm32_uart_t *u;
    uint32_t written;
    uint32_t position;
    uint32_t pushed = 0u;

    if (idx == TP_STM32_BUS_NONE) {
        return 0u;
    }
    u = &s_uart[idx];
    if (u->in_use == 0u) {
        return 0u;
    }

    /* NDTR counts *down* from the programmed length, so the number of bytes the
     * circular DMA has written since it started is length - NDTR. */
    written = TP_STM32_RX_DMA_BYTES - (u->rx_dma->NDTR & 0xFFFFu);
    if (written > TP_STM32_RX_DMA_BYTES) {
        written = TP_STM32_RX_DMA_BYTES;
    }

    position = u->rx_pos;
    if (written == position) {
        return 0u; /* nothing new */
    }

    if (written > position) {
        pushed = tp_ring_buffer_write(&u->rx, &u->rx_scratch[position],
                                      written - position);
        if (pushed != (written - position)) {
            u->errors++;
        }
    } else {
        /* The DMA wrapped: two segments, tail then head. */
        uint32_t first = TP_STM32_RX_DMA_BYTES - position;
        uint32_t got = tp_ring_buffer_write(&u->rx, &u->rx_scratch[position],
                                            first);
        if (got != first) {
            u->errors++;
        }
        pushed = got;
        if (written > 0u) {
            got = tp_ring_buffer_write(&u->rx, &u->rx_scratch[0], written);
            if (got != written) {
                u->errors++;
            }
            pushed += got;
        }
    }

    /* A full wrap leaves the write pointer at 0, not at the buffer length. */
    u->rx_pos = (written == TP_STM32_RX_DMA_BYTES) ? 0u : written;
    return pushed;
}

void tp_stm32_uart_irq_handler(int idx)
{
    tp_stm32_uart_t *u;
    USART_TypeDef   *regs;
    uint32_t         sr;
    uint32_t         cr1;

    if ((idx < 0) || (idx >= (int)TP_STM32_UART_COUNT)) {
        return;
    }
    u = &s_uart[idx];
    if (u->in_use == 0u) {
        return;
    }
    regs = u->regs;

    /* Snapshot once: reading SR has side effects on the error flags, and a
     * second read later could see a different set. */
    sr = regs->SR;
    cr1 = regs->CR1;

    /* --- Idle line: end of a receive burst ---------------------------------
     * This is what makes variable-length frames work without a per-byte
     * interrupt.  Reading DR clears IDLE (it is an SR-read + DR-read sequence).
     * Reading DR also clears TC, which is harmless: TC is only ever polled. */
    if (((sr & USART_SR_IDLE) != 0u) && ((cr1 & USART_CR1_IDLEIE) != 0u)) {
        (void)regs->DR;
        (void)tp_stm32_uart_isr_rx((tp_bus_t)idx);
    }

    /* --- Overrun / framing / noise / parity -------------------------------
     * With the RX DMA running, RXNE is normally cleared by the DMA's own read of
     * DR.  If the DMA is not yet enabled, or a burst outran the DMA's request
     * latency, RXNE and ORE can both be set; reading DR clears both and loses
     * exactly one byte, which the frame CRC will catch.  Without this the
     * receiver would stall (on STM32F4 a set ORE blocks further reception until
     * it is cleared). */
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
        (void)regs->DR;
        u->errors++;

        /* Re-arm the receive DMA if the overrun stopped it.  In circular mode
         * the stream keeps running, but a transfer error can disable it. */
        if ((u->rx_dma->CR & DMA_SxCR_EN) == 0u) {
            u->rx_dma->CR |= DMA_SxCR_EN;
        }
    } else if ((sr & USART_SR_RXNE) != 0u) {
        /* Safety net before the DMA is armed: dropping the byte is better than
         * leaving RXNE set and letting the next one overrun. */
        (void)regs->DR;
    } else {
        /* No source identified; nothing to clear. */
    }
}

/* =========================================================================
 * DMA interrupt dispatch
 * ========================================================================= */

/** @brief Which UART owns a DMA TX stream, or @ref TP_STM32_BUS_NONE. */
static uint32_t tx_owner(int dma, int stream)
{
    DMA_Stream_TypeDef *inst = dma_stream_instance(dma, stream);
    uint32_t i;

    if (inst == NULL) {
        return TP_STM32_BUS_NONE;
    }
    for (i = 0u; i < (uint32_t)TP_STM32_UART_COUNT; ++i) {
        if ((s_uart[i].in_use != 0u) && (s_uart[i].tx_dma == inst)) {
            return i;
        }
    }
    return TP_STM32_BUS_NONE;
}

/** @brief Which UART owns a DMA RX stream, or @ref TP_STM32_BUS_NONE. */
static uint32_t rx_owner(int dma, int stream)
{
    DMA_Stream_TypeDef *inst = dma_stream_instance(dma, stream);
    uint32_t i;

    if (inst == NULL) {
        return TP_STM32_BUS_NONE;
    }
    for (i = 0u; i < (uint32_t)TP_STM32_UART_COUNT; ++i) {
        if ((s_uart[i].in_use != 0u) && (s_uart[i].rx_dma == inst)) {
            return i;
        }
    }
    return TP_STM32_BUS_NONE;
}

void tp_stm32_dma_irq_handler(int dma, int stream)
{
    uint32_t flags;
    uint32_t owner;

    dma_flags_clear(dma, stream, &flags);
    if (flags == 0u) {
        return;
    }

    /* --- ADC1, DMA2 Stream0 --------------------------------------------- */
    if ((dma == 2) && (stream == 0)) {
        if ((flags & (TP_DMA_GRP_TEIF | TP_DMA_GRP_DMEIF | TP_DMA_GRP_FEIF)) != 0u) {
            s_adc_dma_errors++;
        }
        if ((flags & TP_DMA_GRP_HTIF) != 0u) {
            adc_publish_half(0u);
        }
        if ((flags & TP_DMA_GRP_TCIF) != 0u) {
            adc_publish_half(TP_ADC_DMA_BUFFER_LEN / 2u);
        }
        return;
    }

    /* --- UART transmit --------------------------------------------------- */
    owner = tx_owner(dma, stream);
    if (owner != TP_STM32_BUS_NONE) {
        if ((flags & (TP_DMA_GRP_TEIF | TP_DMA_GRP_DMEIF | TP_DMA_GRP_FEIF)) != 0u) {
            s_uart[owner].errors++;
            /* A transfer error means the bytes in flight are lost; clear the
             * active flag so the next kick retries from the same ring tail. */
            s_uart[owner].tx_active = 0u;
            s_uart[owner].tx_len = 0u;
            s_uart[owner].tx_dma->CR &= ~DMA_SxCR_EN;
        }
        if ((flags & TP_DMA_GRP_TCIF) != 0u) {
            uart_tx_complete((int)owner);
        }
        return;
    }

    /* --- UART receive ---------------------------------------------------- */
    owner = rx_owner(dma, stream);
    if (owner != TP_STM32_BUS_NONE) {
        if ((flags & (TP_DMA_GRP_TEIF | TP_DMA_GRP_DMEIF | TP_DMA_GRP_FEIF)) != 0u) {
            s_uart[owner].errors++;
        }
        /* Harvest on both half and full completion.  Normally the IDLE interrupt
         * does this; these two cover a continuous stream that never goes idle,
         * which would otherwise silently wrap the scratch buffer. */
        if ((flags & (TP_DMA_GRP_HTIF | TP_DMA_GRP_TCIF)) != 0u) {
            (void)tp_stm32_uart_isr_rx((tp_bus_t)owner);
        }
    }
}

/* =========================================================================
 * CAN1, register level (no stm32f4xx_hal_can.c is available)
 * ========================================================================= */

/**
 * @brief Wait for a `CAN_MSR` condition with a bounded spin.
 *
 * The HAL is not available for CAN, so this replaces its timeout loops.  The
 * bound is in loop iterations rather than milliseconds on purpose: this runs
 * during bring-up, possibly before the millisecond time base exists, so it must
 * not depend on anything that does.
 *
 * @param[in] mask  Bits to test.
 * @param[in] set   Non-zero to wait for all bits set, 0 to wait for all clear.
 * @return 1 when the condition was met, 0 on timeout.
 */
static int can_wait_msr(uint32_t mask, int set)
{
    uint32_t spin = 0u;

    while (spin < 100000u) {
        uint32_t value = CAN1->MSR & mask;
        if (((set != 0) && (value == mask)) || ((set == 0) && (value == 0u))) {
            return 1;
        }
        spin++;
    }
    return 0;
}

/**
 * @brief Bring CAN1 from reset to normal mode at 500 kbit/s.
 *
 * The bit-timing arithmetic is derived in the file header comment
 * (`TP_STM32_CAN_BRP`/`TS1`/`TS2`).  Filters are configured with one 32-bit mask
 * filter in bank 0 accepting every identifier, because the identifier policy
 * belongs to `can_driver.c`, which installs its own filters through
 * `tp_can_set_filter()`; a single accept-all filter at bring-up means an
 * unexpected frame is visible in the logs rather than silently discarded.
 *
 * @return @ref TP_OK on success.
 */
static tp_status_t can_init(void)
{
    uint32_t btr;
    uint32_t total_tq;

    s_can.rx_head = 0u;
    s_can.rx_tail = 0u;
    s_can.rx_dropped = 0u;
    s_can.bus_off = 0u;

    btr = ((TP_STM32_CAN_SJW << CAN_BTR_SJW_Pos) & CAN_BTR_SJW)
        | ((TP_STM32_CAN_TS2 << CAN_BTR_TS2_Pos) & CAN_BTR_TS2)
        | ((TP_STM32_CAN_TS1 << CAN_BTR_TS1_Pos) & CAN_BTR_TS1)
        | ((TP_STM32_CAN_BRP << CAN_BTR_BRP_Pos) & CAN_BTR_BRP);
    s_can.btr = btr;

    total_tq = 1u + (TP_STM32_CAN_TS1 + 1u) + (TP_STM32_CAN_TS2 + 1u);
    s_can.bitrate = timer_kernel_clock(HAL_RCC_GetPCLK1Freq())
                    / ((TP_STM32_CAN_BRP + 1u) * total_tq);
    s_can.sample_tenths = (((1u + TP_STM32_CAN_TS1 + 1u) * 1000u) / total_tq);

    /* Leave sleep mode.  A part that was reset while its bus peers were talking
     * can come up asleep. */
    CAN1->MCR = (CAN1->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
    if (can_wait_msr(CAN_MSR_INAK, 1) == 0) {
        return TP_ERR_IO;
    }

    /* Configuration is only writable while INAK is set.  MCR bit 15 (RESET) reads
     * as zero, so a read-modify-write of this register is safe.
     *   ABOM: automatic bus-off recovery, so a wiring fault self-heals instead of
     *         latching the node out of the network.
     *   AWUM: automatic wake-up on bus activity.
     *   NART: cleared - automatic retransmission stays enabled, which is what a
     *         drive bus wants; `tp_platform_can_send` reports TP_ERR_OVERFLOW
     *         when all three mailboxes are busy, which is the correct signal for
     *         a saturated bus rather than silent loss.
     *   TXFP: cleared - transmit priority by identifier, so a low-ID emergency
     *         frame overtakes queued telemetry.
     *   RFLM: cleared - on FIFO overflow keep the newest message, because a stale
     *         command is worthless and a fresh one is not. */
    CAN1->MCR = (CAN1->MCR & ~(CAN_MCR_SLEEP | CAN_MCR_INRQ | CAN_MCR_TXFP
                               | CAN_MCR_RFLM | CAN_MCR_NART))
              | CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_INRQ;

    CAN1->BTR = btr;

    /* --- Filter bank 0: 32-bit mask mode, accept everything, into FIFO 0 --- */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FM1R &= ~CAN_FM1R_FBM0;   /* Mask mode (0), not identifier list.  */
    CAN1->FS1R |= CAN_FS1R_FSC0;    /* 32-bit scale.                        */
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0; /* Route to FIFO 0.                     */
    CAN1->sFilterRegister[0].FR1 = 0x00000000u; /* id  = 0 -> all match     */
    CAN1->sFilterRegister[0].FR2 = 0x00000000u; /* mask= 0 -> don't care    */
    CAN1->FA1R |= CAN_FA1R_FACT0;   /* Activate bank 0.                     */
    CAN1->FMR &= ~CAN_FMR_FINIT;

    /* Leave initialisation mode. */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (can_wait_msr(CAN_MSR_INAK, 0) == 0) {
        return TP_ERR_IO;
    }

    /* Interrupts: RX FIFO 0 not empty, and the error/status-change sources that
     * matter for a robot (bus-off, error warning/passive, FIFO overrun). */
    CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_BOFIE | CAN_IER_EPVIE
              | CAN_IER_EWGIE | CAN_IER_FOVIE0;

    s_can.initialised = 1u;
    return TP_OK;
}

/**
 * @brief Pop one frame from the hardware RX FIFO 0 into the software FIFO.
 * @return 1 when a frame was moved.
 */
static int can_rx_pop_hardware(void)
{
    tp_stm32_can_frame_t frame;
    uint32_t rir;
    uint32_t rdtr;
    uint32_t rdlr;
    uint32_t rdhr;
    uint32_t i;

    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0u) {
        return 0;
    }

    rir = CAN1->sFIFOMailBox[0].RIR;
    rdtr = CAN1->sFIFOMailBox[0].RDTR;
    rdlr = CAN1->sFIFOMailBox[0].RDLR;
    rdhr = CAN1->sFIFOMailBox[0].RDHR;

    /* Releasing the mailbox must happen after the payload has been read. */
    CAN1->RF0R = CAN_RF0R_RFOM0;

    frame.id = (rir & CAN_RI0R_STID) >> CAN_RI0R_STID_Pos;
    frame.dlc = (uint8_t)(rdtr & CAN_RDT0R_DLC);
    if (frame.dlc > 8u) {
        frame.dlc = 8u;
    }
    for (i = 0u; i < 4u; ++i) {
        frame.data[i] = (uint8_t)((rdlr >> (8u * i)) & 0xFFu);
        frame.data[i + 4u] = (uint8_t)((rdhr >> (8u * i)) & 0xFFu);
    }

    if ((s_can.rx_head - s_can.rx_tail) >= (uint32_t)TP_STM32_CAN_RX_FRAMES) {
        s_can.rx_dropped++;
        return 1; /* the frame is dropped, but the hardware FIFO is drained */
    }
    s_can.rx[s_can.rx_head & (TP_STM32_CAN_RX_FRAMES - 1u)] = frame;
    s_can.rx_head++;
    return 1;
}

void tp_stm32_can_irq_handler(int event)
{
    if (s_can.initialised == 0u) {
        return;
    }

    if (event == TP_STM32_CAN_EVENT_RX0) {
        while (can_rx_pop_hardware() != 0) {
            /* Drain everything pending: leaving messages in the FIFO would
             * immediately re-assert the interrupt. */
        }
        return;
    }

    /* Status change: bus-off, error warning/passive, FIFO overrun.  Reading MSR
     * and ESR clears the interrupt. */
    {
        uint32_t msr = CAN1->MSR;
        uint32_t esr = CAN1->ESR;

        if ((esr & CAN_ESR_BOFF) != 0u) {
            s_can.bus_off = 1u;
        }
        if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0u) {
            s_can.rx_dropped++;
            CAN1->RF0R = CAN_RF0R_FOVR0; /* write 1 to clear */
        }
        (void)msr;
        (void)CAN1->ESR; /* second read releases LEC-based interrupt sources */
    }
}

/* =========================================================================
 * IWDG, register level (no stm32f4xx_hal_iwdg.c is available)
 * ========================================================================= */

/**
 * @brief Solve the IWDG prescaler and reload for a requested timeout, then arm it.
 *
 * # Arithmetic
 *
 * The IWDG counts down at `f_LSI / (4 * 2^PR)`, where PR is the 3-bit prescaler
 * field (0..7 -> /4, /8, /16, /32, /64, /128, /256), and reloads to `RLR`.  The
 * timeout is therefore
 *
 *       t = (4 * 2^PR) * (RLR + 1) / f_LSI,        f_LSI = 32 000 Hz nominal.
 *
 * For the firmware's request of `TP_WDG_TIMEOUT_MS = 500 ms`:
 *
 *       (4 * 2^PR) * (RLR + 1) = 0.5 s * 32 000 Hz = 16 000 counts.
 *
 * **PR = 0 (/4), RLR = 3999:**
 *       t = 4 * 4000 / 32 000 = 0.5000 s  -- exact, and 3999 <= 4095 so it fits
 *       the 12-bit reload register.  Resolution 4/32 000 = 0.125 ms.
 *
 * **PR = 4 (/64), RLR = 249:** the classic alternative,
 *       t = 64 * 250 / 32 000 = 0.5000 s -- also exact, resolution 2 ms.
 *
 * Both are exact, so the search below prefers the **smallest prescaler**, which
 * gives the finest granularity for any future timeout that does not divide
 * evenly (e.g. 501 ms is exact at PR = 0 and is not at PR = 4).  The search is
 * general rather than a table because the timeout is an argument.
 *
 * # What the achievable value really is
 *
 * The STM32F407's LSI is not trimmed: the datasheet gives a typical 32 kHz and no
 * tight min/max.  Taking a generous +-50 % band (16..64 kHz) the *same* RLR
 * therefore yields
 *
 *       t_min = 16 000 / 64 000 Hz = 250 ms     (LSI fast)
 *       t_max = 16 000 / 16 000 Hz = 1000 ms    (LSI slow)
 *
 * so the honest statement is "500 ms nominal, 250..1000 ms over the whole LSI
 * tolerance band", and the real value should be *measured* on the bench by
 * capturing the LSI against the 168 MHz HSE (one TIM5 input capture on the LSI
 * output is the standard ST technique).  The number reported here is the nominal
 * one; `tp_stm32_iwdg_timeout_ms()` returns it and the boot log prints the band.
 */
static void iwdg_solve(uint32_t timeout_ms, uint32_t *pr, uint32_t *rlr)
{
    uint32_t best_pr = 0u;
    uint32_t best_rlr = 4095u;
    uint64_t best_error = 0xFFFFFFFFFFFFFFFFull;
    uint32_t p;

    for (p = 0u; p <= 7u; ++p) {
        uint64_t divider = 4ull << p;            /* 4, 8, 16, ... 256 */
        uint64_t numerator = (uint64_t)timeout_ms * (uint64_t)TP_STM32_LSI_HZ;
        uint64_t denominator = 1000ull * divider;
        uint64_t reload = (numerator + (denominator / 2ull)) / denominator;
        uint64_t actual_ms;
        uint64_t error;

        if (reload == 0ull) {
            reload = 1ull;
        }
        if (reload > 4096ull) {
            continue; /* RLR is 12 bits: 0..4095 holds 4000 max */
        }

        /* Round-trip the choice through the real formula. */
        actual_ms = (divider * reload * 1000ull) / (uint64_t)TP_STM32_LSI_HZ;
        error = (actual_ms > (uint64_t)timeout_ms)
                    ? (actual_ms - (uint64_t)timeout_ms)
                    : ((uint64_t)timeout_ms - actual_ms);

        if (error < best_error) {
            best_error = error;
            best_pr = p;
            best_rlr = (uint32_t)reload - 1u;
            if (error == 0ull) {
                break; /* smallest prescaler that is exact */
            }
        }
    }

    *pr = best_pr;
    *rlr = best_rlr;
}

/* =========================================================================
 * Bring-up helpers
 * ========================================================================= */

/** @brief Configure one pin as a push-pull output, starting low. */
static void gpio_config_output_low(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef init = { 0 };

    /* Set the level *before* the mode so the pin never glitches high while the
     * gate driver is already powered. */
    gpio_write(port, pin, 0);
    init.Pin = (uint32_t)1u << pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW; /* DIR/EN and DE are static signals. */
    HAL_GPIO_Init(port, &init);
}

/** @brief Configure one pin as an alternate function. */
static void gpio_config_af(GPIO_TypeDef *port, uint32_t pin, uint8_t af,
                           uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef init = { 0 };

    init.Pin = (uint32_t)1u << pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = pull;
    init.Speed = speed;
    init.Alternate = af;
    HAL_GPIO_Init(port, &init);
}

/** @brief Configure one pin as an analogue input for the ADC. */
static void gpio_config_analog(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef init = { 0 };

    init.Pin = (uint32_t)1u << pin;
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &init);
}

/** @brief Configure one pin as a digital input with a specified pull. */
static void gpio_config_input(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    GPIO_InitTypeDef init = { 0 };

    init.Pin = (uint32_t)1u << pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    HAL_GPIO_Init(port, &init);
}

/**
 * @brief Start TIM5 as a free-running 1 kHz, 32-bit counter.
 *
 * 84 MHz / 84 000 = 1 kHz, and `ARR = 0xFFFFFFFF` makes the counter wrap every
 * 2^32 ms = 49.7 days, exactly the wrap the API documents.  Using a hardware
 * counter rather than the RTOS tick means the timestamp does not lose time while
 * interrupts are masked (see the file header).
 *
 * @return @ref TP_OK.
 */
static tp_status_t timebase_init(void)
{
    uint32_t kernel = timer_kernel_clock(HAL_RCC_GetPCLK1Freq());

    __HAL_RCC_TIM5_CLK_ENABLE();

    TIM5->CR1 = 0u;
    TIM5->PSC = (kernel / 1000u) - 1u;   /* 84 000 - 1 -> 1 kHz */
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->CNT = 0u;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0u;
    TIM5->CR1 = TIM_CR1_CEN;

    /* Enable the coarse (DWT) cycle counter for microsecond delays. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    {
        uint32_t before = DWT->CYCCNT;
        volatile uint32_t spin;
        for (spin = 0u; spin < 64u; ++spin) {
            TP_NOP();
        }
        s_dwt_ok = (DWT->CYCCNT != before) ? 1u : 0u;
    }

    s_time_ok = 1u;
    return TP_OK;
}

/**
 * @brief Configure TIM1 for two independent 20 kHz PWM channels.
 *
 * Edge-aligned; see the file header for why.  168 MHz / 20 kHz = 8400 counts, so
 * `ARR = 8399`.  Repetition counter 0 (TIM1 is an advanced-control timer and
 * requires the field to be written).
 *
 * @return @ref TP_OK or @ref TP_ERR_IO.
 */
static tp_status_t pwm_init(void)
{
    TIM_OC_InitTypeDef oc = { 0 };
    uint32_t kernel = timer_kernel_clock(HAL_RCC_GetPCLK2Freq());
    uint32_t arr = (kernel / TP_STM32_PWM_HZ);

    if (arr == 0u) {
        return TP_ERR_IO;
    }

    __HAL_RCC_TIM1_CLK_ENABLE();

    s_htim1.Instance = TIM1;
    s_htim1.Init.Prescaler = 0u;
    s_htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim1.Init.Period = arr - 1u;
    s_htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim1.Init.RepetitionCounter = 0u;
    s_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim1) != HAL_OK) {
        return TP_ERR_IO;
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0u; /* Commanded duty is zero until the control loop says otherwise. */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&s_htim1, &oc, TIM_CHANNEL_1) != HAL_OK) {
        return TP_ERR_IO;
    }
    if (HAL_TIM_PWM_ConfigChannel(&s_htim1, &oc, TIM_CHANNEL_2) != HAL_OK) {
        return TP_ERR_IO;
    }
    if (HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_1) != HAL_OK) {
        return TP_ERR_IO;
    }
    if (HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_2) != HAL_OK) {
        return TP_ERR_IO;
    }

    /* Belt and braces: the compare registers are zero, but a spurious write by a
     * debugger or a bad vector could leave them non-zero, so set them explicitly. */
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, 0u);
    return TP_OK;
}

/**
 * @brief Configure one timer as a 32-bit X4 quadrature encoder.
 *
 * `SMS = 111` (encoder mode 3) counts on both edges of both channels, which is
 * the X4 decoding `bsp_types.h` documents.  The input filter (`IC1Filter = 6`,
 * fDTS/32 with 8 samples) is not optional on a real machine: an unloaded
 * quadrature line rings on every edge, and a filter that is too aggressive is far
 * cheaper than a velocity estimate full of spikes.
 *
 * @param[in] htim   Handle to fill in.
 * @param[in] inst   TIM2 or TIM4.
 * @return @ref TP_OK or @ref TP_ERR_IO.
 */
static tp_status_t encoder_init(TIM_HandleTypeDef *htim, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef encoder = { 0 };

    htim->Instance = inst;
    htim->Init.Prescaler = 0u;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFFFFFFu; /* 32-bit timer: never wraps in practice. */
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12; /* X4: both edges, both channels */
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 6u;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 6u;

    if (HAL_TIM_Encoder_Init(htim, &encoder) != HAL_OK) {
        return TP_ERR_IO;
    }
    __HAL_TIM_SET_COUNTER(htim, 0u);
    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return TP_ERR_IO;
    }
    return TP_OK;
}

/**
 * @brief Configure TIM3 as the 500 Hz ADC trigger.
 *
 * 84 MHz / 84 = 1 MHz, / 2000 = 500 Hz, which is `TP_SENSOR_FREQ_HZ`.  The
 * update event is exported on TRGO so the ADC samples on a hardware edge rather
 * than from software: a software-triggered scan would add jitter that the force
 * loop would see as noise, and it would need the sensor task to be exactly
 * on time.
 *
 * @return @ref TP_OK or @ref TP_ERR_IO.
 */
static tp_status_t adc_trigger_init(void)
{
    TIM_MasterConfigTypeDef master = { 0 };
    uint32_t kernel = timer_kernel_clock(HAL_RCC_GetPCLK1Freq());

    __HAL_RCC_TIM3_CLK_ENABLE();

    s_htim3.Instance = TIM3;
    s_htim3.Init.Prescaler = (kernel / 1000000u) - 1u; /* 1 MHz tick */
    s_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period = (1000000u / TP_SENSOR_FREQ_HZ) - 1u;
    s_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim3) != HAL_OK) {
        return TP_ERR_IO;
    }

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &master) != HAL_OK) {
        return TP_ERR_IO;
    }
    if (HAL_TIM_Base_Start(&s_htim3) != HAL_OK) {
        return TP_ERR_IO;
    }
    return TP_OK;
}

/**
 * @brief Configure ADC1 as a four-channel scan driven by DMA2 Stream0.
 *
 * The HAL configures the ADC (prescaler, resolution, trigger, ranks); the DMA
 * stream is programmed directly so that this file owns the interrupt flags.  Rank
 * order is the `tp_adc_ch_t` order via `s_adc_channel[]`, which is what makes
 * `s_adc_dma[scan * TP_ADC_COUNT + ch]` the natural indexing.
 *
 * @return @ref TP_OK or @ref TP_ERR_IO.
 */
static tp_status_t adc_init(void)
{
    ADC_ChannelConfTypeDef channel = { 0 };
    uint32_t i;

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    s_hadc1.Instance = ADC1;
    /* PCLK2 = 84 MHz; the ADC's maximum clock is 36 MHz (F407 datasheet), so /4
     * gives 21 MHz.  Worth stating rather than copying: /2 would be 42 MHz and
     * out of specification. */
    s_hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode = ENABLE;
    s_hadc1.Init.ContinuousConvMode = DISABLE;   /* TIM3 triggers each scan */
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.NbrOfDiscConversion = 0u;
    s_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion = (uint32_t)TP_ADC_COUNT;
    s_hadc1.Init.DMAContinuousRequests = ENABLE;
    s_hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) {
        return TP_ERR_IO;
    }

    for (i = 0u; i < (uint32_t)TP_ADC_COUNT; ++i) {
        channel.Channel = s_adc_channel[i];
        channel.Rank = i + 1u;
        /* 84 ADC cycles = 4 us at 21 MHz.  The load-cell amplifier drives an RC
         * filter whose source impedance is high enough that a shorter sample
         * window would not settle; the current-sense amplifiers are faster but
         * share the setting for simplicity, and 4 us of the 2 ms sensor period
         * is 0.2 % of the budget. */
        channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        channel.Offset = 0u;
        if (HAL_ADC_ConfigChannel(&s_hadc1, &channel) != HAL_OK) {
            return TP_ERR_IO;
        }
    }

    /* Clear any stale DMA flags before enabling the stream. */
    {
        uint32_t dummy;
        dma_flags_clear(2, 0, &dummy);
    }

    dma_stream_config(DMA2_Stream0, 0u /* channel 0 */,
                      0 /* peripheral -> memory */, 1 /* circular */,
                      1 /* memory increment */,
                      TP_DMA_SIZE_HALF, TP_DMA_SIZE_HALF,
                      (uint32_t)&ADC1->DR, (uint32_t)s_adc_dma,
                      (uint32_t)TP_ADC_DMA_BUFFER_LEN * (uint32_t)TP_ADC_COUNT,
                      DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

    /* Enable the ADC.  With an external trigger configured this only powers the
     * converter up and waits for its stabilisation time; the first conversion
     * happens on the next TIM3 update. */
    if (HAL_ADC_Start(&s_hadc1) != HAL_OK) {
        return TP_ERR_IO;
    }

    /* DDS = 1: keep issuing DMA requests for every conversion instead of stopping
     * after the first block, which is what makes a circular scan work. */
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return TP_OK;
}

/**
 * @brief Bring up one UART, its pins, its two DMA streams and its rings.
 *
 * # Baud-rate arithmetic
 *
 * In 16x oversampling mode the divisor register holds `USARTDIV` in 4.12 fixed
 * point: BRR = round(f_CK / baud), because
 *
 *       baud = f_CK / (16 * USARTDIV)   and   BRR = USARTDIV * 16  =>  BRR = f_CK / baud.
 *
 * (OVER8 mode is not used: it has coarser fraction bits and, at 921 600 baud on
 * a 42 MHz APB1 clock, gives exactly the same result anyway.)
 *
 * @param[in] idx     Index into @ref s_uart.
 * @param[in] regs    USART instance.
 * @param[in] pclk_hz Peripheral clock feeding it.
 * @param[in] baud    Requested baud rate.
 * @param[in] tx_dma  TX DMA stream instance.
 * @param[in] tx_ch   TX DMA request channel.
 * @param[in] tx_num  TX DMA controller number, 1 or 2.
 * @param[in] tx_idx  TX DMA stream index, 0..7.
 * @param[in] rx_dma  RX DMA stream instance.
 * @param[in] rx_ch   RX DMA request channel.
 * @param[in] rx_num  RX DMA controller number, 1 or 2.
 * @param[in] rx_idx  RX DMA stream index, 0..7.
 * @param[in] irqn    USART interrupt number.
 * @return @ref TP_OK or @ref TP_ERR_IO.
 */
static tp_status_t uart_init(int idx, USART_TypeDef *regs, uint32_t pclk_hz,
                             uint32_t baud, DMA_Stream_TypeDef *tx_dma,
                             uint32_t tx_ch, int tx_num, int tx_idx,
                             DMA_Stream_TypeDef *rx_dma, uint32_t rx_ch,
                             int rx_num, int rx_idx, IRQn_Type irqn)
{
    tp_stm32_uart_t *u = &s_uart[idx];
    uint32_t brr;
    uint32_t dummy;

    u->regs = regs;
    u->tx_dma = tx_dma;
    u->rx_dma = rx_dma;
    u->pclk_hz = pclk_hz;
    u->baud = baud;
    u->de_asserted = 0u;
    u->tx_active = 0u;
    u->tx_len = 0u;
    u->rx_pos = 0u;
    u->errors = 0u;

    /* Round to nearest: BRR = f_CK / baud with 4 fraction bits. */
    brr = (pclk_hz + (baud / 2u)) / baud;
    if (brr < 16u) {
        /* Below 16 the mantissa is 0 and the divider is unusable: the requested
         * baud rate is above what this peripheral clock can produce. */
        return TP_ERR_RANGE;
    }
    u->brr = brr;

    /* RS485 DE guard: four character times at the configured baud rate.  The
     * driver adds its own TP_RS485_DE_GUARD_US on top; this one exists to keep
     * the *platform* holding DE until the shift register has emptied, which is
     * the part the driver cannot see. */
    u->de_hold_us = (4u * 10u * 1000000u) / baud;
    if (u->de_hold_us < 4u) {
        u->de_hold_us = 4u;
    }

    (void)tp_ring_buffer_init(&u->tx, u->tx_storage, sizeof(u->tx_storage));
    (void)tp_ring_buffer_init(&u->rx, u->rx_storage, sizeof(u->rx_storage));

    /* --- USART, register level ------------------------------------------ */
    regs->CR1 = 0u;   /* Disable the peripheral while it is reconfigured. */
    regs->CR2 = 0u;   /* 1 stop bit, no clock output.                     */
    regs->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    regs->BRR = brr;
    /* 8 data bits (M = 0), no parity (PCE = 0), TX + RX enabled, IDLE interrupt
     * on (the end-of-burst marker), UE last.  RXNEIE is deliberately *not* set:
     * the DMA reads DR, and a per-byte interrupt at 921 600 baud would be 92 k
     * interrupts per second for no benefit. */
    regs->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;
    (void)regs->SR;
    (void)regs->DR;

    /* --- Receive DMA: circular into the scratch buffer ------------------ */
    dma_flags_clear(rx_num, rx_idx, &dummy);
    dma_stream_config(rx_dma, rx_ch,
                      0 /* peripheral -> memory */, 1 /* circular */,
                      1 /* memory increment */,
                      TP_DMA_SIZE_BYTE, TP_DMA_SIZE_BYTE,
                      (uint32_t)&regs->DR, (uint32_t)u->rx_scratch,
                      TP_STM32_RX_DMA_BYTES,
                      DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

    /* --- Transmit DMA: normal mode, reprogrammed per transfer ----------- */
    dma_flags_clear(tx_num, tx_idx, &dummy);
    dma_stream_config(tx_dma, tx_ch,
                      1 /* memory -> peripheral */, 0 /* normal */,
                      1 /* memory increment */,
                      TP_DMA_SIZE_BYTE, TP_DMA_SIZE_BYTE,
                      (uint32_t)&regs->DR, (uint32_t)u->tx.data,
                      1u, /* Replaced by every kick; a zero length is illegal. */
                      DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

    u->rx_dma->CR |= DMA_SxCR_EN;
    u->in_use = 1u;

    HAL_NVIC_SetPriority(irqn, 3, 0); /* Above the HAL tick, below the ADC DMA. */
    HAL_NVIC_EnableIRQ(irqn);

    return TP_OK;
}

/* =========================================================================
 * platform_api.h implementation
 * ========================================================================= */

tp_status_t tp_platform_init(void)
{
    uint32_t kernel_pclk1;
    tp_status_t st;

    if (s_initialised != 0u) {
        return TP_OK; /* Idempotent, as the contract requires. */
    }

    /* --- 0. Reset diagnostics, before anything can clear the flags -------- */
    s_reset_flags = RCC->CSR;

    /* --- 1. Clocks ------------------------------------------------------- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* --- 2. Gate drivers to a safe state *before* their pins become
     *        outputs.  Writing ODR/BSRR while the pins are still inputs means
     *        the driver never sees a logic-high enable for even one cycle. */
    GPIOE->BSRR = (GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR2 | GPIO_BSRR_BR3
                   | GPIO_BSRR_BR11);
    GPIOD->BSRR = GPIO_BSRR_BR12;
    s_enabled[0] = 0u;
    s_enabled[1] = 0u;
    s_pwm[0] = 0.0f;
    s_pwm[1] = 0.0f;

    /* --- 3. GPIO --------------------------------------------------------- */
    gpio_config_output_low(GPIOE, 0u);  /* joint DIR      */
    gpio_config_output_low(GPIOE, 1u);  /* joint enable   */
    gpio_config_output_low(GPIOE, 2u);  /* gripper DIR    */
    gpio_config_output_low(GPIOE, 3u);  /* gripper enable */
    gpio_config_output_low(GPIOD, 12u); /* RS485 DE       */

    /* PWM: PA8 = TIM1_CH1, PE11 = TIM1_CH2.  Very high speed because a 20 kHz
     * square wave needs fast edges; a slow pin driver would round them off and
     * waste switching loss in the gate driver. */
    gpio_config_af(GPIOA, 8u, GPIO_AF1_TIM1, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOE, 11u, GPIO_AF1_TIM1, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);

    /* Encoders: pull-ups so that an unplugged encoder reads a stable level and
     * produces no counts, which the stall detector catches, rather than a
     * floating input that produces a random walk. */
    gpio_config_af(GPIOA, 0u, GPIO_AF1_TIM2, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    gpio_config_af(GPIOA, 1u, GPIO_AF1_TIM2, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    gpio_config_af(GPIOB, 6u, GPIO_AF2_TIM4, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    gpio_config_af(GPIOB, 7u, GPIO_AF2_TIM4, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);

    /* ADC inputs.  See the conflict resolution in the file header: force is on
     * PA4/IN4 and the joint current on PA5/IN5 because PA0/PA1 belong to TIM2. */
    gpio_config_analog(GPIOA, 4u);
    gpio_config_analog(GPIOA, 5u);
    gpio_config_analog(GPIOA, 2u);
    gpio_config_analog(GPIOA, 3u);

    /* UARTs.  RX pins get a pull-up so a disconnected cable reads idle-high
     * (which is the UART idle state) instead of picking up noise. */
    gpio_config_af(GPIOA, 9u, GPIO_AF7_USART1, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOA, 10u, GPIO_AF7_USART1, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOD, 5u, GPIO_AF7_USART2, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOD, 6u, GPIO_AF7_USART2, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOB, 10u, GPIO_AF7_USART3, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOB, 11u, GPIO_AF7_USART3, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);

    /* CAN1 on PD0/PD1, AF9. */
    gpio_config_af(GPIOD, 0u, GPIO_AF9_CAN1, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);
    gpio_config_af(GPIOD, 1u, GPIO_AF9_CAN1, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);

    /* Discrete inputs, PE4..PE9 contiguous so that one IDR read gets them all.
     *
     * PE4 (E-stop) is the exception: the loop is normally-closed to 3.3 V and the
     * pin has a **pull-down**, so
     *     level 1 = loop intact (safe)
     *     level 0 = loop open  (E-stop pressed, or the wire has fallen off)
     * which is the fail-safe polarity the API documents ("NC loop, high when
     * intact").  The other five switch to ground and therefore use pull-ups, so
     * for them 0 means "active".  Both polarities are returned raw; the driver
     * layer (`limit_switch.c`, `safety_monitor.c`) owns the interpretation, which
     * is what makes a mis-wired prototype visible in the logs. */
    gpio_config_input(GPIOE, 4u, GPIO_PULLDOWN);
    gpio_config_input(GPIOE, 5u, GPIO_PULLUP);
    gpio_config_input(GPIOE, 6u, GPIO_PULLUP);
    gpio_config_input(GPIOE, 7u, GPIO_PULLUP);
    gpio_config_input(GPIOE, 8u, GPIO_PULLUP);
    gpio_config_input(GPIOE, 9u, GPIO_PULLUP);

    /* --- 4. Time base first: the logger needs it and the HAL uses it ----- */
    st = timebase_init();
    if (st != TP_OK) {
        return st;
    }

    /* --- 5. Actuators, in their safe state ------------------------------ */
    st = pwm_init();
    if (st != TP_OK) {
        return st;
    }
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    st = encoder_init(&s_htim2, TIM2);
    if (st != TP_OK) {
        return st;
    }
    st = encoder_init(&s_htim4, TIM4);
    if (st != TP_OK) {
        return st;
    }

    /* --- 6. Sensing ------------------------------------------------------ */
    st = adc_trigger_init();
    if (st != TP_OK) {
        return st;
    }
    st = adc_init();
    if (st != TP_OK) {
        return st;
    }

    /* --- 7. Buses --------------------------------------------------------
     * DMA request mapping, RM0090 "DMA1/DMA2 request mapping" tables:
     *   USART1_TX = DMA2 Stream7 channel 4   USART1_RX = DMA2 Stream5 channel 4
     *   USART2_TX = DMA1 Stream6 channel 4   USART2_RX = DMA1 Stream5 channel 4
     *   USART3_TX = DMA1 Stream3 channel 4   USART3_RX = DMA1 Stream1 channel 4
     *   ADC1      = DMA2 Stream0 channel 0
     */
    kernel_pclk1 = HAL_RCC_GetPCLK1Freq();

    __HAL_RCC_USART1_CLK_ENABLE();
    st = uart_init(TP_STM32_UART_DEBUG, USART1, HAL_RCC_GetPCLK2Freq(),
                   TP_UART_DEBUG_BAUD, DMA2_Stream7, 4u, 2, 7,
                   DMA2_Stream5, 4u, 2, 5, USART1_IRQn);
    if (st != TP_OK) {
        return st;
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    st = uart_init(TP_STM32_UART_VISION, USART2, kernel_pclk1,
                   TP_UART_VISION_BAUD, DMA1_Stream6, 4u, 1, 6,
                   DMA1_Stream5, 4u, 1, 5, USART2_IRQn);
    if (st != TP_OK) {
        return st;
    }

    __HAL_RCC_USART3_CLK_ENABLE();
    st = uart_init(TP_STM32_UART_RS485, USART3, kernel_pclk1,
                   TP_RS485_BAUD, DMA1_Stream3, 4u, 1, 3,
                   DMA1_Stream1, 4u, 1, 1, USART3_IRQn);
    if (st != TP_OK) {
        return st;
    }

    /* TIM7: the one-pulse timer that releases RS485 DE once the line is idle.
     * Prescaled to 1 MHz so the guard delay is expressed directly in microseconds. */
    __HAL_RCC_TIM7_CLK_ENABLE();
    s_tim7_psc = (timer_kernel_clock(kernel_pclk1) / 1000000u) - 1u;
    TIM7->CR1 = 0u;
    TIM7->DIER = 0u;
    TIM7->SR = 0u;
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    /* --- 8. CAN1 --------------------------------------------------------- */
    __HAL_RCC_CAN1_CLK_ENABLE();
    st = can_init();
    if (st != TP_OK) {
        return st;
    }
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 4, 0);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

    /* --- 9. DMA and ADC interrupt priorities ------------------------------
     * The ADC DMA is the highest of the peripherals: it publishes the samples
     * the 1 kHz control loop reads, and a delayed half-buffer publish is a
     * delayed measurement.  All three are still below the RTOS-safe threshold
     * question only in that none of these handlers calls an RTOS API. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* --- 10. Mirror the host backend's post-init drive state --------------
     * `platform_host.c` leaves both channels enabled with zero duty, and a unit
     * test may assert exactly that, so the target does the same.  It is safe
     * because the compare registers are zero: an energised gate driver with a 0 %
     * duty cannot move a motor, and the application's self-test owns the decision
     * to disable the drives before motion is attempted. */
    s_enabled[0] = 1u;
    s_enabled[1] = 1u;
    motor_enable_apply(0);
    motor_enable_apply(1);

    if (s_killed != 0u) {
        /* Someone called tp_platform_emergency_kill() before init (the boot
         * self-test is allowed to).  Honour it now that the pins exist. */
        s_enabled[0] = 0u;
        s_enabled[1] = 0u;
        motor_enable_apply(0);
        motor_enable_apply(1);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, 0u);
        __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, 0u);
    }

    s_initialised = 1u;

    /* --- 11. Report what was actually configured ------------------------- */
    LOG_INFO(TP_MOD_MAIN,
             "stm32f407 platform up: SYSCLK %lu MHz, HCLK %lu MHz, PCLK1 %lu MHz, PCLK2 %lu MHz, flash 5WS",
             (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000u),
             (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000u),
             (unsigned long)(kernel_pclk1 / 1000000u),
             (unsigned long)(HAL_RCC_GetPCLK2Freq() / 1000000u));
    LOG_INFO(TP_MOD_MAIN,
             "pwm TIM1 20 kHz edge-aligned ARR=%lu, encoders X4 (TIM2/TIM4), adc 4ch @%lu Hz",
             (unsigned long)TP_STM32_PWM_ARR, (unsigned long)TP_SENSOR_FREQ_HZ);
    LOG_INFO(TP_MOD_COMM,
             "uart1 %lu baud (%ld ppm), uart2 %lu baud (%ld ppm), uart3 %lu baud (%ld ppm)",
             (unsigned long)tp_stm32_uart_actual_baud(TP_BUS_DEBUG_UART),
             (long)tp_stm32_uart_baud_error_ppm(TP_BUS_DEBUG_UART),
             (unsigned long)tp_stm32_uart_actual_baud(TP_BUS_VISION_UART),
             (long)tp_stm32_uart_baud_error_ppm(TP_BUS_VISION_UART),
             (unsigned long)tp_stm32_uart_actual_baud(TP_BUS_RS485),
             (long)tp_stm32_uart_baud_error_ppm(TP_BUS_RS485));
    LOG_INFO(TP_MOD_COMM,
             "can1 btr=0x%08lX -> %lu bit/s, sample point %lu.%lu %% (target 87.5 %%)",
             (unsigned long)s_can.btr, (unsigned long)s_can.bitrate,
             (unsigned long)(s_can.sample_tenths / 10u),
             (unsigned long)(s_can.sample_tenths % 10u));

    return TP_OK;
}

uint32_t tp_platform_time_ms(void)
{
    /* Before the timer starts the only honest answer is zero.  Returning the
     * counter of a stopped timer would report a constant that looks like a valid
     * elapsed time and would silently break every timeout comparison. */
    if (s_time_ok == 0u) {
        return 0u;
    }
    return (uint32_t)TIM5->CNT;
}

void tp_platform_delay_ms(uint32_t ms)
{
    uint32_t start;

    if (ms == 0u) {
        return;
    }
    if (s_time_ok == 0u) {
        /* No time base yet (very early bring-up).  A calibrated loop is not
         * possible without one, so this is a crude non-zero wait; it exists so
         * that a caller cannot spin forever. */
        volatile uint32_t spin;
        for (spin = 0u; spin < (ms * 20000u); ++spin) {
            TP_NOP();
        }
        return;
    }
    start = (uint32_t)TIM5->CNT;
    while (((uint32_t)TIM5->CNT - start) < ms) {
        /* Unsigned subtraction makes this correct across the 49.7-day wrap. */
    }
}

void tp_platform_delay_us(uint32_t us)
{
    uint32_t cycles;
    uint32_t start;

    if (us == 0u) {
        return;
    }

    if (s_dwt_ok == 0u) {
        /* The DWT cycle counter is not running (a debugger may have left the
         * trace unit disabled).  This fallback is a loop of NOPs whose iteration
         * count is derived from SystemCoreClock, so it is right to within the
         * pipeline's own uncertainty — a few tens of percent. */
        volatile uint32_t spin;
        uint32_t iterations = us * ((SystemCoreClock / 4000000u) + 1u);
        for (spin = 0u; spin < iterations; ++spin) {
            TP_NOP();
        }
        return;
    }

    /* 64-bit intermediate: us * 168 is fine in 32 bits today, but a future
     * 400 MHz part would overflow. */
    cycles = (uint32_t)(((uint64_t)us * (uint64_t)SystemCoreClock) / 1000000ull);
    start = DWT->CYCCNT;
    while (((uint32_t)DWT->CYCCNT - start) < cycles) {
        /* Unsigned subtraction handles the counter wrap. */
    }
}

tp_status_t tp_platform_io_read(tp_io_id_t id, uint8_t *level)
{
    if (level == NULL) {
        return TP_ERR_PARAM;
    }
    /* Unsigned-range test: an enumerator with only non-negative values is an
     * unsigned type here, so this single comparison also rejects a hostile
     * negative cast (which arrives as a huge unsigned value). */
    if ((uint32_t)id >= (uint32_t)TP_IO_COUNT) {
        return TP_ERR_PARAM;
    }
    *level = (uint8_t)((tp_platform_io_read_all() >> (int)id) & 0x1u);
    return TP_OK;
}

uint8_t tp_platform_io_read_all(void)
{
    /* One IDR read, because PE4..PE9 were chosen to be contiguous.  Before init
     * the GPIOE clock is off and this returns 0, i.e. "E-stop asserted", which is
     * the fail-safe answer for a call made too early. */
    return (uint8_t)((GPIOE->IDR >> TP_STM32_IO_SHIFT) & TP_STM32_IO_MASK);
}

tp_status_t tp_platform_motor_set_pwm(tp_motor_ch_t ch, float permille)
{
    float clamped;

    if ((uint32_t)ch >= (uint32_t)TP_MOTOR_COUNT) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(permille)) {
        return TP_ERR_RANGE;
    }
    if (s_killed != 0u) {
        /* The kill latch is what makes the E-stop meaningful rather than
         * advisory: a stray write from a confused task cannot re-energise a
         * drive.  The compare register is forced to zero on every attempt, and
         * the caller is told the write did not take effect. */
        s_pwm[ch] = 0.0f;
        pwm_apply((int)ch, 0.0f);
        return TP_ERR_FAULT_ACTIVE;
    }

    clamped = TP_CLAMP(permille, -TP_MAX_PWM_PERMILLE, TP_MAX_PWM_PERMILLE);
    s_pwm[ch] = clamped;
    pwm_apply((int)ch, clamped);
    return TP_OK;
}

tp_status_t tp_platform_motor_enable(tp_motor_ch_t ch, int enable)
{
    if ((uint32_t)ch >= (uint32_t)TP_MOTOR_COUNT) {
        return TP_ERR_PARAM;
    }
    s_enabled[ch] = (enable != 0) ? 1u : 0u;
    if (s_enabled[ch] == 0u) {
        /* Disabling must be the fastest possible reaction, so the duty is
         * zeroed here as well: the driver's coast/brake response should not
         * depend on the next control period arriving. */
        s_pwm[ch] = 0.0f;
        pwm_apply((int)ch, 0.0f);
    }
    motor_enable_apply((int)ch);
    return TP_OK;
}

int tp_platform_motor_is_enabled(tp_motor_ch_t ch)
{
    if ((uint32_t)ch >= (uint32_t)TP_MOTOR_COUNT) {
        return 0;
    }
    return (s_enabled[ch] != 0u) ? 1 : 0;
}

tp_status_t tp_platform_motor_current_ma(tp_motor_ch_t ch, float *ma)
{
    tp_adc_ch_t adc_ch;
    uint16_t counts;

    if (ma == NULL) {
        return TP_ERR_PARAM;
    }
    switch (ch) {
    case TP_MOTOR_JOINT:
        adc_ch = TP_ADC_CURRENT_JOINT;
        break;
    case TP_MOTOR_GRIPPER:
        adc_ch = TP_ADC_CURRENT_GRIPPER;
        break;
    default:
        return TP_ERR_PARAM;
    }
    if (s_adc_valid[adc_ch] == 0u) {
        return TP_ERR_NOT_READY;
    }
    counts = s_adc_latest[adc_ch];

    /* Same first-order shunt-amplifier mapping as the host backend
     * ((counts - 2048) * 5 mA) so that a host test and a bench run produce
     * comparable numbers.  The exact gain and offset are per-board calibration
     * data and belong in the calibration record, not hard-coded in the seam. */
    *ma = ((float)counts - 2048.0f) * 5.0f;
    return TP_OK;
}

tp_status_t tp_platform_encoder_read(tp_encoder_ch_t ch, int32_t *count)
{
    if (count == NULL) {
        return TP_ERR_PARAM;
    }
    switch (ch) {
    case TP_ENCODER_JOINT:
        /* A single 32-bit load: the hardware counter cannot be torn. */
        *count = (int32_t)__HAL_TIM_GET_COUNTER(&s_htim2);
        return TP_OK;
    case TP_ENCODER_GRIPPER:
        *count = (int32_t)__HAL_TIM_GET_COUNTER(&s_htim4);
        return TP_OK;
    default:
        return TP_ERR_PARAM;
    }
    /* Encoder plausibility (implausible acceleration, stall) is deliberately not
     * checked here: it is `encoder.c`'s job, it needs the sample history, and
     * duplicating it in the BSP would give two disagreeing definitions of a
     * "faulty" encoder.  Only the host backend can flag a hardware fault because
     * only the plant model has a fault injector. */
}

tp_status_t tp_platform_encoder_reset(tp_encoder_ch_t ch)
{
    switch (ch) {
    case TP_ENCODER_JOINT:
        __HAL_TIM_SET_COUNTER(&s_htim2, 0u);
        return TP_OK;
    case TP_ENCODER_GRIPPER:
        __HAL_TIM_SET_COUNTER(&s_htim4, 0u);
        return TP_OK;
    default:
        return TP_ERR_PARAM;
    }
}

tp_status_t tp_platform_adc_read(tp_adc_ch_t ch, uint16_t *counts)
{
    if (counts == NULL) {
        return TP_ERR_PARAM;
    }
    if ((uint32_t)ch >= (uint32_t)TP_ADC_COUNT) {
        return TP_ERR_PARAM;
    }
    if (s_adc_valid[ch] == 0u) {
        return TP_ERR_NOT_READY;
    }
    *counts = s_adc_latest[ch];
    return TP_OK;
}

uint32_t tp_platform_adc_read_block(tp_adc_ch_t ch, uint16_t *buffer,
                                    uint32_t count)
{
    uint32_t key;
    uint32_t base;
    uint32_t available;
    uint32_t i;
    uint32_t n;

    if ((buffer == NULL) || (count == 0u)) {
        return 0u;
    }
    if ((uint32_t)ch >= (uint32_t)TP_ADC_COUNT) {
        return 0u;
    }

    /* Take the completed half-buffer atomically: the DMA interrupt must not
     * publish a new half between reading the flag and reading the data. */
    key = tp_platform_enter_critical();
    if (s_adc_block_ready == 0u) {
        tp_platform_exit_critical(key);
        return 0u;
    }
    base = s_adc_block_base;
    s_adc_block_ready = 0u;
    tp_platform_exit_critical(key);

    available = TP_ADC_DMA_BUFFER_LEN / 2u;
    n = (count < available) ? count : available;
    for (i = 0u; i < n; ++i) {
        buffer[i] = s_adc_dma[((base + i) * (uint32_t)TP_ADC_COUNT)
                              + (uint32_t)ch];
    }
    return n;
}

tp_status_t tp_platform_bus_write(tp_bus_t bus, const uint8_t *data,
                                  uint32_t len)
{
    uint32_t idx;
    tp_stm32_uart_t *u;
    uint32_t key;

    if ((data == NULL) && (len > 0u)) {
        return TP_ERR_PARAM;
    }
    if ((uint32_t)bus >= (uint32_t)TP_BUS_COUNT) {
        return TP_ERR_PARAM;
    }
    if (bus == TP_BUS_CAN) {
        /* A CAN frame is not a byte stream, so there is nothing sensible for a
         * firmware module to write here.  The host backend accepts bytes on this
         * "bus" only because its CAN capture ring is a test convenience; on
         * silicon the honest answer is that the operation does not exist.
         * `can_driver.c` uses `tp_platform_can_send()` for real traffic. */
        return TP_ERR_UNSUPPORTED;
    }

    idx = bus_to_uart(bus);
    if (idx == TP_STM32_BUS_NONE) {
        return TP_ERR_PARAM;
    }
    u = &s_uart[idx];
    if (u->in_use == 0u) {
        return TP_ERR_NOT_READY;
    }
    if (tp_ring_buffer_write(&u->tx, data, len) != TP_OK) {
        return TP_ERR_OVERFLOW;
    }

    if (bus == TP_BUS_RS485) {
        /* Asserting DE and starting the DMA must not be interrupted by the
         * one-pulse DE-release timer, otherwise a multi-chunk frame would have
         * the pair released in the gap between chunks and another node could
         * start driving.  This critical section is a handful of instructions. */
        key = tp_platform_enter_critical();
        rs485_de_hold();
        (void)uart_tx_kick((int)idx);
        tp_platform_exit_critical(key);
    } else {
        (void)uart_tx_kick((int)idx);
    }
    return TP_OK;
}

uint32_t tp_platform_bus_read(tp_bus_t bus, uint8_t *data, uint32_t max_len)
{
    uint32_t idx;

    if ((uint32_t)bus >= (uint32_t)TP_BUS_COUNT) {
        return 0u;
    }
    if (data == NULL) {
        return 0u;
    }
    idx = bus_to_uart(bus);
    if (idx == TP_STM32_BUS_NONE) {
        return 0u;
    }
    if (s_uart[idx].in_use == 0u) {
        return 0u;
    }
    return tp_ring_buffer_read(&s_uart[idx].rx, data, max_len);
}

int tp_platform_bus_tx_done(tp_bus_t bus)
{
    uint32_t idx;
    tp_stm32_uart_t *u;

    if ((uint32_t)bus >= (uint32_t)TP_BUS_COUNT) {
        return 0;
    }
    if (bus == TP_BUS_CAN) {
        /* "Drained" for CAN means at least one transmit mailbox is free.  The
         * host backend has no mailbox concept and always reports 1, which is the
         * same answer whenever the bus is not saturated. */
        return ((CAN1->TSR & CAN_TSR_TME) != 0u) ? 1 : 0;
    }

    idx = bus_to_uart(bus);
    if (idx == TP_STM32_BUS_NONE) {
        return 0;
    }
    u = &s_uart[idx];
    if (u->in_use == 0u) {
        return 0;
    }
    if ((u->tx_active != 0u) || (tp_ring_buffer_count(&u->tx) != 0u)) {
        return 0;
    }
    /* The DMA has drained, but the shift register may not have: TC is the only
     * flag that means "the last stop bit has physically left the pin", and it is
     * polled rather than cleared so that the DR-write clearing sequence (which
     * would transmit a spurious byte) is never needed. */
    return ((u->regs->SR & USART_SR_TC) != 0u) ? 1 : 0;
}

tp_status_t tp_platform_can_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    uint32_t mailbox;
    uint32_t tsr;
    uint32_t tdlr = 0u;
    uint32_t tdhr = 0u;
    uint32_t i;

    if (dlc > 8u) {
        return TP_ERR_PARAM;
    }
    if ((data == NULL) && (dlc > 0u)) {
        return TP_ERR_PARAM;
    }
    if (id > 0x7FFu) {
        /* 29-bit extended identifiers are not part of the design; rejecting here
         * rather than silently masking keeps a wiring or configuration bug
         * visible, exactly as the host backend does. */
        return TP_ERR_RANGE;
    }
    if (s_can.initialised == 0u) {
        return TP_ERR_NOT_READY;
    }

    /* Pick a free mailbox.  TSR.TME[2:0] has one bit per mailbox, so the first
     * zero bit gives the lowest-numbered free one.  Checking all three here is
     * what lets the caller be told TP_ERR_OVERFLOW instead of silently losing a
     * frame: the contract says the caller must retry. */
    tsr = CAN1->TSR;
    if ((tsr & CAN_TSR_TME0) != 0u) {
        mailbox = 0u;
    } else if ((tsr & CAN_TSR_TME1) != 0u) {
        mailbox = 1u;
    } else if ((tsr & CAN_TSR_TME2) != 0u) {
        mailbox = 2u;
    } else {
        return TP_ERR_OVERFLOW;
    }

    for (i = 0u; i < (uint32_t)dlc; ++i) {
        if (i < 4u) {
            tdlr |= ((uint32_t)data[i]) << (8u * i);
        } else {
            tdhr |= ((uint32_t)data[i]) << (8u * (i - 4u));
        }
    }

    /* IDE = 0 (standard), RTR = 0 (data frame), TXRQ written last so the
     * hardware cannot start transmitting a half-written mailbox. */
    CAN1->sTxMailBox[mailbox].TDTR = (uint32_t)dlc;
    CAN1->sTxMailBox[mailbox].TDLR = tdlr;
    CAN1->sTxMailBox[mailbox].TDHR = tdhr;
    CAN1->sTxMailBox[mailbox].TIR = ((id << CAN_TI0R_STID_Pos) & CAN_TI0R_STID);
    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

    return TP_OK;
}

tp_status_t tp_platform_can_recv(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    tp_stm32_can_frame_t frame;
    uint32_t i;
    uint32_t key;

    if ((id == NULL) || (data == NULL) || (dlc == NULL)) {
        return TP_ERR_PARAM;
    }
    if (s_can.initialised == 0u) {
        return TP_ERR_NOT_READY;
    }

    /* The hardware FIFO is drained into a software FIFO by the RX0 interrupt, so
     * the answer here is about the software FIFO.  Draining in the ISR is what
     * keeps a 3-deep hardware FIFO from overflowing between communication task
     * runs at 100 Hz. */
    if (s_can.rx_head == s_can.rx_tail) {
        /* Belt and braces for the case where the interrupt was masked: take
         * anything the hardware holds right now.  The critical section keeps the
         * software FIFO strictly single-producer, which is what makes the
         * head/tail pair safe without atomics. */
        key = tp_platform_enter_critical();
        (void)can_rx_pop_hardware();
        tp_platform_exit_critical(key);

        if (s_can.rx_head == s_can.rx_tail) {
            return TP_ERR_NOT_READY;
        }
    }

    frame = s_can.rx[s_can.rx_tail & (TP_STM32_CAN_RX_FRAMES - 1u)];
    s_can.rx_tail++;

    *id = frame.id;
    *dlc = frame.dlc;
    for (i = 0u; i < 8u; ++i) {
        data[i] = (i < (uint32_t)frame.dlc) ? frame.data[i] : 0u;
    }
    return TP_OK;
}

tp_status_t tp_platform_watchdog_init(uint32_t timeout_ms)
{
    uint32_t pr;
    uint32_t rlr;

    if (timeout_ms == 0u) {
        return TP_ERR_PARAM;
    }

    /* Turn the LSI on explicitly instead of relying on the IWDG start sequence
     * to do it: the prescaler/reload handshake below is only accepted once the
     * LSI is actually running, and waiting for it here makes that deterministic. */
    RCC->CSR |= RCC_CSR_LSION;
    {
        uint32_t spin = 0u;
        while (((RCC->CSR & RCC_CSR_LSIRDY) == 0u) && (spin < 100000u)) {
            spin++;
        }
        if ((RCC->CSR & RCC_CSR_LSIRDY) == 0u) {
            return TP_ERR_IO;
        }
    }

    iwdg_solve(timeout_ms, &pr, &rlr);

    /* 0xCCCC starts the counter (and forces the LSI on, which is already done);
     * 0x5555 unlocks PR and RLR for writing. */
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = pr & IWDG_PR_PR;
    IWDG->RLR = rlr & IWDG_RLR_RL;

    /* Wait for the prescaler/reload update to be transferred to the counter
     * domain; writing again before PVU/RVU clear is ignored by the hardware. */
    {
        uint32_t spin = 0u;
        while ((IWDG->SR != 0u) && (spin < 100000u)) {
            spin++;
        }
        if (IWDG->SR != 0u) {
            return TP_ERR_TIMEOUT;
        }
    }

    IWDG->KR = 0xAAAAu; /* Reload, so the full timeout starts from now. */

    s_iwdg_armed = 1u;
    s_iwdg_timeout_ms = timeout_ms;
    s_iwdg_div = 4u << (pr & 7u);
    s_iwdg_reload = rlr & IWDG_RLR_RL;

    /* The nominal answer and the honest uncertainty band, as derived in
     * iwdg_solve()'s comment. */
    LOG_INFO(TP_MOD_SAFETY,
             "iwdg armed: PR=/%lu RLR=%lu -> %lu ms nominal at LSI %lu Hz (250..1000 ms over a +-50%% LSI band)",
             (unsigned long)s_iwdg_div, (unsigned long)s_iwdg_reload,
             (unsigned long)((s_iwdg_div * (s_iwdg_reload + 1u) * 1000u)
                             / TP_STM32_LSI_HZ),
             (unsigned long)TP_STM32_LSI_HZ);
    return TP_OK;
}

void tp_platform_watchdog_feed(void)
{
    /* Exactly one store, and nothing else.  No counter, no timestamp, no
     * conditional: anything more here would be state that a hung caller could
     * still be updating, and the whole point of this function is that the only
     * thing it proves is that its caller ran. */
    IWDG->KR = 0xAAAAu;
}

int tp_platform_watchdog_caused_reset(void)
{
    int caused = ((s_reset_flags & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0u)
                     ? 1 : 0;
    uint32_t key = tp_platform_enter_critical();

    s_reset_flags = 0u;   /* Report once, as the hardware flag would. */
    RCC->CSR |= RCC_CSR_RMVF; /* Clear every reset flag (write 1 to clear). */
    tp_platform_exit_critical(key);

    return caused;
}

uint32_t tp_platform_enter_critical(void)
{
    uint32_t key = ((uint32_t)__get_PRIMASK()) & 0x1u;

    __disable_irq();
    s_critical_depth++;
    return key;
}

void tp_platform_exit_critical(uint32_t key)
{
    if (s_critical_depth > 0u) {
        s_critical_depth--;
    }
    /* Restore the *previous* state rather than unconditionally enabling: a
     * critical section nested inside an interrupt-disabled region must not
     * re-enable interrupts on the way out. */
    __set_PRIMASK(key & 0x1u);
}

void tp_platform_emergency_kill(void)
{
    uint32_t key = tp_platform_enter_critical();
    int      first_time = (s_killed == 0u) ? 1 : 0;

    s_killed = 1u;

    /* Zero the duties first: a gate driver that is still switching while its
     * enable pin drops can put a transient on the motor. */
    s_pwm[0] = 0.0f;
    s_pwm[1] = 0.0f;
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&s_htim1, TIM_CHANNEL_2, 0u);

    s_enabled[0] = 0u;
    s_enabled[1] = 0u;
    motor_enable_apply(0);
    motor_enable_apply(1);

    tp_platform_exit_critical(key);

    /* Idempotent: the log is emitted once, on the transition, and only from a
     * thread context.  Logging from handler mode would touch the rate limiter's
     * state concurrently with the debug task, which the logger does not support;
     * a fault handler that needs to say something uses
     * tp_stm32_debug_write_blocking() instead. */
    if ((first_time != 0) && (__get_IPSR() == 0u)) {
        LOG_ERROR(TP_MOD_SAFETY,
                  "EMERGENCY KILL latched, both drives disabled and duties zeroed");
    }
}

void tp_platform_emergency_release(void)
{
    uint32_t key;
    int      was_killed = tp_platform_is_killed();

    key = tp_platform_enter_critical();
    s_killed = 0u;
    s_enabled[0] = 1u;
    s_enabled[1] = 1u;
    motor_enable_apply(0);
    motor_enable_apply(1);
    tp_platform_exit_critical(key);

    if ((was_killed != 0) && (__get_IPSR() == 0u)) {
        LOG_WARN(TP_MOD_SAFETY,
                 "emergency kill released by RESET, drives re-enabled at zero duty");
    }
}

int tp_platform_is_killed(void)
{
    return (s_killed != 0u) ? 1 : 0;
}

/* =========================================================================
 * Target-only helpers
 * ========================================================================= */

uint32_t tp_stm32_iwdg_timeout_ms(void)
{
    return s_iwdg_timeout_ms;
}

uint32_t tp_stm32_iwdg_prescaler_div(void)
{
    return s_iwdg_div;
}

uint32_t tp_stm32_iwdg_reload(void)
{
    return s_iwdg_reload;
}

int tp_stm32_iwdg_armed(void)
{
    return (s_iwdg_armed != 0u) ? 1 : 0;
}

uint32_t tp_stm32_reset_flags(void)
{
    return s_reset_flags;
}

uint32_t tp_stm32_can_btr(void)
{
    return s_can.btr;
}

uint32_t tp_stm32_can_bitrate_bps(void)
{
    return s_can.bitrate;
}

uint32_t tp_stm32_can_sample_point_tenths(void)
{
    return s_can.sample_tenths;
}

int tp_stm32_can_bus_off(void)
{
    if (s_can.initialised == 0u) {
        return 0;
    }
    if ((CAN1->ESR & CAN_ESR_BOFF) != 0u) {
        s_can.bus_off = 1u;
    }
    return (s_can.bus_off != 0u) ? 1 : 0;
}

uint32_t tp_stm32_can_tec(void)
{
    if (s_can.initialised == 0u) {
        return 0u;
    }
    return (CAN1->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
}

uint32_t tp_stm32_can_rec(void)
{
    if (s_can.initialised == 0u) {
        return 0u;
    }
    return (CAN1->ESR & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
}

uint32_t tp_stm32_can_rx_dropped(void)
{
    return s_can.rx_dropped;
}

uint32_t tp_stm32_uart_errors(tp_bus_t bus)
{
    uint32_t idx = bus_to_uart(bus);

    if (idx == TP_STM32_BUS_NONE) {
        return 0u;
    }
    return s_uart[idx].errors;
}

uint32_t tp_stm32_bus_rx_dropped(tp_bus_t bus)
{
    uint32_t idx = bus_to_uart(bus);

    if (idx == TP_STM32_BUS_NONE) {
        return 0u;
    }
    return tp_ring_buffer_overflow_count(&s_uart[idx].rx);
}

int32_t tp_stm32_uart_baud_error_ppm(tp_bus_t bus)
{
    uint32_t idx = bus_to_uart(bus);
    uint32_t actual;
    int64_t  error;

    if (idx == TP_STM32_BUS_NONE) {
        return 0;
    }
    actual = tp_stm32_uart_actual_baud(bus);
    if (s_uart[idx].baud == 0u) {
        return 0;
    }
    error = ((int64_t)actual - (int64_t)s_uart[idx].baud) * 1000000ll
            / (int64_t)s_uart[idx].baud;
    return (int32_t)error;
}

uint32_t tp_stm32_uart_actual_baud(tp_bus_t bus)
{
    uint32_t idx = bus_to_uart(bus);

    if (idx == TP_STM32_BUS_NONE) {
        return 0u;
    }
    if ((s_uart[idx].in_use == 0u) || (s_uart[idx].brr == 0u)) {
        return 0u;
    }
    /* baud = f_CK / BRR (see the derivation in uart_init). */
    return s_uart[idx].pclk_hz / s_uart[idx].brr;
}

uint32_t tp_stm32_critical_depth(void)
{
    return s_critical_depth;
}

int tp_stm32_dwt_available(void)
{
    return (s_dwt_ok != 0u) ? 1 : 0;
}

void tp_stm32_tim7_irq_handler(void)
{
    if ((TIM7->SR & TIM_SR_UIF) != 0u) {
        TIM7->SR = (uint32_t)(~TIM_SR_UIF); /* Update flag is rc_w0. */
        TIM7->CR1 &= ~TIM_CR1_CEN;
        TIM7->DIER &= ~TIM_DIER_UIE;

        /* Only release the pair if the transmitter really is finished.  If a
         * frame is still being shifted out (or another chunk has just been
         * queued), re-arm instead: releasing DE early would truncate the last
         * byte, which the far end reports as a CRC error on an almost-complete
         * frame, i.e. the most confusing possible failure. */
        if ((USART3->SR & USART_SR_TC) != 0u) {
            rs485_de_write(0);
            s_uart[TP_STM32_UART_RS485].de_asserted = 0u;
        } else {
            rs485_de_arm_release();
        }
    }
}

void tp_stm32_debug_write_blocking(const char *data, uint32_t len)
{
    uint32_t i;

    if (data == NULL) {
        return;
    }
    for (i = 0u; i < len; ++i) {
        uint32_t spin = 0u;

        /* Wait for the transmit register, bounded so a dead USART cannot turn a
         * fault dump into a second, permanent hang. */
        while (((USART1->SR & USART_SR_TXE) == 0u) && (spin < 1000000u)) {
            spin++;
        }
        USART1->DR = (uint32_t)(uint8_t)data[i];
    }
    {
        uint32_t spin = 0u;
        while (((USART1->SR & USART_SR_TC) == 0u) && (spin < 1000000u)) {
            spin++;
        }
    }
}

void tp_stm32_debug_puts_blocking(const char *s)
{
    uint32_t len = 0u;

    if (s == NULL) {
        return;
    }
    while (s[len] != '\0') {
        len++;
    }
    tp_stm32_debug_write_blocking(s, len);
}

void tp_stm32_debug_put_hex_blocking(const char *label, uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[8];
    int  i;

    if (label != NULL) {
        tp_stm32_debug_puts_blocking(label);
    }
    for (i = 7; i >= 0; --i) {
        text[i] = hex[value & 0xFu];
        value >>= 4;
    }
    tp_stm32_debug_write_blocking(text, 8u);
}

void tp_stm32_fault_report(uint32_t exception, const uint32_t *frame)
{
    static const char *const names[] = {
        "Thread", "Reserved", "NMI", "HardFault",
        "MemManage", "BusFault", "UsageFault", "Reserved"
    };
    const char *name = (exception < 8u) ? names[exception] : "Unknown";

    tp_stm32_debug_puts_blocking("\r\n*** FAULT: ");
    tp_stm32_debug_puts_blocking(name);
    tp_stm32_debug_puts_blocking(" ***\r\n");

    if (frame != NULL) {
        tp_stm32_debug_put_hex_blocking("R0  = 0x", frame[0]);
        tp_stm32_debug_put_hex_blocking(" R1  = 0x", frame[1]);
        tp_stm32_debug_puts_blocking("\r\n");
        tp_stm32_debug_put_hex_blocking("R2  = 0x", frame[2]);
        tp_stm32_debug_put_hex_blocking(" R3  = 0x", frame[3]);
        tp_stm32_debug_puts_blocking("\r\n");
        tp_stm32_debug_put_hex_blocking("R12 = 0x", frame[4]);
        tp_stm32_debug_put_hex_blocking(" LR  = 0x", frame[5]);
        tp_stm32_debug_puts_blocking("\r\n");
        tp_stm32_debug_put_hex_blocking("PC  = 0x", frame[6]);
        tp_stm32_debug_put_hex_blocking(" PSR = 0x", frame[7]);
        tp_stm32_debug_puts_blocking("\r\n");
    }

    tp_stm32_debug_put_hex_blocking("CFSR= 0x", SCB->CFSR);
    tp_stm32_debug_put_hex_blocking(" HFSR= 0x", SCB->HFSR);
    tp_stm32_debug_puts_blocking("\r\n");
    tp_stm32_debug_put_hex_blocking("MMFAR=0x", SCB->MMFAR);
    tp_stm32_debug_put_hex_blocking(" BFAR= 0x", SCB->BFAR);
    tp_stm32_debug_puts_blocking("\r\n");
}

/* =========================================================================
 * HAL hooks owned by this file
 * ========================================================================= */

/**
 * @brief HAL parameter-assertion trap.
 *
 * Compiled only when `USE_FULL_ASSERT` is defined (see
 * `firmware/Core/stm32f4xx_hal_conf.h`, which has to supply `assert_param()`
 * because the vendored HAL tree does not).  The file name is printed verbatim
 * because the whole point of enabling this on the bench is to find out *which*
 * call site had a bad argument; the line number follows it.
 *
 * It resets rather than returning: a HAL argument assertion that fires on a live
 * robot means a caller is passing something the driver cannot interpret, and
 * continuing would drive the motors from an undefined configuration.
 *
 * @param[in] file  `__FILE__` of the failing assertion.
 * @param[in] line  `__LINE__` of the failing assertion.
 */
#if defined(USE_FULL_ASSERT)
void assert_failed(uint8_t *file, uint32_t line)
{
    tp_stm32_debug_puts_blocking("\r\n*** HAL assert_param failed: ");
    if (file != NULL) {
        tp_stm32_debug_puts_blocking((const char *)file);
    }
    tp_stm32_debug_put_hex_blocking(":", line);
    tp_stm32_debug_puts_blocking("\r\n");

    tp_platform_emergency_kill();

    __disable_irq();
    NVIC_SystemReset();
    for (;;) {
    }
}
#endif /* USE_FULL_ASSERT */

/**
 * @brief HAL MSP bring-up.
 *
 * Called by `HAL_Init()`.  Everything that would normally live here (GPIO
 * clocks, pin muxing, DMA clock, NVIC priorities) is done explicitly in
 * `tp_platform_init()` so that the whole hardware configuration of this board is
 * readable in one place.  This hook exists, and is deliberately empty, so that
 * nobody later adds a second, hidden configuration path through the HAL's MSP
 * callbacks.
 */
void HAL_MspInit(void)
{
    /* Enable the power interface clock: the ADC and the voltage regulator
     * configuration both live behind it, and enabling it here matches what the
     * HAL expects after HAL_Init(). */
    __HAL_RCC_PWR_CLK_ENABLE();
}

#endif /* TP_PLATFORM_STM32F407 */
