/**
 * @file    config.h
 * @brief   Central compile-time configuration for the tea-picking robot firmware.
 *
 * @details
 * Every tunable number of the system lives here so that no "magic number" is
 * scattered across the source tree.  The file is organised as:
 *
 *   1. Build / platform selection
 *   2. Task rates and periods
 *   3. Physical and actuator limits
 *   4. Control (PID) default gains
 *   5. Force-feedback / flexible-gripping parameters
 *   6. Communication parameters
 *   7. Safety thresholds
 *   8. Sensor signal-conditioning parameters
 *   9. Debug parameters
 *
 * @note All values are expressed in the *internal* engineering units of the
 *       firmware.  See @ref docs/units_convention.md for the full table:
 *
 *       | Quantity            | Internal unit | Wire unit              |
 *       |---------------------|---------------|------------------------|
 *       | Joint angle         | mrad (float)  | mrad (int32)           |
 *       | Joint velocity      | mrad/s (float)| mrad/s (int16)         |
 *       | Gripper jaw gap     | mm (float)    | um (uint16)            |
 *       | Force               | N (float)     | mN (int16)             |
 *       | Motor current       | mA (float)    | mA (int16)             |
 *       | PWM duty            | per-mille     | per-mille (int16)      |
 *       | Cartesian target    | mm (float)    | mm (int16)             |
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_CONFIG_H
#define TP_CONFIG_H

/* ==========================================================================
 * 1. Build / platform selection
 * ========================================================================== */

/** @brief Firmware semantic version (MAJOR.MINOR.PATCH). */
#define TP_FW_VERSION_MAJOR 2
#define TP_FW_VERSION_MINOR 0
#define TP_FW_VERSION_PATCH 0

/**
 * @brief Target platform selector.
 *
 * Exactly one of these is defined by the build system
 * (`-DTP_PLATFORM_HOST` or `-DTP_PLATFORM_STM32F407`).  If the build system
 * defines none, host is assumed so that a plain `gcc foo.c` still compiles.
 */
#if !defined(TP_PLATFORM_HOST) && !defined(TP_PLATFORM_STM32F407)
#define TP_PLATFORM_HOST 1
#endif

/** @brief When 1, the host build injects a deterministic plant model. */
#if defined(TP_PLATFORM_HOST)
#define TP_SIMULATION_MODE 1
#else
#define TP_SIMULATION_MODE 0
#endif

/* ==========================================================================
 * 2. Task rates and periods
 * ========================================================================== */

/** @name Task rates (Hz)
 *  The control loop is the time base; every other rate must divide it evenly
 *  so that the slow tasks are harmonic with respect to the fast one.  A
 *  compile-time static assertion below enforces this.
 *  @{ */
#define TP_CONTROL_FREQ_HZ   1000u /**< Cascade PID + motor PWM update.      */
#define TP_SENSOR_FREQ_HZ     500u /**< ADC/DMA, force, encoder sampling.    */
#define TP_COMM_FREQ_HZ       100u /**< UART / CAN / RS485 service.          */
#define TP_ROBOT_FREQ_HZ      100u /**< Robot FSM step.                      */
#define TP_SAFETY_FREQ_HZ     100u /**< Fault detection.                     */
#define TP_TELEMETRY_FREQ_HZ   20u /**< Periodic data upload.                */
#define TP_DEBUG_FREQ_HZ        5u /**< Log flush / console.                 */
/** @} */

/** @brief Convert a frequency in Hz to a period in milliseconds. */
#define TP_MS_PER_HZ(hz) (1000u / (hz))

#define TP_CONTROL_PERIOD_MS   TP_MS_PER_HZ(TP_CONTROL_FREQ_HZ)
#define TP_SENSOR_PERIOD_MS    TP_MS_PER_HZ(TP_SENSOR_FREQ_HZ)
#define TP_COMM_PERIOD_MS      TP_MS_PER_HZ(TP_COMM_FREQ_HZ)
#define TP_ROBOT_PERIOD_MS     TP_MS_PER_HZ(TP_ROBOT_FREQ_HZ)
#define TP_SAFETY_PERIOD_MS    TP_MS_PER_HZ(TP_SAFETY_FREQ_HZ)
#define TP_TELEMETRY_PERIOD_MS TP_MS_PER_HZ(TP_TELEMETRY_FREQ_HZ)
#define TP_DEBUG_PERIOD_MS     TP_MS_PER_HZ(TP_DEBUG_FREQ_HZ)

/** @brief Control loop sample time in seconds (float, used by the PID). */
#define TP_CONTROL_DT_S (1.0f / (float)TP_CONTROL_FREQ_HZ)

/** @brief Sensor task sample time in seconds. */
#define TP_SENSOR_DT_S (1.0f / (float)TP_SENSOR_FREQ_HZ)

/** @brief Robot (FSM) task sample time in seconds. */
#define TP_ROBOT_DT_S (1.0f / (float)TP_ROBOT_FREQ_HZ)

/** @brief Telemetry task sample time in seconds. */
#define TP_TELEMETRY_DT_S (1.0f / (float)TP_TELEMETRY_FREQ_HZ)

/** @name FreeRTOS task stack depths, in 32-bit words (not bytes). @{ */
#define TP_STACK_CONTROL   512u
#define TP_STACK_SENSOR    384u
#define TP_STACK_COMM      512u
#define TP_STACK_ROBOT     512u
#define TP_STACK_SAFETY    384u
#define TP_STACK_TELEMETRY 512u
#define TP_STACK_DEBUG     384u
/** @} */

/** @name FreeRTOS task priorities (higher number = higher priority). @{ */
#define TP_PRIO_CONTROL   6u /**< Hard real-time: must never miss a period.   */
#define TP_PRIO_SAFETY    5u /**< Fault reaction, just below control.         */
#define TP_PRIO_SENSOR    4u /**< Feeds the control loop.                     */
#define TP_PRIO_ROBOT     3u /**< FSM, soft real-time.                        */
#define TP_PRIO_COMM      2u /**< UART/CAN/RS485, jitter tolerant.            */
#define TP_PRIO_TELEMETRY 1u /**< Uploads, best effort.                       */
#define TP_PRIO_DEBUG     0u /**< Logging, strictly lowest.                   */
/** @} */

/** @name Queue depths. @{ */
#define TP_QCMD_DEPTH     8u /**< Down-link command queue.                    */
#define TP_QTELEM_DEPTH   16u/**< Up-link telemetry queue.                    */
#define TP_QEVENT_DEPTH   16u/**< FSM event queue.                            */
#define TP_QFAULT_DEPTH   8u /**< Fault-notification queue.                   */
/** @} */

/* ==========================================================================
 * 3. Physical and actuator limits
 * ========================================================================== */

/** @name Joint (arm) limits. @{ */
#define TP_JOINT_MIN_MRAD     (-90000.0f) /**< -90.000 deg soft limit.         */
#define TP_JOINT_MAX_MRAD     ( 90000.0f) /**< +90.000 deg soft limit.         */
#define TP_JOINT_HOME_MRAD    (       0.0f)
#define TP_JOINT_MAX_SPEED_MRAD_S ( 60000.0f) /**< 60 deg/s.                  */
#define TP_JOINT_MAX_ACCEL_MRAD_S2 (200000.0f) /**< Implied by the profile.  *//** @} */

/** @name Gripper (linear jaw) limits. @{ */
#define TP_GRIPPER_MIN_MM      (  0.0f)  /**< Fully closed.                   */
#define TP_GRIPPER_MAX_MM      ( 40.0f)  /**< Fully open.                     */
#define TP_GRIPPER_MAX_SPEED_MM_S (120.0f)
/** @} */

/** @name Electrical / drive limits. @{ */
#define TP_MAX_PWM_PERMILLE    (1000.0f) /**< |PWM| clamp, 1000 = 100 %.      */
#define TP_MAX_CURRENT_MA      (3000.0f) /**< Continuous motor current limit. */
#define TP_PEAK_CURRENT_MA     (5000.0f) /**< Short-time peak, < 50 ms.       */
/**
 * @brief Duty slew limit, per-mille per second.
 *
 * The purpose is to stop the controller from stepping the applied voltage from
 * one extreme to the other inside a single 1 ms sample, which produces a
 * current spike limited only by the winding resistance.  The value is
 * **200 000 per-mille/s**, i.e. full scale in 5 ms.
 *
 * This number was got wrong in the first revision and the mistake is worth
 * recording: the initial value was 4 000 per-mille/s, which takes 250 ms to
 * sweep the full range.  That is not a slew limiter, it is a dominant lag
 * inside the velocity loop — the loop had roughly 60 degrees of unmodelled
 * phase lag at its crossover frequency and oscillated with an amplitude of
 * tens of thousands of milliradians per second.  The lesson is that any
 * actuator-rate limit has to be checked against the loop bandwidth it sits
 * inside: full scale must be reached in a time much shorter than 1/(2*pi*fc).
 * At an 8 Hz velocity loop that budget is ~20 ms, so 5 ms is comfortable and
 * still ten times slower than the electrical time constant it protects.
 * @{ */
#define TP_MAX_DUTY_RATE_PER_S (200000.0f)
/** @} */

/* ==========================================================================
 * 4. Control (PID) default gains
 * ========================================================================== */

/**
 * @name Position loop (outer) default gains.
 * Input  : position error in mrad.
 * Output : velocity reference in mrad/s.
 *
 * **Tuning basis.**  The plant seen by this loop is the *closed* velocity loop,
 * which behaves as a first-order lag of about 20 ms.  Commanding
 * `v = Kp * e` therefore closes the position loop with a time constant of
 * `1 / Kp`; `Kp = 5.0` gives 200 ms, i.e. a bandwidth of about 0.8 Hz, which is
 * a **10x separation** from the 8 Hz inner loop.  That ratio is the design
 * constraint (see `docs/05_control_algorithm.md`), not a coincidence: below
 * roughly 5x the cascade loses its advantage over a single loop, and above
 * roughly 20x the position loop becomes unnecessarily slow.
 *
 * The integral clamp is deliberately an order of magnitude below the output
 * clamp.  It bounds how much velocity the I term can demand *on its own*, which
 * is the single most effective anti-overshoot measure in the controller: a long
 * approach with a standing error cannot charge the integrator and then release
 * it as an overshoot at the end of the move.
 * @{ */
#define TP_POS_PID_KP        (  5.0f)
#define TP_POS_PID_KI        (  0.5f)
#define TP_POS_PID_KD        (  0.0f)
#define TP_POS_PID_FF_GAIN   (  1.0f)  /**< Velocity feed-forward gain.      */
#define TP_POS_PID_IMAX      ( 2000.0f)/**< Integral clamp, mrad/s.          */
#define TP_POS_PID_OUT_MAX   ( 60000.0f)/**< = TP_JOINT_MAX_SPEED_MRAD_S.    */
#define TP_POS_PID_OUT_MIN   (-60000.0f)
#define TP_POS_PID_TAU_D_S   ( 0.002f) /**< Derivative low-pass time const.  */
/** @} */

/**
 * @name Velocity loop (inner) default gains.
 * Input  : velocity error in mrad/s.
 * Output : PWM duty in per-mille.
 *
 * **Tuning basis.**  The duty-to-speed path is, from the electrical and
 * mechanical parameters in `plant_model.c`, a first-order lag with a DC gain of
 * about 480 mrad/s per per-mille and a mechanical time constant of
 * `J / (b + Kt*Ke/R) = 0.05 s`.  The standard PI placement
 * `Kp = tau / (K * Tc)` with a desired closed-loop time constant `Tc = 20 ms`
 * gives `Kp = 0.05 / (480 * 0.02) = 0.0052`, and `Ki = Kp / tau = 0.104`.  The
 * shipped values are those rounded to one significant figure.
 *
 * The first revision used `Kp = 0.015`, three times higher, which put the
 * crossover at roughly 23 Hz and left too little phase margin once the
 * electrical lag and the duty slew limiter were accounted for.  The resulting
 * limit cycle is what the initial simulation run showed.
 * @{ */
#define TP_VEL_PID_KP        ( 0.005f)
#define TP_VEL_PID_KI        ( 0.10f)
#define TP_VEL_PID_KD        (  0.0f)
#define TP_VEL_PID_FF_GAIN   (  0.0f)  /**< Torque feed-forward gain.        */
#define TP_VEL_PID_IMAX      (250.0f)  /**< Integral clamp, permille.        */
#define TP_VEL_PID_OUT_MAX   ( 1000.0f)
#define TP_VEL_PID_OUT_MIN   (-1000.0f)
#define TP_VEL_PID_TAU_D_S   ( 0.001f)
/** @} */

/**
 * @name Force loop default gains.
 * Input  : force error in N.
 * Output : gripper closing velocity in mm/s.
 * @{ */
#define TP_FORCE_PID_KP      ( 14.0f)
#define TP_FORCE_PID_KI      (  6.0f)
#define TP_FORCE_PID_KD      (  0.05f)
#define TP_FORCE_PID_IMAX    ( 20.0f)  /**< Integral clamp, N*s.             */
#define TP_FORCE_PID_OUT_MAX ( 25.0f)  /**< Max closing speed, mm/s.         */
#define TP_FORCE_PID_OUT_MIN (-25.0f)
#define TP_FORCE_PID_TAU_D_S ( 0.010f)
/** @} */

/** @brief PID anti-windup strategy selector, see @ref pid.h. */
#define TP_PID_ANTIWINDUP_BACK_CALC 1

/* ==========================================================================
 * 5. Force-feedback / flexible gripping parameters
 * ========================================================================== */

/** @name Force targets (N). @{ */
#define TP_FORCE_TARGET_N           (2.20f) /**< Desired grip force.          */
#define TP_FORCE_TARGET_MIN_N       (0.80f) /**< Lower bound of the valid range.*/
#define TP_FORCE_TARGET_MAX_N       (3.00f) /**< Upper bound of the valid range.*/
#define TP_FORCE_CONTACT_N          (0.15f) /**< Contact detected threshold.  */
#define TP_FORCE_SOFT_LIMIT_N       (3.50f) /**< Warning threshold.           */
#define TP_FORCE_MAX_N              (5.00f) /**< Hard limit -> immediate stop.*/
#define TP_FORCE_RELEASE_N          (0.10f) /**< Regarded as "no load".       */
#define TP_FORCE_BAND_N             (0.10f) /**< +-band considered "in place".*/
/** @} */

/** @name Gripper approach profiles. @{ */
#define TP_APPROACH_FAST_SPEED_MM_S ( 40.0f) /**< Coarse approach speed.      */
#define TP_APPROACH_SLOW_SPEED_MM_S (  6.0f) /**< Fine approach speed.        */
#define TP_APPROACH_SLOW_AT_MM      (  4.0f) /**< Switch distance to target.  */
#define TP_GRIPPER_BACKOFF_MM       ( 12.0f) /**< Retreat after release.      */
#define TP_GRIPPER_SETTLE_MS        (  80u)  /**< Force settle time.          */
/** @} */

/** @name Adaptive force threshold. @{ */
#define TP_FORCE_ADAPT_ENABLE       (1)
#define TP_FORCE_ADAPT_ALPHA        (0.05f)/**< EMA rate for the observed force. */
#define TP_FORCE_ADAPT_MARGIN_N     (0.60f) /**< Target = observed + margin.  */
/** @} */

/* ==========================================================================
 * 6. Communication parameters
 * ========================================================================== */

/** @brief Frame protocol magic. Frames always start with 0xAA 0x55. */
#define TP_PROTO_HEADER0      (0xAAu)
#define TP_PROTO_HEADER1      (0x55u)
#define TP_PROTO_MAX_PAYLOAD  (64u)   /**< Maximum payload length, bytes.     */
#define TP_PROTO_OVERHEAD     (7u)    /**< HDR(2)+CMD(1)+LEN(1)+CRC(2)+TAIL?  */
#define TP_PROTO_MAX_FRAME    (TP_PROTO_MAX_PAYLOAD + TP_PROTO_OVERHEAD)

/** @name Link timeouts (ms). @{ */
#define TP_COMM_TIMEOUT_MS       (200u) /**< No valid frame -> link lost.    */
#define TP_COMM_HEARTBEAT_MS     (500u) /**< Host heartbeat period.          */
#define TP_COMM_RETRY_COUNT      (3u)
#define TP_COMM_TX_TIMEOUT_MS    (20u)
/** @} */

/** @name Serial / bus configuration. @{ */
#define TP_UART_DEBUG_BAUD   (115200u)
#define TP_UART_VISION_BAUD  (921600u) /**< Vision host link.                */
#define TP_RS485_BAUD        (115200u) /**< Modbus RTU bus.                  */
#define TP_CAN_BITRATE_BPS   (500000u) /**< 500 kbit/s.                      */
#define TP_MODBUS_SLAVE_ADDR (0x11u)   /**< This node on the RS485 bus.      */
#define TP_MODBUS_BAUD       (TP_RS485_BAUD)
/** @} */

/* ==========================================================================
 * 7. Safety thresholds
 * ========================================================================== */

/** @brief Watchdog reload period, ms. Must exceed the slowest task period. */
#define TP_WDG_TIMEOUT_MS        (500u)
/** @brief Number of consecutive control periods above current limit. */
#define TP_OVERCURRENT_TRIP_MS   (50u)
/** @brief Encoder is declared stalled if |dq| stays below this while driving. */
#define TP_STALL_SPEED_THRESHOLD_MRAD_S (30.0f)
/** @brief Minimum PWM demand for stall detection to be armed, permille.
 *
 * This value has to be read together with the velocity-loop gains: the detector
 * is armed only once the controller is genuinely *asking* for torque, and how
 * long that takes depends on how fast the loop's integral builds.  With
 * `TP_VEL_PID_KI = 0.10` and a large position error (a long move into a jam),
 * a demand of 80 per-mille is reached in roughly 100 ms, so the debounce below
 * then dominates the detection latency.  A higher threshold would still be
 * correct but would make the detector slow enough to be useless on a real jam;
 * a much lower one would arm on the noise floor and trip on every move.
 *
 * 80 per-mille is 8 % duty, i.e. about 1.9 V across the winding — clearly a
 * deliberate torque command rather than a residual. */
#define TP_STALL_PWM_THRESHOLD   (80.0f)
/** @brief Time the above conditions must hold before ERR_MOTOR_STALL. */
#define TP_STALL_TRIP_MS         (300u)
/** @brief Maximum |dq/dt| physically plausible, mrad/s^2. Above => encoder fault. */
#define TP_ENCODER_MAX_ACCEL_MRAD_S2 (4000000.0f)
/** @brief Force-sensor plausibility window, N. Outside => ERR_SENSOR. */
#define TP_SENSOR_FORCE_MIN_N    (-20.0f)
#define TP_SENSOR_FORCE_MAX_N    ( 50.0f)
/** @brief ADC raw plausibility window, counts (12-bit). */
#define TP_SENSOR_ADC_MIN_COUNTS (5u)
#define TP_SENSOR_ADC_MAX_COUNTS (4090u)
/** @brief Number of consecutive bad samples before the sensor is declared faulty. */
#define TP_SENSOR_FAULT_DEBOUNCE (10u)

/**
 * @brief Sentinel reported in the `temp_c_x10` field when no sensor is fitted.
 *
 * The reference design has no board-temperature sensor, so the field reports
 * this value rather than a plausible-looking zero.  A host tool that renders
 * "0.0 C" would be actively misleading; "3276.7 C" is obviously a sentinel.
 */
#define TP_SENSOR_TEMP_UNAVAILABLE (32767)

/* ==========================================================================
 * 8. Sensor signal-conditioning parameters
 * ========================================================================== */

/** @name ADC. @{ */
#define TP_ADC_RESOLUTION_BITS  (12u)
#define TP_ADC_FULL_SCALE       (4095.0f) /**< 2^12 - 1.                    */
#define TP_ADC_VREF_V           (3.300f)
#define TP_ADC_DMA_BUFFER_LEN   (16u)  /**< Circular DMA half/full buffers.  */
/** @} */

/**
 * @name Force-sensor analogue front end (see hardware/force_sensor.md).
 *
 * The chain is:
 * @verbatim
 *   10 N load cell (1 mV/V, 5 V excitation)
 *     -> INA333 instrumentation amp, gain 201 (RG = 249 ohm)
 *     -> RC anti-alias filter, fc = 1.6 kHz
 *     -> STM32 ADC1_IN0, 12 bit, Vref = 3.3 V
 * @endverbatim
 *
 * **Why a 10 N cell and not a 50 N one.**  The measurement requirement drives
 * the choice: the target grip force is 2.2 N and the hard limit is 5 N, so a
 * useful reading needs a resolution of well under 20 mN.  One ADC step is
 * `3.3 V / 4096 = 0.806 mV`.  Solving `0.806 mV / LSB < 20 mN` requires a
 * front-end sensitivity above `40 mV/N`.  A 10 N cell at 1 mV/V with 5 V
 * excitation gives `5 mV / 10 N = 0.5 mV/N`, so a gain of 201 is enough
 * (`100.5 mV/N`, i.e. an **8.0 mN LSB**), while a 50 N cell would only give
 * `0.1 mV/N` and would need a gain of 400 — at which point the amplifier's own
 * offset drift and the bridge's thermal drift dominate the reading.  Choosing
 * the sensor so the electronics stay simple is the better engineering answer
 * than amplifying a small signal by 400.
 *
 * Check the headroom: `100.5 mV/N * 10 N = 1005 mV` at rated load, comfortably
 * under the 3.3 V reference, and the ADC reaches full scale at 32.8 N — far
 * above the 5 N hard limit, so overload is detected by the firmware rather
 * than clipping silently against the rail.
 *
 * The numbers below are a **reference design** derived from datasheet figures,
 * not measurements taken on a built board.  Everything downstream (LSB size,
 * usable range) is computed from them, so changing one constant here
 * propagates consistently instead of leaving a stale magic number in the
 * driver.  Two static assertions at the bottom of this file fail the build if
 * the design is over-ranged or under-resolved.
 * @{ */
#define TP_LOADCELL_SENSITIVITY_MV_V (1.0f)  /**< Rated output, mV per volt.   */
#define TP_LOADCELL_RATED_N          (10.0f) /**< Rated capacity, N.           */
#define TP_LOADCELL_EXCITATION_V     (5.0f)  /**< Bridge excitation, V.        */
#define TP_FORCE_AMP_GAIN_STAGE1     (201.0f)/**< INA333 gain, RG = 249 ohm.   */
#define TP_FORCE_AMP_GAIN_STAGE2     (1.0f)  /**< Unity buffer; kept so a second
                                                  stage can be added without
                                                  touching any other constant. */
/** @brief Total front-end gain. */
#define TP_FORCE_AMP_GAIN (TP_FORCE_AMP_GAIN_STAGE1 * TP_FORCE_AMP_GAIN_STAGE2)
/** @brief Load-cell bridge output per newton, mV/N. */
#define TP_LOADCELL_MV_PER_N \
    ((TP_LOADCELL_SENSITIVITY_MV_V * TP_LOADCELL_EXCITATION_V) / TP_LOADCELL_RATED_N)
/** @brief Front-end output per newton, mV/N. */
#define TP_FORCE_FRONTEND_MV_PER_N (TP_LOADCELL_MV_PER_N * TP_FORCE_AMP_GAIN)
/** @brief Size of one ADC step in millivolts. */
#define TP_ADC_MV_PER_COUNT \
    ((TP_ADC_VREF_V * 1000.0f) / (TP_ADC_FULL_SCALE + 1.0f))
/** @brief Bridge output at full rated load, mV (must stay under Vref). */
#define TP_FORCE_FULLSCALE_MV (TP_FORCE_FRONTEND_MV_PER_N * TP_LOADCELL_RATED_N)
/** @brief Force at which the ADC input saturates, N. */
#define TP_FORCE_ADC_SATURATION_N \
    ((TP_ADC_VREF_V * 1000.0f) / TP_FORCE_FRONTEND_MV_PER_N)
/** @} */

/** @name Digital filtering. @{ */
#define TP_FILTER_FORCE_MA_WINDOW   (8u)   /**< Moving-average window.        */
#define TP_FILTER_FORCE_LPF_TAU_S   (0.006f) /**< 1st-order LPF, 26.5 Hz.     */
#define TP_FILTER_SPEED_MA_WINDOW   (5u)   /**< Velocity moving average.      */
#define TP_FILTER_SPEED_LPF_TAU_S   (0.003f)
#define TP_FILTER_MEDIAN_WINDOW     (5u)   /**< Spike rejection window.       */
/** @} */

/** @name Calibration defaults (overwritten by NVM / host at runtime).
 *
 * `TP_FORCE_CAL_SCALE_N_PER_COUNT` is *computed* from the front-end design
 * rather than typed in, so it can never drift out of step with the gain
 * constants above.  With the values in this file it evaluates to
 * `0.8057 mV / 100.5 mV/N` = **0.00802 N per ADC count**, i.e. an 8.0 mN LSB
 * and a usable range of about +-16.4 N about mid-scale.  Compare that with
 * @ref TP_FORCE_TARGET_N = 2.2 N: the target sits at ~274 counts above zero,
 * so one LSB is 0.36 % of the reading.  That is the number that justifies the
 * median + moving-average + low-pass chain in @ref filters.h — without it the
 * force loop would be chasing quantisation noise rather than the tea stem.
 * @{ */
#define TP_FORCE_CAL_ZERO_COUNTS (2048.0f) /**< ADC counts at zero load.      */
#define TP_FORCE_CAL_SCALE_N_PER_COUNT \
    (TP_ADC_MV_PER_COUNT / TP_FORCE_FRONTEND_MV_PER_N)
/** @brief Usable force range about mid-scale, N (information for the docs). */
#define TP_FORCE_USABLE_RANGE_N \
    ((TP_ADC_FULL_SCALE / 2.0f) * TP_FORCE_CAL_SCALE_N_PER_COUNT)
/** @} */

/* ==========================================================================
 * 9. Debug parameters
 * ========================================================================== */

/** @name Log levels, see @ref logger.h. @{ */
#define TP_LOG_LEVEL_DEBUG 0
#define TP_LOG_LEVEL_INFO  1
#define TP_LOG_LEVEL_WARN  2
#define TP_LOG_LEVEL_ERROR 3
#define TP_LOG_LEVEL_NONE  4
/** @} */

/** @brief Compile-time log level filter. */
#ifndef TP_LOG_LEVEL
#define TP_LOG_LEVEL TP_LOG_LEVEL_INFO
#endif

/** @brief Size of the log ring buffer, bytes. */
#define TP_LOG_BUFFER_SIZE   (2048u)

/* ==========================================================================
 * Compile-time consistency checks
 * ========================================================================== */

#include "compiler.h"

TP_STATIC_ASSERT((TP_CONTROL_FREQ_HZ % TP_SENSOR_FREQ_HZ) == 0u,
                 "SENSOR_FREQ must divide CONTROL_FREQ");
TP_STATIC_ASSERT((TP_CONTROL_FREQ_HZ % TP_COMM_FREQ_HZ) == 0u,
                 "COMM_FREQ must divide CONTROL_FREQ");
TP_STATIC_ASSERT((TP_CONTROL_FREQ_HZ % TP_ROBOT_FREQ_HZ) == 0u,
                 "ROBOT_FREQ must divide CONTROL_FREQ");
TP_STATIC_ASSERT((TP_CONTROL_FREQ_HZ % TP_SAFETY_FREQ_HZ) == 0u,
                 "SAFETY_FREQ must divide CONTROL_FREQ");
TP_STATIC_ASSERT((TP_CONTROL_FREQ_HZ % TP_TELEMETRY_FREQ_HZ) == 0u,
                 "TELEMETRY_FREQ must divide CONTROL_FREQ");
TP_STATIC_ASSERT(TP_MS_PER_HZ(TP_CONTROL_FREQ_HZ) >= 1u,
                 "CONTROL_FREQ is too high for a 1 ms tick");
TP_STATIC_ASSERT(TP_FORCE_TARGET_N < TP_FORCE_SOFT_LIMIT_N,
                 "TARGET_FORCE must be below the soft limit");
TP_STATIC_ASSERT(TP_FORCE_SOFT_LIMIT_N < TP_FORCE_MAX_N,
                 "SOFT_LIMIT must be below MAX_FORCE");
TP_STATIC_ASSERT(TP_JOINT_MAX_SPEED_MRAD_S <= TP_POS_PID_OUT_MAX,
                 "Position-loop output clamp must allow the full speed range");
TP_STATIC_ASSERT(TP_PROTO_MAX_FRAME <= 128u,
                 "Frame buffer must stay small enough for the RX ring");
TP_STATIC_ASSERT(TP_FORCE_FULLSCALE_MV < (TP_ADC_VREF_V * 1000.0f),
                 "Front-end gain over-ranges the ADC at rated load");
TP_STATIC_ASSERT(TP_FORCE_USABLE_RANGE_N > TP_FORCE_MAX_N,
                 "Usable force range must cover the hard force limit");

#endif /* TP_CONFIG_H */
