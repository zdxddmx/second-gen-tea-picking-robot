/**
 * @file    gripper.h
 * @brief   Gripper jaw driver: gap measurement, velocity and position motion.
 *
 * @details
 * # What it is
 * The device driver for the jaw axis.  It owns the gripper's motor channel, its
 * linear encoder and its two limit switches, and it exposes exactly three ways
 * to command the jaw:
 *
 *   - @ref tp_gripper_set_velocity_mm_s — "close at 6 mm/s".  This is what the
 *     force controller uses, because force control is fundamentally a velocity
 *     command problem: to raise the force you close a little, to lower it you
 *     back off a little.
 *   - @ref tp_gripper_move_to_mm — "go to a 6 mm gap", a bounded-speed
 *     position move.  Used for homing and for the open/close primitives.
 *   - @ref tp_gripper_open / @ref tp_gripper_close — the two named postures.
 *
 * # Unit convention
 * `gap_mm` is the **distance between the jaws**.  It decreases as the gripper
 * closes, which matches how an operator thinks about it ("how far apart are
 * they?") and keeps the sign of `velocity_mm_s` the same as the sign of the
 * motion: negative velocity means closing.  The motor duty therefore has the
 * opposite sign convention, which is inverted in exactly one place — the
 * @ref gripper_velocity_to_duty helper — so no other file has to remember it.
 *
 * # Why the limit switches are handled here and not in the caller
 * A jaw driven into its mechanical stop stalls the motor and burns the driver.
 * The switch is therefore treated as a *motion veto* inside
 * @ref tp_gripper_update, not as advisory information: if the "open" switch is
 * asserted, any further opening command is zeroed regardless of who issued it.
 * Putting that rule in the driver rather than in each caller means it cannot be
 * forgotten by a new call site.
 *
 * # The veto must not destroy the request
 * This is subtle enough to be worth stating explicitly, because the first
 * revision got it wrong.  The vetoed value is written to
 * @ref tp_gripper_t::applied_velocity_mm_s, **not** back into
 * @ref tp_gripper_t::cmd_velocity_mm_s.  Writing it back looks harmless and is
 * not: the jaw sits on a limit switch, the veto zeroes the velocity, the zero is
 * stored as the new setpoint, and from then on there is nothing left to veto —
 * so when the jaw later moves off the switch, the stored setpoint is still zero
 * and the axis never moves again.
 *
 * In this firmware it produced a concrete failure: `tp_gripper_open()` stalled
 * at about 39.8 mm, a few tenths of a millimetre short of the fully-open
 * position, because the open switch at 39.8 mm vetoed the last part of the move
 * and latched the setpoint at zero.  Functionally it looked like an intermittent
 * gripper, and the unit test caught it.
 *
 * # What happens without it
 * Without the veto, the force controller's release phase would hold the jaw
 * against the stop until the operator noticed the buzzing — a few seconds of
 * stall current is enough to damage a small brushed motor.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_GRIPPER_H
#define TP_GRIPPER_H

#include <stdint.h>

#include "bsp_types.h"
#include "encoder.h"
#include "motor.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Proportional gain of the jaw position loop, (mm/s) per mm. */
#define TP_GRIPPER_POS_KP 6.0f

/** @brief Position tolerance for @ref tp_gripper_is_at_target, mm. */
#define TP_GRIPPER_POS_TOLERANCE_MM 0.15f

/**
 * @brief One gripper axis.
 */
typedef struct {
    tp_motor_t   motor;    /**< Jaw drive channel.                            */
    tp_encoder_t encoder;  /**< Linear jaw scale.                             */

    float    gap_mm;       /**< Latest measured gap.                          */
    float    velocity_mm_s;/**< Latest measured jaw velocity.                 */
    float    target_mm;    /**< Target of the last position move.             */

    float    cmd_velocity_mm_s; /**< Velocity *requested* by the caller.       */
    float    applied_velocity_mm_s; /**< Velocity after the limit veto, i.e.
                                 *   what the drive is actually being asked for.
                                 *   Kept separate from the request on purpose:
                                 *   see the note below.                     */
    float    cmd_duty;     /**< Duty after the limit veto and clamping.       */

    int      limit_open;   /**< Non-zero when the open switch is asserted.    */
    int      limit_closed; /**< Non-zero when the closed switch is asserted.  */
    int      veto_active;  /**< Non-zero when a limit vetoed this update.     */
    int      position_mode;/**< Non-zero while a position move is active.     */

    uint32_t limit_events; /**< Times a limit vetoed a command.               */
    uint32_t move_count;   /**< Number of @ref tp_gripper_move_to_mm calls.   */
    uint32_t stall_events; /**< Times the jaw stopped without reaching target.*/
    int      initialised;  /**< Non-zero after a successful init.             */
} tp_gripper_t;

/**
 * @brief Initialise the gripper axis and home it against the open switch.
 *
 * Homing drives slowly towards open until the switch asserts, then declares
 * that position to be @ref TP_GRIPPER_MAX_MM.  A linear jaw has no absolute
 * reference, so this is the only way the firmware can know where it is; doing
 * it here means every caller gets a defined origin for free.
 *
 * @param[in,out] g     Instance.
 * @param[in]     dt_s  Control period in seconds.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM, @ref TP_ERR_IO when the switches
 *         never assert within the homing budget (indicating a wiring fault).
 */
tp_status_t tp_gripper_init(tp_gripper_t *g, float dt_s);

/**
 * @brief Command a jaw velocity.
 *
 * @param[in,out] g         Instance.
 * @param[in]     mm_s      Velocity in mm/s; negative closes.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE if the request
 *         exceeds @ref TP_GRIPPER_MAX_SPEED_MM_S.
 */
tp_status_t tp_gripper_set_velocity_mm_s(tp_gripper_t *g, float mm_s);

/**
 * @brief Command a bounded-speed move to a gap.
 *
 * @param[in,out] g          Instance.
 * @param[in]     target_mm  Target gap, clamped into the mechanical range.
 * @param[in]     speed_mm_s Maximum closing/opening speed, positive.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_gripper_move_to_mm(tp_gripper_t *g, float target_mm,
                                  float speed_mm_s);

/**
 * @brief Open the jaws fully.
 *
 * @param[in,out] g  Instance.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_gripper_open(tp_gripper_t *g);

/**
 * @brief Close the jaws at the fast-approach speed, without force feedback.
 *
 * This is the "dumb" primitive kept for the position-only baseline used by
 * Experiment 03.  Normal picking uses the force controller instead.
 *
 * @param[in,out] g  Instance.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_gripper_close(tp_gripper_t *g);

/**
 * @brief Command zero velocity but leave the drive enabled (actively braked).
 *
 * @param[in,out] g  Instance.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_gripper_stop(tp_gripper_t *g);

/**
 * @brief Run the position loop, apply the limit veto and write the motor.
 *
 * Must be called once per control period.
 *
 * @param[in,out] g     Instance.
 * @param[in]     dt_s  Elapsed seconds.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE when the drive is latched.
 */
tp_status_t tp_gripper_update(tp_gripper_t *g, float dt_s);

/**
 * @brief Latest measured gap.
 *
 * @param[in] g  Instance, may be NULL.
 * @return Gap in millimetres, or 0.0f when @p g is NULL.
 */
float tp_gripper_get_gap_mm(const tp_gripper_t *g);

/**
 * @brief Latest measured jaw velocity.
 *
 * @param[in] g  Instance.
 * @return Velocity in millimetres per second, negative when closing.
 */
float tp_gripper_get_velocity_mm_s(const tp_gripper_t *g);

/**
 * @brief Whether a position move has arrived.
 *
 * @param[in] g  Instance.
 * @return Non-zero when inside @ref TP_GRIPPER_POS_TOLERANCE_MM of the target,
 *         or when no move is active.
 */
int tp_gripper_is_at_target(const tp_gripper_t *g);

/**
 * @brief Whether the jaws are at or beyond the fully-open position.
 *
 * @param[in] g  Instance.
 * @return Non-zero when open.
 */
int tp_gripper_is_open(const tp_gripper_t *g);

/**
 * @brief Whether a limit switch is currently asserted.
 *
 * @param[in] g  Instance.
 * @return Non-zero when either the open or the closed switch reads asserted.
 */
int tp_gripper_at_limit(const tp_gripper_t *g);

/**
 * @brief Access the underlying motor channel, for the safety layer.
 *
 * The safety monitor needs the measured current and the ability to disable the
 * drive, and duplicating those through the gripper API would create two ways to
 * do the same thing.
 *
 * @param[in] g  Instance.
 * @return Pointer to the motor channel, or NULL.
 */
tp_motor_t *tp_gripper_motor(tp_gripper_t *g);

/**
 * @brief Access the underlying encoder channel.
 *
 * @param[in] g  Instance.
 * @return Pointer to the encoder channel, or NULL.
 */
tp_encoder_t *tp_gripper_encoder(tp_gripper_t *g);

#ifdef __cplusplus
}
#endif

#endif /* TP_GRIPPER_H */
