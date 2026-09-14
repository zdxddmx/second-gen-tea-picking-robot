/**
 * @file    robot_fsm.h
 * @brief   Table-driven finite state machine for the picking cycle.
 *
 * @details
 * # What it is
 * The decision layer.  It answers exactly one question 鈥?*"given that I am in
 * state S and event E just happened, where do I go next, and what do I do on
 * the way?"* 鈥?and nothing else.  It does not touch hardware, does not compute
 * control outputs and does not encode protocol frames.
 *
 * # Why a transition table instead of nested if/else
 * The picking cycle has 12 states and 14 events.  A hand-written `switch` over
 * states containing `switch` over events is roughly 170 lines of nested
 * branches in which:
 *
 *   - an unhandled `(state, event)` pair is **silently ignored**, so a missing
 *     transition looks identical to a correct no-op;
 *   - the set of legal transitions cannot be reviewed 鈥?you have to trace every
 *     branch to answer "can this machine ever get from GRIP to RELEASE without
 *     passing through PICK?";
 *   - adding a state means editing several places, and a forgotten one
 *     manifests as a robot that hangs in a state with no explanation.
 *
 * The table below is data.  It can be:
 *
 *   - **enumerated**: "show me every transition out of APPROACH" is a loop;
 *   - **validated**: a unit test walks the whole table and asserts that every
 *     state is reachable and that no transition targets an undefined state;
 *   - **logged**: the transition has a name, so the log line says
 *     `APPROACH-->CONTACT (CONTACT_DETECTED)` rather than `state 4 -> 5`;
 *   - **plotted**: the Mermaid FSM diagram in the README is generated from the
 *     same data (`scripts/gen_fsm_diagram.py`), so the diagram cannot drift out
 *     of date the way a hand-drawn one always does.
 *
 * # Guards and actions
 * A transition may carry:
 *   - a **guard**: a predicate that must hold for the transition to be taken.
 *     If it fails, the event is *consumed but not acted upon*, which is
 *     different from being ignored and is logged as such.
 *   - an **action**: run after the state changes, so the action can already see
 *     the new state (this ordering is what makes the FSM's own view of itself
 *     consistent inside an action).
 *
 * # What the FSM does not do
 * It does not run the pick *sequence* 鈥?@ref tp_force_control_t does, because
 * the sequence is a continuous control problem (six sub-states, regulated by a
 * PID) while the FSM is a discrete one.  Conflating the two produces a state
 * machine with a 1 kHz heartbeat, which is a well-known way to make both parts
 * hard to test.  The FSM's job in the GRIP state is to watch for the force
 * controller's completion and generate the corresponding event.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_ROBOT_FSM_H
#define TP_ROBOT_FSM_H

#include <stdint.h>

#include "robot_state.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Events that can drive the state machine.
 *
 * The numeric values are stable because they appear in the transition log.
 */
typedef enum {
    TP_EV_NONE = 0,             /**< No event; a plain periodic tick.        */
    TP_EV_SELF_CHECK_PASS = 1,  /**< Boot self-test succeeded.               */
    TP_EV_SELF_CHECK_FAIL = 2,  /**< Boot self-test failed.                  */
    TP_EV_TARGET = 3,           /**< A valid vision target arrived.          */
    TP_EV_START = 4,            /**< `START_PICK` received.                  */
    TP_EV_APPROACH_DONE = 5,    /**< The arm reached the pre-grasp pose.     */
    TP_EV_CONTACT = 6,          /**< The gripper detected contact.           */
    TP_EV_FORCE_REACHED = 7,    /**< The grip force settled at the setpoint. */
    TP_EV_PICK_DONE = 8,        /**< Detachment complete.                    */
    TP_EV_RELEASE_DONE = 9,     /**< Jaws open, bud dropped.                 */
    TP_EV_HOME_REACHED = 10,    /**< The arm is back at the home pose.       */
    TP_EV_STOP = 11,            /**< `STOP` received.                        */
    TP_EV_FAULT = 12,           /**< A latched fault was raised.             */
    TP_EV_ESTOP = 13,           /**< Emergency stop asserted.                */
    TP_EV_RESET = 14,           /**< `RESET` received and the fault cleared. */
    TP_EV_EVENT_COUNT = 15
} tp_event_t;

/**
 * @brief One entry of the transition table.
 *
 * Declared in the header so that a test (and the diagram generator) can walk the
 * table rather than duplicating it.
 *
 * @note A guard and an action both receive the **@ref tp_robot_fsm_t instance
 *       itself**, not the application's context pointer.  That is deliberate and
 *       it is not a detail: the first revision passed the application context
 *       and had the guards cast it to `tp_robot_fsm_t *`, which is a type error
 *       the compiler cannot see.  The guards then read whatever happened to sit
 *       at the FSM's field offsets inside the application struct 鈥?so
 *       `guard_has_target()` silently returned "no target" and the machine
 *       refused to start a cycle, intermittently, depending on unrelated fields.
 *       Everything a guard needs (the target flag, the fault latch, the E-stop
 *       state) is already mirrored into the FSM by the setters above, so passing
 *       the FSM is both sufficient and type-correct.
 */
typedef struct tp_robot_fsm tp_robot_fsm_t;

typedef struct {
    tp_robot_state_t from;      /**< Source state.                           */
    tp_event_t       event;     /**< Triggering event.                       */
    tp_robot_state_t to;        /**< Destination state.                      */
    const char      *name;      /**< Human-readable transition name.         */
    /** @brief Optional guard; NULL means "always allowed".
     *  Receives the FSM instance.  Return non-zero to allow the transition. */
    int (*guard)(tp_robot_fsm_t *fsm);
    /** @brief Optional action, run after the state has changed.  May be NULL.
     *  Receives the FSM instance. */
    void (*action)(tp_robot_fsm_t *fsm);
} tp_fsm_transition_t;

/**
 * @brief A recorded transition, for the trace and the `State vs Time` plot.
 *
 * The trace is a fixed-size ring: a robot running for a week must not grow its
 * memory, and the last 64 transitions are always more useful than the first 64.
 */
typedef struct {
    uint32_t         t_ms;    /**< When the transition happened.              */
    tp_robot_state_t from;    /**< Source state.                             */
    tp_robot_state_t to;      /**< Destination state.                        */
    tp_event_t       event;   /**< Event that caused it.                     */
    int              guard_failed; /**< Non-zero when the guard rejected it.  */
} tp_fsm_trace_entry_t;

/** @brief Number of transitions retained in the trace ring. */
#define TP_FSM_TRACE_DEPTH 64u

/**
 * @brief The state machine.
 */
struct tp_robot_fsm {
    tp_robot_state_t state;      /**< Current state.                          */
    tp_robot_state_t prev_state; /**< Previous state, for edge detection.     */
    uint32_t state_entered_ms;   /**< When the current state was entered.     */
    uint32_t now_ms;             /**< Latest time supplied by the caller.     */
    void    *ctx;                /**< Opaque context passed to guards/actions.*/

    int      fault_latched;      /**< Mirrors the fault manager's latch.      */
    int      estop_active;       /**< Mirrors the E-stop input.               */
    int      target_valid;       /**< Non-zero when a usable target is held.  */
    uint16_t target_id;          /**< Identifier of the held target.          */

    uint32_t transition_count;   /**< Total transitions taken.                */
    uint32_t rejected_events;    /**< Events with no matching transition.     */
    uint32_t guard_rejections;   /**< Events consumed but blocked by a guard. */
    uint32_t cycles_completed;   /**< Completed pick cycles.                  */

    tp_fsm_trace_entry_t trace[TP_FSM_TRACE_DEPTH]; /**< Transition ring.      */
    uint32_t trace_head;         /**< Next write index.                       */
    uint32_t trace_count;        /**< Entries stored, saturating at the depth.*/

    int      initialised;        /**< Non-zero after a successful init.       */
};

/**
 * @brief Initialise the state machine in @ref TP_STATE_INIT.
 *
 * @param[in,out] fsm    Instance.
 * @param[in]     unused Reserved for a future application-supplied guard
 *                       context.  The built-in guards and actions all take the
 *                       FSM itself, so passing NULL here is correct and is what
 *                       the production code does.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_fsm_init(tp_robot_fsm_t *fsm, void *unused);

/**
 * @brief Deliver an event and perform any resulting transition.
 *
 * @param[in,out] fsm    Instance.
 * @param[in]     event  Event to deliver.
 * @param[in]     now_ms Current timestamp.
 * @return @ref TP_OK when a transition was taken; @ref TP_ERR_NOT_FOUND when no
 *         transition matches (counted, and *not* logged at ERROR level because
 *         a periodic tick with no event is a perfectly normal no-op); and
 *         @ref TP_ERR_STATE when a matching transition's guard rejected it.
 */
tp_status_t tp_fsm_handle_event(tp_robot_fsm_t *fsm, tp_event_t event,
                               uint32_t now_ms);

/**
 * @brief Periodic tick: refreshes timing and evaluates time-driven transitions.
 *
 * @param[in,out] fsm    Instance.
 * @param[in]     now_ms Current timestamp.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_fsm_step(tp_robot_fsm_t *fsm, uint32_t now_ms);

/**
 * @brief Force a transition to a state, bypassing the table.
 *
 * Used only for the emergency paths, where the table must not be able to say
 * "no".  Every other transition goes through @ref tp_fsm_handle_event.
 *
 * @param[in,out] fsm    Instance.
 * @param[in]     state  Target state.
 * @param[in]     now_ms Current timestamp.
 * @param[in]     why    Reason, for the trace and the log.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_fsm_force_state(tp_robot_fsm_t *fsm, tp_robot_state_t state,
                               uint32_t now_ms, const char *why);

/**
 * @brief Current state.
 *
 * @param[in] fsm  Instance.
 * @return State.
 */
tp_robot_state_t tp_fsm_state(const tp_robot_fsm_t *fsm);

/**
 * @brief Milliseconds spent in the current state.
 *
 * @param[in] fsm  Instance.
 * @return Dwell time in milliseconds.
 */
uint32_t tp_fsm_state_dwell_ms(const tp_robot_fsm_t *fsm);

/**
 * @brief Notify the FSM that a fault has been latched or cleared.
 *
 * The FSM stores this so its guards can refuse to start a cycle on a faulted
 * machine *without* having to include the fault manager's header.
 *
 * @param[in,out] fsm     Instance.
 * @param[in]     latched Non-zero when a fault is latched.
 */
void tp_fsm_set_fault_state(tp_robot_fsm_t *fsm, int latched);

/**
 * @brief Notify the FSM of the E-stop input state.
 *
 * @param[in,out] fsm     Instance.
 * @param[in]     active  Non-zero when the E-stop is asserted.
 */
void tp_fsm_set_estop(tp_robot_fsm_t *fsm, int active);

/**
 * @brief Store a target identifier and mark the target as valid.
 *
 * @param[in,out] fsm        Instance.
 * @param[in]     target_id  Vision system's identifier.
 * @param[in]     valid      Non-zero when a usable target is held.
 */
void tp_fsm_set_target(tp_robot_fsm_t *fsm, uint16_t target_id, int valid);

/**
 * @brief Read the held target identifier.
 *
 * @param[in] fsm  Instance.
 * @return Target identifier, 0 when none.
 */
uint16_t tp_fsm_target_id(const tp_robot_fsm_t *fsm);

/**
 * @brief Whether a usable target is currently held.
 *
 * @param[in] fsm  Instance.
 * @return Non-zero when valid.
 */
int tp_fsm_has_target(const tp_robot_fsm_t *fsm);

/**
 * @brief Access the transition table.
 *
 * @param[out] count  Receives the number of entries.
 * @return Pointer to the static table, never NULL.
 */
const tp_fsm_transition_t *tp_fsm_table(uint32_t *count);

/**
 * @brief Copy out the transition trace, oldest first.
 *
 * @param[in]  fsm    Instance.
 * @param[out] out    Destination array.
 * @param[in]  max    Capacity of @p out.
 * @return Number of entries copied.
 */
uint32_t tp_fsm_get_trace(const tp_robot_fsm_t *fsm, tp_fsm_trace_entry_t *out,
                          uint32_t max);

/**
 * @brief Event name, for logs and for the trace dump.
 *
 * @param[in] event  Event.
 * @return Static string such as `"CONTACT_DETECTED"`.  Never NULL.
 */
const char *tp_fsm_event_name(tp_event_t event);

/**
 * @brief Whether an event can be delivered from a given state.
 *
 * Exposed so a unit test can assert the *reachability* of every state without
 * actually running the machine through a full cycle (a 2.5 s approach per test
 * would make the suite painfully slow).
 *
 * @param[in] from   Source state.
 * @param[in] event  Event.
 * @return Non-zero when a transition exists.
 */
int tp_fsm_can_handle(tp_robot_state_t from, tp_event_t event);

#ifdef __cplusplus
}
#endif

#endif /* TP_ROBOT_FSM_H */
