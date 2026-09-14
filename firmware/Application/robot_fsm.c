/**
 * @file    robot_fsm.c
 * @brief   The transition table and the state machine's implementation.
 * @copyright MIT License, see LICENSE.
 */

#include "robot_fsm.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"

/* =========================================================================
 * Guards
 * ========================================================================= */

/**
 * @brief Guard: only start a cycle when there is a target and no fault.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when the transition is allowed.
 */
static int guard_can_start(tp_robot_fsm_t *fsm)
{
    if (fsm == NULL) {
        return 0;
    }
    if (fsm->fault_latched != 0) {
        return 0;
    }
    if (fsm->estop_active != 0) {
        return 0;
    }
    return (fsm->target_valid != 0) ? 1 : 0;
}

/**
 * @brief Guard: a valid target must be held before APPROACH is entered.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when the transition is allowed.
 */
static int guard_has_target(tp_robot_fsm_t *fsm)
{
    if (fsm == NULL) {
        return 0;
    }
    return (fsm->target_valid != 0) &&
           (fsm->fault_latched == 0) &&
           (fsm->estop_active == 0);
}

/**
 * @brief Guard: a fault must be latched before ERROR is entered.
 *
 * Without this guard the FSM could be moved to `ERROR` by a spurious event and
 * would then refuse to leave, because the RESET transition requires the fault
 * state to have changed.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when a fault is actually latched.
 */
static int guard_has_fault(tp_robot_fsm_t *fsm)
{
    return ((fsm != NULL) && (fsm->fault_latched != 0)) ? 1 : 0;
}

/**
 * @brief Guard: the E-stop must be asserted before EMERGENCY_STOP is entered.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when the E-stop is asserted.
 */
static int guard_has_estop(tp_robot_fsm_t *fsm)
{
    return ((fsm != NULL) && (fsm->estop_active != 0)) ? 1 : 0;
}

/**
 * @brief Guard: leaving ERROR requires the fault to have been cleared.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when no fault is latched.
 */
static int guard_fault_cleared(tp_robot_fsm_t *fsm)
{
    return ((fsm != NULL) && (fsm->fault_latched == 0)) ? 1 : 0;
}

/**
 * @brief Guard: leaving EMERGENCY_STOP requires the E-stop to be released.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 * @return Non-zero when the E-stop is released.
 */
static int guard_estop_cleared(tp_robot_fsm_t *fsm)
{
    return ((fsm != NULL) && (fsm->estop_active == 0)) ? 1 : 0;
}

/* =========================================================================
 * Actions
 * ========================================================================= */

/**
 * @brief Action: count a completed cycle.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 */
static void action_cycle_complete(tp_robot_fsm_t *fsm)
{
    if (fsm != NULL) {
        fsm->cycles_completed++;
    }
}

/**
 * @brief Action: invalidate the target that has just been consumed.
 *
 * Marking the target stale at the *end* of a cycle rather than at the start
 * means a stop-and-resume during APPROACH can still find it.  A target becomes
 * invalid once it has actually been picked, which is the semantic the vision
 * system's re-detection logic expects.
 *
 * @param[in] ctx  The @ref tp_robot_fsm_t instance.
 */
static void action_consume_target(tp_robot_fsm_t *fsm)
{
    if (fsm != NULL) {
        fsm->target_valid = 0;
    }
}

/* =========================================================================
 * Transition table
 * ========================================================================= */

/**
 * @brief The complete transition table.
 *
 * Read it as the specification of the cycle.  Two properties are worth noting
 * explicitly because they are deliberate:
 *
 *   1. **`STOP` is legal from every cyclic state and always lands in `IDLE`.**
 *      A stop that depends on where you are is a stop you cannot rely on.
 *   2. **A fault or an E-stop is legal from every state**, including `INIT` and
 *      the abnormal ones, so a fault raised during boot still produces a
 *      correct `ERROR` entry.  The only state with no outgoing fault edge is
 *      `EMERGENCY_STOP` itself, because it is already the most severe.
 */
static const tp_fsm_transition_t s_transitions[] = {
/*  from                     event                    to                       name                     guard                action                */
{ TP_STATE_INIT,            TP_EV_SELF_CHECK_PASS,   TP_STATE_IDLE,           "init->idle",            NULL,                NULL                  },
{ TP_STATE_INIT,            TP_EV_SELF_CHECK_FAIL,   TP_STATE_ERROR,          "init->error",           NULL,                NULL                  },

{ TP_STATE_SELF_CHECK,      TP_EV_SELF_CHECK_PASS,   TP_STATE_IDLE,           "selfcheck->idle",       NULL,                NULL                  },
{ TP_STATE_SELF_CHECK,      TP_EV_SELF_CHECK_FAIL,   TP_STATE_ERROR,          "selfcheck->error",      NULL,                NULL                  },

{ TP_STATE_IDLE,            TP_EV_TARGET,            TP_STATE_TARGET_RECEIVED,"idle->target",          NULL,                NULL                  },
{ TP_STATE_IDLE,            TP_EV_START,             TP_STATE_APPROACH,       "idle->approach",        guard_can_start,     NULL                  },
{ TP_STATE_IDLE,            TP_EV_STOP,              TP_STATE_IDLE,           "idle->idle(stop)",      NULL,                NULL                  },

{ TP_STATE_TARGET_RECEIVED, TP_EV_START,             TP_STATE_APPROACH,       "target->approach",      guard_has_target,    NULL                  },
{ TP_STATE_TARGET_RECEIVED, TP_EV_TARGET,            TP_STATE_TARGET_RECEIVED,"target->target(update)",NULL,                NULL                  },
{ TP_STATE_TARGET_RECEIVED, TP_EV_STOP,              TP_STATE_IDLE,           "target->idle(stop)",    NULL,                NULL                  },

{ TP_STATE_APPROACH,        TP_EV_APPROACH_DONE,     TP_STATE_CONTACT,        "approach->contact",     NULL,                NULL                  },
{ TP_STATE_APPROACH,        TP_EV_CONTACT,           TP_STATE_CONTACT,        "approach->contact(ev)", NULL,                NULL                  },
{ TP_STATE_APPROACH,        TP_EV_STOP,              TP_STATE_IDLE,           "approach->idle(stop)",  NULL,                NULL                  },

{ TP_STATE_CONTACT,         TP_EV_CONTACT,           TP_STATE_GRIP,           "contact->grip",         NULL,                NULL                  },
{ TP_STATE_CONTACT,         TP_EV_FORCE_REACHED,     TP_STATE_GRIP,           "contact->grip(force)",  NULL,                NULL                  },
{ TP_STATE_CONTACT,         TP_EV_STOP,              TP_STATE_IDLE,           "contact->idle(stop)",   NULL,                NULL                  },

{ TP_STATE_GRIP,            TP_EV_FORCE_REACHED,     TP_STATE_PICK,           "grip->pick",            NULL,                NULL                  },
{ TP_STATE_GRIP,            TP_EV_PICK_DONE,         TP_STATE_RELEASE,        "grip->release",         NULL,                NULL                  },
{ TP_STATE_GRIP,            TP_EV_STOP,              TP_STATE_IDLE,           "grip->idle(stop)",      NULL,                NULL                  },

{ TP_STATE_PICK,            TP_EV_PICK_DONE,         TP_STATE_RELEASE,        "pick->release",         NULL,                NULL                  },
{ TP_STATE_PICK,            TP_EV_STOP,              TP_STATE_IDLE,           "pick->idle(stop)",      NULL,                NULL                  },

{ TP_STATE_RELEASE,         TP_EV_RELEASE_DONE,      TP_STATE_RETURN,         "release->return",       NULL,                action_cycle_complete },
{ TP_STATE_RELEASE,         TP_EV_STOP,              TP_STATE_IDLE,           "release->idle(stop)",   NULL,                NULL                  },

{ TP_STATE_RETURN,          TP_EV_HOME_REACHED,      TP_STATE_IDLE,           "return->idle",          NULL,                action_consume_target },
{ TP_STATE_RETURN,          TP_EV_STOP,              TP_STATE_IDLE,           "return->idle(stop)",    NULL,                NULL                  },

/* ERROR: only RESET gets out, and only once the fault has actually cleared. */
{ TP_STATE_ERROR,           TP_EV_RESET,             TP_STATE_IDLE,           "error->idle(reset)",    guard_fault_cleared, NULL                  },

/* EMERGENCY_STOP: only RESET, and only once the E-stop is physically released. */
{ TP_STATE_EMERGENCY_STOP,  TP_EV_RESET,             TP_STATE_IDLE,           "estop->idle(reset)",    guard_estop_cleared, NULL                  },

/* Global fault edges.  Written out one state at a time rather than handled by a
 * special case in the lookup, so that "can a fault be raised here?" has a
 * visible answer in the table for every state. */
{ TP_STATE_INIT,            TP_EV_FAULT,             TP_STATE_ERROR,          "init->error(fault)",    guard_has_fault,     NULL                  },
{ TP_STATE_SELF_CHECK,      TP_EV_FAULT,             TP_STATE_ERROR,          "selfcheck->error(fault)",guard_has_fault,    NULL                  },
{ TP_STATE_IDLE,            TP_EV_FAULT,             TP_STATE_ERROR,          "idle->error(fault)",    guard_has_fault,     NULL                  },
{ TP_STATE_TARGET_RECEIVED, TP_EV_FAULT,             TP_STATE_ERROR,          "target->error(fault)",  guard_has_fault,     NULL                  },
{ TP_STATE_APPROACH,        TP_EV_FAULT,             TP_STATE_ERROR,          "approach->error(fault)",guard_has_fault,     NULL                  },
{ TP_STATE_CONTACT,         TP_EV_FAULT,             TP_STATE_ERROR,          "contact->error(fault)", guard_has_fault,     NULL                  },
{ TP_STATE_GRIP,            TP_EV_FAULT,             TP_STATE_ERROR,          "grip->error(fault)",    guard_has_fault,     NULL                  },
{ TP_STATE_PICK,            TP_EV_FAULT,             TP_STATE_ERROR,          "pick->error(fault)",    guard_has_fault,     NULL                  },
{ TP_STATE_RELEASE,         TP_EV_FAULT,             TP_STATE_ERROR,          "release->error(fault)", guard_has_fault,     NULL                  },
{ TP_STATE_RETURN,          TP_EV_FAULT,             TP_STATE_ERROR,          "return->error(fault)",  guard_has_fault,     NULL                  },
{ TP_STATE_ERROR,           TP_EV_FAULT,             TP_STATE_ERROR,          "error->error(fault)",   guard_has_fault,     NULL                  },

/* Global emergency edges.  The E-stop does not pass through ERROR: it is a
 * separate state because its recovery procedure is different (release the
 * button, then reset) and because the drives are killed in hardware, not
 * merely commanded to stop. */
{ TP_STATE_INIT,            TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "init->estop",           guard_has_estop,     NULL                  },
{ TP_STATE_SELF_CHECK,      TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "selfcheck->estop",      guard_has_estop,     NULL                  },
{ TP_STATE_IDLE,            TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "idle->estop",           guard_has_estop,     NULL                  },
{ TP_STATE_TARGET_RECEIVED, TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "target->estop",         guard_has_estop,     NULL                  },
{ TP_STATE_APPROACH,        TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "approach->estop",       guard_has_estop,     NULL                  },
{ TP_STATE_CONTACT,         TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "contact->estop",        guard_has_estop,     NULL                  },
{ TP_STATE_GRIP,            TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "grip->estop",           guard_has_estop,     NULL                  },
{ TP_STATE_PICK,            TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "pick->estop",           guard_has_estop,     NULL                  },
{ TP_STATE_RELEASE,         TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "release->estop",        guard_has_estop,     NULL                  },
{ TP_STATE_RETURN,          TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "return->estop",         guard_has_estop,     NULL                  },
{ TP_STATE_ERROR,           TP_EV_ESTOP,             TP_STATE_EMERGENCY_STOP, "error->estop",          guard_has_estop,     NULL                  }
};

/** @brief Number of entries in the transition table. */
#define TP_FSM_TRANSITION_COUNT (sizeof(s_transitions) / sizeof(s_transitions[0]))

/** @brief Event names, indexed by @ref tp_event_t. */
static const char *const s_event_names[TP_EV_EVENT_COUNT] = {
    "NONE", "SELF_CHECK_PASS", "SELF_CHECK_FAIL", "TARGET", "START",
    "APPROACH_DONE", "CONTACT_DETECTED", "FORCE_REACHED", "PICK_DONE",
    "RELEASE_DONE", "HOME_REACHED", "STOP", "FAULT", "E_STOP", "RESET"
};

const char *tp_fsm_event_name(tp_event_t event)
{
    if (((int)event < 0) || (event >= TP_EV_EVENT_COUNT)) {
        return "UNKNOWN";
    }
    return s_event_names[(int)event];
}

const tp_fsm_transition_t *tp_fsm_table(uint32_t *count)
{
    if (count != NULL) {
        *count = (uint32_t)TP_FSM_TRANSITION_COUNT;
    }
    return s_transitions;
}

/**
 * @brief Append an entry to the transition trace ring.
 *
 * @param[in,out] fsm           Instance.
 * @param[in]     from          Source state.
 * @param[in]     to            Destination state.
 * @param[in]     event         Event.
 * @param[in]     guard_failed  Whether the guard rejected the transition.
 * @param[in]     now_ms        Timestamp.
 */
static void fsm_trace_push(tp_robot_fsm_t *fsm, tp_robot_state_t from,
                           tp_robot_state_t to, tp_event_t event,
                           int guard_failed, uint32_t now_ms)
{
    tp_fsm_trace_entry_t *e = &fsm->trace[fsm->trace_head];

    e->t_ms = now_ms;
    e->from = from;
    e->to = to;
    e->event = event;
    e->guard_failed = guard_failed;
    fsm->trace_head = (fsm->trace_head + 1u) % TP_FSM_TRACE_DEPTH;
    if (fsm->trace_count < TP_FSM_TRACE_DEPTH) {
        fsm->trace_count++;
    }
}

tp_status_t tp_fsm_init(tp_robot_fsm_t *fsm, void *ctx)
{
    if (fsm == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(fsm, 0, sizeof(*fsm));
    fsm->state = TP_STATE_INIT;
    fsm->prev_state = TP_STATE_INIT;
    fsm->ctx = ctx;
    fsm->initialised = 1;
    LOG_INFO(TP_MOD_ROBOT, "FSM initialised in state %s (%u transitions defined)",
             tp_state_name(fsm->state), (unsigned)TP_FSM_TRANSITION_COUNT);
    return TP_OK;
}

tp_status_t tp_fsm_handle_event(tp_robot_fsm_t *fsm, tp_event_t event,
                               uint32_t now_ms)
{
    uint32_t i;

    if ((fsm == NULL) || (fsm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    fsm->now_ms = now_ms;

    for (i = 0u; i < TP_FSM_TRANSITION_COUNT; ++i) {
        const tp_fsm_transition_t *t = &s_transitions[i];

        if ((t->from != fsm->state) || (t->event != event)) {
            continue;
        }
        if ((t->guard != NULL) && (t->guard(fsm) == 0)) {
            /* The event matched but the guard refused.  This is recorded as a
             * rejection rather than a no-op so a stuck "START does nothing"
             * report can be diagnosed from the counters alone. */
            fsm->guard_rejections++;
            fsm_trace_push(fsm, fsm->state, fsm->state, event, 1, now_ms);
            LOG_DEBUG(TP_MOD_ROBOT, "%s rejected: guard failed in state %s",
                      tp_fsm_event_name(event), tp_state_name(fsm->state));
            return TP_ERR_STATE;
        }

        fsm->prev_state = fsm->state;
        fsm->state = t->to;
        fsm->state_entered_ms = now_ms;
        fsm->transition_count++;
        fsm_trace_push(fsm, fsm->prev_state, fsm->state, event, 0, now_ms);

        LOG_INFO(TP_MOD_ROBOT, "%s --> %s (%s)", tp_state_name(fsm->prev_state),
                 tp_state_name(fsm->state), tp_fsm_event_name(event));

        if (t->action != NULL) {
            /* Run the action *after* the state has changed, so an action can
             * read the FSM and see the new state.  The opposite order makes
             * actions that inspect the current state silently wrong. */
            t->action(fsm);
        }
        return TP_OK;
    }

    fsm->rejected_events++;
    return TP_ERR_NOT_FOUND;
}

tp_status_t tp_fsm_step(tp_robot_fsm_t *fsm, uint32_t now_ms)
{
    if ((fsm == NULL) || (fsm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    fsm->now_ms = now_ms;

    /* Time-driven safety: an E-stop or a fault raised while the FSM is sitting
     * in a cyclic state must transition it even if the application forgot to
     * send the event.  Doing it here means the FSM cannot be left in a state
     * that claims motion is permitted while the hardware is killed. */
    if (fsm->estop_active != 0 && fsm->state != TP_STATE_EMERGENCY_STOP) {
        return tp_fsm_handle_event(fsm, TP_EV_ESTOP, now_ms);
    }
    if ((fsm->fault_latched != 0) &&
        (fsm->state != TP_STATE_ERROR) &&
        (fsm->state != TP_STATE_EMERGENCY_STOP)) {
        return tp_fsm_handle_event(fsm, TP_EV_FAULT, now_ms);
    }
    return TP_OK;
}

tp_status_t tp_fsm_force_state(tp_robot_fsm_t *fsm, tp_robot_state_t state,
                               uint32_t now_ms, const char *why)
{
    if ((fsm == NULL) || (fsm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (((int)state < 0) || (state >= TP_STATE_COUNT)) {
        return TP_ERR_RANGE;
    }
    if (fsm->state == state) {
        return TP_OK;
    }
    fsm->prev_state = fsm->state;
    fsm->state = state;
    fsm->state_entered_ms = now_ms;
    fsm->transition_count++;
    fsm_trace_push(fsm, fsm->prev_state, fsm->state, TP_EV_NONE, 0, now_ms);
    LOG_WARN(TP_MOD_ROBOT, "forced %s --> %s (%s)", tp_state_name(fsm->prev_state),
             tp_state_name(fsm->state), (why != NULL) ? why : "no reason given");
    return TP_OK;
}

tp_robot_state_t tp_fsm_state(const tp_robot_fsm_t *fsm)
{
    return (fsm != NULL) ? fsm->state : TP_STATE_ERROR;
}

uint32_t tp_fsm_state_dwell_ms(const tp_robot_fsm_t *fsm)
{
    if (fsm == NULL) {
        return 0u;
    }
    return fsm->now_ms - fsm->state_entered_ms;
}

void tp_fsm_set_fault_state(tp_robot_fsm_t *fsm, int latched)
{
    if (fsm == NULL) {
        return;
    }
    fsm->fault_latched = (latched != 0) ? 1 : 0;
}

void tp_fsm_set_estop(tp_robot_fsm_t *fsm, int active)
{
    if (fsm == NULL) {
        return;
    }
    fsm->estop_active = (active != 0) ? 1 : 0;
}

void tp_fsm_set_target(tp_robot_fsm_t *fsm, uint16_t target_id, int valid)
{
    if (fsm == NULL) {
        return;
    }
    fsm->target_id = target_id;
    fsm->target_valid = (valid != 0) ? 1 : 0;
}

uint16_t tp_fsm_target_id(const tp_robot_fsm_t *fsm)
{
    return (fsm != NULL) ? fsm->target_id : 0u;
}

int tp_fsm_has_target(const tp_robot_fsm_t *fsm)
{
    return (fsm != NULL) ? fsm->target_valid : 0;
}

uint32_t tp_fsm_get_trace(const tp_robot_fsm_t *fsm, tp_fsm_trace_entry_t *out,
                          uint32_t max)
{
    uint32_t n;
    uint32_t i;
    uint32_t start;

    if ((fsm == NULL) || (out == NULL) || (max == 0u)) {
        return 0u;
    }
    n = (fsm->trace_count < max) ? fsm->trace_count : max;
    /* Oldest first: the ring's head points at the *next* write, so the oldest
     * entry is `trace_count` positions behind it (or at 0 while filling). */
    start = (fsm->trace_count < TP_FSM_TRACE_DEPTH)
          ? 0u
          : fsm->trace_head;
    for (i = 0u; i < n; ++i) {
        out[i] = fsm->trace[(start + i) % TP_FSM_TRACE_DEPTH];
    }
    return n;
}

int tp_fsm_can_handle(tp_robot_state_t from, tp_event_t event)
{
    uint32_t i;

    for (i = 0u; i < TP_FSM_TRANSITION_COUNT; ++i) {
        if ((s_transitions[i].from == from) && (s_transitions[i].event == event)) {
            return 1;
        }
    }
    return 0;
}
