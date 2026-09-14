/**
 * @file    test_fsm_fault.c
 * @brief   Unit tests for the robot state machine and the fault registry.
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include "fault_manager.h"
#include "robot_fsm.h"
#include "robot_state.h"

/* =========================================================================
 * Structural properties of the transition table
 *
 * These are the tests a hand-written if/else chain cannot have.  They walk the
 * table as data, so they keep working when a transition is added and they fail
 * loudly when one is added wrongly.
 * ========================================================================= */

TP_TEST_CASE(fsm, every_transition_targets_a_defined_state)
{
    uint32_t n = 0u;
    const tp_fsm_transition_t *table = tp_fsm_table(&n);
    uint32_t i;

    TP_ASSERT_GT(n, 0u);
    TP_ASSERT(table != NULL);
    for (i = 0u; i < n; ++i) {
        TP_ASSERT(table[i].to >= 0);
        TP_ASSERT(table[i].to < TP_STATE_COUNT);
        TP_ASSERT(table[i].from >= 0);
        TP_ASSERT(table[i].from < TP_STATE_COUNT);
        TP_ASSERT(table[i].event >= 0);
        TP_ASSERT(table[i].event < TP_EV_EVENT_COUNT);
        TP_ASSERT(table[i].name != NULL);
    }
}

TP_TEST_CASE(fsm, no_duplicate_state_event_pair)
{
    uint32_t n = 0u;
    const tp_fsm_transition_t *table = tp_fsm_table(&n);
    uint32_t i;
    uint32_t j;

    /* A duplicate pair means the second entry is dead code, and which one wins
     * depends on the search order 鈥?a latent bug that is invisible in review. */
    for (i = 0u; i < n; ++i) {
        for (j = i + 1u; j < n; ++j) {
            int same = (table[i].from == table[j].from) &&
                       (table[i].event == table[j].event);
            TP_ASSERT_MSG(same == 0, "duplicate (state, event) pair in the table");
        }
    }
}

TP_TEST_CASE(fsm, every_normal_state_is_reachable_from_idle)
{
    /* Flood fill over the transition graph and assert that every healthy state
     * can be entered.  An unreachable state is a design bug: it means a phase of
     * the cycle can never run, and the code inside it is untestable dead weight. */
    int reached[TP_STATE_COUNT];
    uint32_t n = 0u;
    const tp_fsm_transition_t *table = tp_fsm_table(&n);
    int changed = 1;
    int i;
    uint32_t k;

    for (i = 0; i < (int)TP_STATE_COUNT; ++i) {
        reached[i] = 0;
    }
    reached[TP_STATE_IDLE] = 1;
    while (changed != 0) {
        changed = 0;
        for (k = 0u; k < n; ++k) {
            if ((reached[(int)table[k].from] != 0) &&
                (reached[(int)table[k].to] == 0)) {
                reached[(int)table[k].to] = 1;
                changed = 1;
            }
        }
    }
    for (i = 0; i < (int)TP_STATE_COUNT; ++i) {
        if ((i == (int)TP_STATE_INIT) || (i == (int)TP_STATE_SELF_CHECK)) {
            continue; /* entered at boot, not from IDLE */
        }
        TP_ASSERT_MSG(reached[i] != 0, "state is unreachable from IDLE");
    }
}

TP_TEST_CASE(fsm, every_cyclic_state_has_a_stop_edge)
{
    int i;

    /* "STOP must work from everywhere" and "a fault must be able to stop the
     * machine from anywhere" are the two safety properties the table is
     * supposed to encode.  Checking them as properties rather than as individual
     * rows is what makes the test survive a table edit. */
    for (i = 0; i < (int)TP_STATE_COUNT; ++i) {
        tp_robot_state_t s = (tp_robot_state_t)i;

        if ((s == TP_STATE_ERROR) || (s == TP_STATE_EMERGENCY_STOP)) {
            continue; /* already stopped; only RESET leaves them */
        }
        if ((s == TP_STATE_INIT) || (s == TP_STATE_SELF_CHECK)) {
            /* The two boot states take SELF_CHECK_PASS / SELF_CHECK_FAIL
             * instead; there is nothing to stop yet. */
            continue;
        }
        TP_ASSERT_MSG(tp_fsm_can_handle(s, TP_EV_STOP) != 0,
                      "no STOP edge from this state");
    }
}

TP_TEST_CASE(fsm, error_and_estop_require_their_condition_to_clear)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);

    /* Force the machine into ERROR with a fault latched, then try to reset
     * without clearing the fault: the guard must refuse. */
    tp_fsm_set_fault_state(&fsm, 1);
    TP_ASSERT_EQ(tp_fsm_force_state(&fsm, TP_STATE_ERROR, 100u, "test"), TP_OK);
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_RESET, 200u), TP_ERR_STATE);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_ERROR);

    tp_fsm_set_fault_state(&fsm, 0);
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_RESET, 300u), TP_OK);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_IDLE);
}

/* =========================================================================
 * Behaviour
 * ========================================================================= */

TP_TEST_CASE(fsm, start_requires_a_valid_target)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_SELF_CHECK_PASS, 0u), TP_OK);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_IDLE);

    /* START with no target: the guard must refuse, and the state must not
     * change.  This is the bug that the wrong-context guard pointer produced 鈥?     * it refused *with* a target and allowed without one, intermittently. */
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_START, 10u), TP_ERR_STATE);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_IDLE);
    TP_ASSERT_EQ(fsm.guard_rejections, 1u);

    tp_fsm_set_target(&fsm, 42u, 1);
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_START, 20u), TP_OK);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_APPROACH);
}

TP_TEST_CASE(fsm, full_cycle_walks_every_phase_in_order)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    (void)tp_fsm_handle_event(&fsm, TP_EV_SELF_CHECK_PASS, 0u);
    tp_fsm_set_target(&fsm, 1u, 1);
    (void)tp_fsm_handle_event(&fsm, TP_EV_TARGET, 10u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_TARGET_RECEIVED);
    (void)tp_fsm_handle_event(&fsm, TP_EV_START, 20u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_APPROACH);
    (void)tp_fsm_handle_event(&fsm, TP_EV_APPROACH_DONE, 30u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_CONTACT);
    (void)tp_fsm_handle_event(&fsm, TP_EV_CONTACT, 40u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_GRIP);
    (void)tp_fsm_handle_event(&fsm, TP_EV_FORCE_REACHED, 50u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_PICK);
    (void)tp_fsm_handle_event(&fsm, TP_EV_PICK_DONE, 60u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_RELEASE);
    (void)tp_fsm_handle_event(&fsm, TP_EV_RELEASE_DONE, 70u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_RETURN);
    (void)tp_fsm_handle_event(&fsm, TP_EV_HOME_REACHED, 80u);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_IDLE);

    /* The cycle-complete action must have run exactly once. */
    TP_ASSERT_EQ(fsm.cycles_completed, 1u);
    /* And the consumed target must have been invalidated. */
    TP_ASSERT_EQ(tp_fsm_has_target(&fsm), 0);
}

TP_TEST_CASE(fsm, estop_from_every_state_lands_in_emergency_stop)
{
    int i;

    for (i = 0; i < (int)TP_STATE_COUNT; ++i) {
        tp_robot_fsm_t fsm;
        tp_robot_state_t s = (tp_robot_state_t)i;

        if (s == TP_STATE_EMERGENCY_STOP) {
            continue;
        }
        TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
        (void)tp_fsm_force_state(&fsm, s, 0u, "test setup");
        tp_fsm_set_estop(&fsm, 1);
        /* tp_fsm_step is the safety net: an E-stop must transition the machine
         * even if the application forgets to deliver the event. */
        (void)tp_fsm_step(&fsm, 10u);
        TP_ASSERT_MSG(tp_fsm_state(&fsm) == TP_STATE_EMERGENCY_STOP,
                      "E-stop did not force EMERGENCY_STOP");
    }
}

TP_TEST_CASE(fsm, unhandled_event_is_counted_not_fatal)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    (void)tp_fsm_handle_event(&fsm, TP_EV_SELF_CHECK_PASS, 0u);
    /* PICK_DONE makes no sense in IDLE.  It must be counted and ignored. */
    TP_ASSERT_EQ(tp_fsm_handle_event(&fsm, TP_EV_PICK_DONE, 10u),
                 TP_ERR_NOT_FOUND);
    TP_ASSERT_EQ(tp_fsm_state(&fsm), TP_STATE_IDLE);
    TP_ASSERT_EQ(fsm.rejected_events, 1u);
    TP_ASSERT_EQ(fsm.transition_count, 1u); /* only the boot transition */
}

TP_TEST_CASE(fsm, trace_records_transitions_oldest_first)
{
    tp_robot_fsm_t fsm;
    tp_fsm_trace_entry_t trace[8];
    uint32_t n;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    (void)tp_fsm_handle_event(&fsm, TP_EV_SELF_CHECK_PASS, 0u);
    tp_fsm_set_target(&fsm, 7u, 1);
    (void)tp_fsm_handle_event(&fsm, TP_EV_TARGET, 10u);

    n = tp_fsm_get_trace(&fsm, trace, 8u);
    TP_ASSERT_EQ(n, 2u);
    TP_ASSERT_EQ(trace[0].from, TP_STATE_INIT);
    TP_ASSERT_EQ(trace[0].to, TP_STATE_IDLE);
    TP_ASSERT_EQ(trace[1].from, TP_STATE_IDLE);
    TP_ASSERT_EQ(trace[1].to, TP_STATE_TARGET_RECEIVED);
    TP_ASSERT_EQ(trace[1].event, TP_EV_TARGET);
    TP_ASSERT_EQ(trace[1].t_ms, 10u);
}

TP_TEST_CASE(fsm, dwell_time_is_measured_from_entry)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    (void)tp_fsm_handle_event(&fsm, TP_EV_SELF_CHECK_PASS, 100u);
    (void)tp_fsm_step(&fsm, 350u);
    TP_ASSERT_EQ(tp_fsm_state_dwell_ms(&fsm), 250u);
}

TP_TEST_CASE(fsm, state_vocabulary_is_consistent)
{
    int i;

    for (i = 0; i < (int)TP_STATE_COUNT; ++i) {
        tp_robot_state_t s = (tp_robot_state_t)i;
        TP_ASSERT(tp_state_name(s) != NULL);
        TP_ASSERT(tp_state_desc(s) != NULL);
        TP_ASSERT(strcmp(tp_state_name(s), "unknown") != 0);
    }
    TP_ASSERT_STR_EQ(tp_state_name((tp_robot_state_t)999), "unknown");
    TP_ASSERT_EQ(tp_state_is_normal(TP_STATE_ERROR), 0);
    TP_ASSERT_EQ(tp_state_is_normal(TP_STATE_GRIP), 1);
    TP_ASSERT_EQ(tp_state_allows_motion(TP_STATE_INIT), 0);
    TP_ASSERT_EQ(tp_state_allows_motion(TP_STATE_APPROACH), 1);
    TP_ASSERT_EQ(tp_state_allows_motion(TP_STATE_EMERGENCY_STOP), 0);
}

TP_TEST_CASE(fsm, invalid_arguments_are_rejected)
{
    tp_robot_fsm_t fsm;

    TP_ASSERT_EQ(tp_fsm_init(NULL, NULL), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_fsm_init(&fsm, NULL), TP_OK);
    TP_ASSERT_EQ(tp_fsm_force_state(&fsm, (tp_robot_state_t)99, 0u, "x"),
                 TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_fsm_handle_event(NULL, TP_EV_START, 0u), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_fsm_get_trace(&fsm, NULL, 4u), 0u);
    TP_ASSERT_STR_EQ(tp_fsm_event_name((tp_event_t)999), "UNKNOWN");
}

/* =========================================================================
 * Fault registry
 * ========================================================================= */

TP_TEST_CASE(fault, raise_latches_and_counts)
{
    tp_fault_manager_t fm;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    TP_ASSERT_EQ(tp_fault_any_latched(&fm), 0);

    (void)tp_fault_raise(&fm, TP_FAULT_FORCE_OVERLOAD, 12u, 100u);
    TP_ASSERT_EQ(tp_fault_is_active(&fm, TP_FAULT_FORCE_OVERLOAD), 1);
    TP_ASSERT_EQ(tp_fault_is_latched(&fm, TP_FAULT_FORCE_OVERLOAD), 1);
    TP_ASSERT_EQ(tp_fault_any_latched(&fm), 1);
    TP_ASSERT_EQ(tp_fault_highest(&fm), TP_FAULT_FORCE_OVERLOAD);

    /* Re-raising an already-active fault must not write a second history entry,
     * or a 1 kHz detector would flush the ring in 16 ms. */
    (void)tp_fault_raise(&fm, TP_FAULT_FORCE_OVERLOAD, 13u, 101u);
    TP_ASSERT_EQ(fm.record[TP_FAULT_FORCE_OVERLOAD].count, 2u);
    TP_ASSERT_EQ(fm.history_count, 1u);
}

TP_TEST_CASE(fault, clear_is_refused_while_active)
{
    tp_fault_manager_t fm;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    (void)tp_fault_raise(&fm, TP_FAULT_OVERCURRENT, 0u, 100u);
    /* An operator must not be able to defeat a protection by acknowledging it
     * while the physical condition persists. */
    TP_ASSERT_EQ(tp_fault_clear(&fm, TP_FAULT_OVERCURRENT, 200u),
                 TP_ERR_FAULT_ACTIVE);
    TP_ASSERT_EQ(tp_fault_is_latched(&fm, TP_FAULT_OVERCURRENT), 1);

    (void)tp_fault_clear_condition(&fm, TP_FAULT_OVERCURRENT, 300u);
    TP_ASSERT_EQ(tp_fault_clear(&fm, TP_FAULT_OVERCURRENT, 400u), TP_OK);
    TP_ASSERT_EQ(tp_fault_is_latched(&fm, TP_FAULT_OVERCURRENT), 0);
    TP_ASSERT_EQ(tp_fault_any_latched(&fm), 0);
}

TP_TEST_CASE(fault, warning_severity_self_clears)
{
    tp_fault_manager_t fm;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    (void)tp_fault_raise(&fm, TP_FAULT_LIMIT, 0u, 100u);
    /* A limit switch is a WARNING: it must not require an acknowledgement. */
    TP_ASSERT_EQ(tp_fault_severity(TP_FAULT_LIMIT), TP_SEVERITY_WARNING);
    TP_ASSERT_EQ(tp_fault_is_latched(&fm, TP_FAULT_LIMIT), 0);
    (void)tp_fault_clear_condition(&fm, TP_FAULT_LIMIT, 200u);
    TP_ASSERT_EQ(tp_fault_is_active(&fm, TP_FAULT_LIMIT), 0);
}

TP_TEST_CASE(fault, errors_block_motion_and_criticals_kill)
{
    tp_fault_manager_t fm;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    TP_ASSERT_EQ(tp_fault_blocks_motion(&fm), 0);
    TP_ASSERT_EQ(tp_fault_requires_kill(&fm), 0);

    (void)tp_fault_raise(&fm, TP_FAULT_COMM_TIMEOUT, 0u, 10u);
    /* A comms blip permits motion: refusing to move on one would make the
     * machine unusable, which is why severity exists. */
    TP_ASSERT_EQ(tp_fault_blocks_motion(&fm), 0);
    TP_ASSERT_EQ(tp_fault_requires_kill(&fm), 0);

    (void)tp_fault_raise(&fm, TP_FAULT_MOTOR_STALL, 0u, 20u);
    TP_ASSERT_EQ(tp_fault_blocks_motion(&fm), 1);
    TP_ASSERT_EQ(tp_fault_requires_kill(&fm), 0);
    TP_ASSERT_EQ(tp_fault_highest(&fm), TP_FAULT_MOTOR_STALL);

    (void)tp_fault_raise(&fm, TP_FAULT_ESTOP, 0u, 30u);
    TP_ASSERT_EQ(tp_fault_requires_kill(&fm), 1);
    TP_ASSERT_EQ(tp_fault_highest(&fm), TP_FAULT_ESTOP);
}

TP_TEST_CASE(fault, clear_all_reports_how_many_were_cleared)
{
    tp_fault_manager_t fm;
    uint32_t cleared;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    (void)tp_fault_raise(&fm, TP_FAULT_OVERCURRENT, 0u, 10u);
    (void)tp_fault_raise(&fm, TP_FAULT_ENCODER, 0u, 10u);
    (void)tp_fault_clear_condition(&fm, TP_FAULT_OVERCURRENT, 20u);
    /* Only the one whose condition has gone may be cleared. */
    cleared = tp_fault_clear_all(&fm, 30u);
    TP_ASSERT_EQ(cleared, 1u);
    TP_ASSERT_EQ(tp_fault_is_latched(&fm, TP_FAULT_ENCODER), 1);
    TP_ASSERT_EQ(tp_fault_any_latched(&fm), 1);
}

TP_TEST_CASE(fault, active_time_accumulates)
{
    tp_fault_manager_t fm;
    tp_fault_record_t rec;

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    (void)tp_fault_raise(&fm, TP_FAULT_SENSOR, 0u, 0u);
    tp_fault_tick(&fm, 100u);
    tp_fault_tick(&fm, 150u);
    TP_ASSERT_EQ(tp_fault_get_record(&fm, TP_FAULT_SENSOR, &rec), TP_OK);
    TP_ASSERT_EQ(rec.active_ms, 250u);
}

TP_TEST_CASE(fault, every_code_has_a_name_and_a_hint)
{
    int i;

    for (i = 0; i < (int)TP_FAULT_CODE_COUNT; ++i) {
        tp_fault_code_t c = (tp_fault_code_t)i;
        TP_ASSERT(tp_fault_name(c) != NULL);
        TP_ASSERT(tp_fault_desc(c) != NULL);
        TP_ASSERT(tp_fault_recovery_hint(c) != NULL);
        TP_ASSERT(strcmp(tp_fault_name(c), "ERR_UNKNOWN") != 0);
    }
    /* An out-of-range code must fail safe, not fail open. */
    TP_ASSERT_EQ(tp_fault_severity((tp_fault_code_t)999), TP_SEVERITY_ERROR);
    TP_ASSERT_STR_EQ(tp_fault_name((tp_fault_code_t)999), "ERR_UNKNOWN");
}

TP_TEST_CASE(fault, invalid_arguments_are_rejected)
{
    tp_fault_manager_t fm;

    TP_ASSERT_EQ(tp_fault_manager_init(NULL), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    TP_ASSERT_EQ(tp_fault_raise(&fm, TP_FAULT_NONE, 0u, 0u), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_fault_raise(&fm, (tp_fault_code_t)999, 0u, 0u),
                 TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_fault_get_record(&fm, TP_FAULT_ESTOP, NULL), TP_ERR_PARAM);
}

TP_TEST_CASE(fault, formatted_summary_is_readable)
{
    tp_fault_manager_t fm;
    char buf[128];

    TP_ASSERT_EQ(tp_fault_manager_init(&fm), TP_OK);
    TP_ASSERT_EQ(tp_fault_format_active(&fm, buf, sizeof(buf)), 2u);
    TP_ASSERT_STR_EQ(buf, "OK");

    (void)tp_fault_raise(&fm, TP_FAULT_FORCE_OVERLOAD, 0u, 10u);
    TP_ASSERT_GT(tp_fault_format_active(&fm, buf, sizeof(buf)), 0u);
    TP_ASSERT(strstr(buf, "ERR_FORCE_OVERLOAD") != NULL);
}
