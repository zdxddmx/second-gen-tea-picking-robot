/**
 * @file    main_host.c
 * @brief   PC simulation entry point: runs the firmware against the physics.
 *
 * @details
 * # What this program is
 * It is the **firmware**, compiled for the host with `TP_PLATFORM_HOST`, driven
 * by a deterministic 1 ms scheduler, with the plant model standing in for the
 * motors and sensors.  It is not a re-implementation and not a mock: the same
 * `pid.c`, `cascade.c`, `force_control.c`, `robot_fsm.c`, `safety_monitor.c`
 * and `protocol.c` that run on the STM32 run here.
 *
 * That property is the whole reason the STM32 port is trustworthy.  A separate
 * PC model would validate the *model*; this validates the *firmware*.
 *
 * # What it produces
 *   - the log stream on stdout (the same format the target's debug UART emits);
 *   - a CSV trace in `data/logs/` with one row per control period;
 *   - a one-line summary per cycle, and a final PASS/FAIL.
 *
 * The CSV is deliberately the same column layout that
 * `tools/analyze_log.py` and the Python simulator's recorder use, so the three
 * can be plotted against each other without a conversion step.
 *
 * # Usage
 * @verbatim
 *   build/host/tea_picking_robot_host [--duration S] [--scenario A|B|C|D|E|F|G]
 *                                     [--csv PATH] [--quiet] [--cycles N]
 * @endverbatim
 *
 * @copyright MIT License, see LICENSE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_tasks.h"
#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_host.h"
#include "tp_format.h"

/** @brief Maximum simulated duration accepted from the command line, seconds. */
#define HOST_MAX_DURATION_S 120u

/** @brief Maximum pick cycles to attempt in one run. */
#define HOST_MAX_CYCLES 20u

/**
 * @brief One row of the CSV trace.
 *
 * Kept as a struct with an explicit writer rather than a chain of `printf`
 * calls so that the column order exists in exactly one place, and so a reader
 * can be written against the same struct.
 */
typedef struct {
    uint32_t t_ms;
    float    ref_mrad;
    float    pos_mrad;
    float    vel_mrad_s;
    float    force_n;
    float    gap_mm;
    float    duty;
    float    current_ma;
    uint8_t  state;
    uint8_t  grip_state;
    uint8_t  fault;
    uint16_t cycle;
} host_trace_row_t;

/** @brief Command-line options. */
typedef struct {
    uint32_t    duration_s;
    uint32_t    max_cycles;
    const char *csv_path;
    char        scenario;
    int         quiet;
} host_options_t;

/**
 * @brief Write the CSV header.
 *
 * @param[in] f  Open file.
 */
static void host_write_csv_header(FILE *f)
{
    (void)fprintf(f, "t_ms,ref_mrad,pos_mrad,vel_mrad_s,force_n,gap_mm,"
                     "duty_permille,current_ma,state,grip_state,fault,cycle\n");
}

/**
 * @brief Write one CSV row.
 *
 * @param[in] f    Open file.
 * @param[in] row  Row to write.
 */
static void host_write_csv_row(FILE *f, const host_trace_row_t *row)
{
    (void)fprintf(f, "%lu,%.2f,%.2f,%.2f,%.4f,%.3f,%.1f,%.1f,%u,%u,%u,%u\n",
                  (unsigned long)row->t_ms, (double)row->ref_mrad,
                  (double)row->pos_mrad, (double)row->vel_mrad_s,
                  (double)row->force_n, (double)row->gap_mm, (double)row->duty,
                  (double)row->current_ma, (unsigned)row->state,
                  (unsigned)row->grip_state, (unsigned)row->fault,
                  (unsigned)row->cycle);
}

/**
 * @brief Capture the current application state into a trace row.
 *
 * @param[in]  app  Application context.
 * @param[out] row  Row to fill.
 */
static void host_capture_row(const tp_app_context_t *app, host_trace_row_t *row)
{
    row->t_ms = tp_platform_time_ms();
    row->ref_mrad = tp_cascade_position_ref(&app->cascade);
    row->pos_mrad = app->meas_position_mrad;
    row->vel_mrad_s = app->meas_velocity_mrad_s;
    row->force_n = app->meas_force_n;
    row->gap_mm = tp_gripper_get_gap_mm(&app->gripper);
    row->duty = tp_motor_get_duty(&app->joint_motor);
    row->current_ma = app->meas_current_ma;
    row->state = (uint8_t)tp_fsm_state(&app->fsm);
    row->grip_state = (uint8_t)tp_force_control_state(&app->grip_ctl);
    row->fault = (uint8_t)tp_fault_highest(&app->faults);
    row->cycle = (uint16_t)((app->cycle_count > 65535u) ? 65535u
                                                       : app->cycle_count);
}

/** @brief What a scenario is supposed to demonstrate. */
typedef struct {
    char        letter;         /**< Scenario letter.                        */
    const char *title;          /**< Short description.                      */
    int         expect_cycle;   /**< Non-zero when a cycle should complete.  */
    int         expect_fault;   /**< 1 = a fault must be latched, 0 = none,
                                 *   -1 = either is acceptable.              */
    tp_fault_code_t alt_code;   /**< Second acceptable code, or
                                 *   @ref TP_FAULT_NONE.                      */
    tp_fault_code_t expect_code;/**< Specific code required, or
                                 *   @ref TP_FAULT_NONE when any will do.     */
    tp_robot_state_t expect_state; /**< Final state, or @ref TP_STATE_COUNT
                                    *   when it is not part of the criterion. */
} host_scenario_expect_t;

/**
 * @brief The seven scenarios and their success criteria.
 *
 * Declaring the expectation as data rather than as `if` statements in the
 * reporting code is what makes the verdict meaningful: for the fault-injection
 * scenarios **a completed pick cycle is a failure**, because the whole point is
 * that the protection stops the robot.  A harness that reported "0 cycles
 * completed" as a failure for scenario D would be reporting the correct
 * behaviour as a bug.
 */
static const host_scenario_expect_t s_scenarios[] = {
    { 'A', "nominal pick",                     1,  0, TP_FAULT_NONE, TP_FAULT_NONE,
      TP_STATE_IDLE },
    { 'B', "target position error",             0,  1, TP_FAULT_NONE, TP_FAULT_TARGET,
      TP_STATE_COUNT },
    { 'C', "force sensor noise",                1,  0, TP_FAULT_NONE, TP_FAULT_NONE,
      TP_STATE_IDLE },
    { 'D', "excessive grip force",              0,  1, TP_FAULT_NONE, TP_FAULT_FORCE_OVERLOAD,
      TP_STATE_COUNT },
    { 'E', "communication timeout",             0,  1, TP_FAULT_NONE, TP_FAULT_COMM_TIMEOUT,
      TP_STATE_COUNT },
    { 'F', "motor stall",                       0,  1, TP_FAULT_OVERCURRENT, TP_FAULT_MOTOR_STALL,
      TP_STATE_COUNT },
    { 'G', "emergency stop",                    0,  1, TP_FAULT_NONE, TP_FAULT_ESTOP,
      TP_STATE_EMERGENCY_STOP }
};

/**
 * @brief Look up a scenario's expectation.
 *
 * @param[in] letter  Scenario letter.
 * @return Pointer to the entry; falls back to scenario A for an unknown letter.
 */
static const host_scenario_expect_t *host_scenario_lookup(char letter)
{
    uint32_t i;

    for (i = 0u; i < TP_ARRAY_SIZE(s_scenarios); ++i) {
        if (s_scenarios[i].letter == letter) {
            return &s_scenarios[i];
        }
    }
    return &s_scenarios[0];
}

/**
 * @brief Apply a scenario's disturbance injection.
 *
 * The scenarios mirror the seven in `simulator/scenarios.py`, so a result from
 * the C firmware and a result from the Python model can be compared directly.
 * Each is implemented by manipulating the **plant** or the **link**, never the
 * controller: injecting a disturbance into the control law would test nothing.
 *
 * Every scenario that is supposed to run a cycle injects its own target, so the
 * scenarios are independent of each other and of the default branch.  The first
 * revision of this function relied on the `default` branch to supply the
 * target, which silently left scenarios C, E and F with nothing to do 鈥?they
 * ran for nine seconds in `IDLE` and "failed" for a reason that had nothing to
 * do with what they were testing.
 *
 * @param[in,out] app      Application context.
 * @param[in]     scenario Scenario letter.
 * @param[in]     t_ms     Current simulated time.
 */
static void host_apply_scenario(tp_app_context_t *app, char scenario, uint32_t t_ms)
{
    tp_plant_t *plant = tp_host_platform_plant();

    switch (scenario) {
    case 'B':
        /* Target position error: the vision system reports the bud 8 mm
         * laterally and 5 mm in depth away from where it is.  The lateral error
         * shifts the *expected* contact gap, so the gripper slows down at the
         * wrong distance and the force loop has less travel to work with. */
        if (t_ms == 1200u) {
            LOG_INFO(TP_MOD_TEST,
                     "scenario B: injecting a +8 mm / +5 mm vision target error");
            (void)tp_app_inject_target(app, 8, 20, 190, 95u, 2u);
            (void)tp_app_start_pick(app);
        }
        break;

    case 'C':
        /* Force-sensor noise: 80 mN RMS, roughly ten LSBs of the 8.0 mN
         * quantisation step, i.e. a sensor whose shielding is inadequate. */
        if (t_ms == 100u) {
            tp_plant_set_force_noise(plant, 0.08f);
            LOG_INFO(TP_MOD_TEST, "scenario C: force sensor noise sigma = 80 mN");
        }
        if (t_ms == 1200u) {
            (void)tp_app_inject_target(app, 0, 15, 185, 95u, 3u);
            (void)tp_app_start_pick(app);
        }
        break;

    case 'D':
        /* Excessive grip force, demonstrated the only honest way: with the
         * **position-only** controller of Experiment 03, closing to a fixed gap
         * on a stem that turns out to be stiffer and further out than the
         * approach model assumed.
         *
         * The first attempt at this scenario simply made the stem stiffer while
         * force control was active, and the cycle completed normally with a peak
         * of 3.2 N.  That is not a bug in the scenario, it is the central result
         * of the whole project: **a force-controlled gripper does not care how
         * stiff the bud is.**  To demonstrate the over-force protection you have
         * to remove the force feedback, which is exactly what the first-
         * generation machine did.
         *
         * With the jaws driving to a fixed 4 mm gap against a stem surface at
         * 16 mm, the model predicts 0.30*12 + 0.06*144 = 12.2 N - well past the
         * 5 N hard limit and past the 4 N damage threshold.  The safety monitor
         * is armed throughout, so this also measures how much damage the
         * supervision can actually prevent, and how late it is.
         */
        if (t_ms == 1100u) {
            LOG_WARN(TP_MOD_TEST,
                     "scenario D: POSITION-ONLY gripping, stem surface at 16 mm "
                     "(fixed 4 mm gap target) - force feedback disabled");
            tp_plant_set_stem_gap(plant, 16.0f);
            (void)tp_app_set_grip_mode(app, 1);
            (void)tp_app_inject_target(app, 0, 15, 185, 95u, 4u);
            (void)tp_app_start_pick(app);
        }
        break;

    case 'E':
        /* Communication timeout.
         *
         * The link supervision has to be *armed* before it can time out: a
         * robot that has never heard from a host must not report a link loss,
         * because at boot there is nothing to lose.  So the scenario first
         * injects one genuine heartbeat frame on the host bus 鈥?the same frame
         * a real host would send, built by the same encoder 鈥?and then goes
         * silent.  The timeout is therefore exercised through the production
         * path, exactly as it would be by unplugging the cable. */
        if (t_ms == 200u) {
            uint8_t frame[16];
            uint32_t len = tp_frame_encode(frame, sizeof(frame), TP_CMD_HEARTBEAT,
                                           NULL, 0u);
            LOG_INFO(TP_MOD_TEST,
                     "scenario E: one heartbeat frame, then the link goes silent");
            (void)tp_host_platform_inject_rx(TP_BUS_VISION_UART, frame, len);
        }
        if (t_ms == 400u) {
            (void)tp_app_inject_target(app, 0, 15, 185, 95u, 5u);
            (void)tp_app_start_pick(app);
        }
        break;

    case 'F':
        /* Motor stall: the joint is clamped harder than the drive can push.
         *
         * Two details make this a real test rather than a formality.  First,
         * the jam is applied *before* the move, because a stall detector that
         * only looks at speed cannot distinguish "stopped" from "stalled" 鈥?the
         * commanded duty has to be high for the condition to mean anything.
         * Second, the target is a long move (57 degrees), so the position loop
         * genuinely demands torque for long enough to be detected.  A small
         * move into a jam produces a small duty demand and correctly does not
         * trip anything. */
        if (t_ms == 900u) {
            LOG_INFO(TP_MOD_TEST,
                     "scenario F: clamping the joint (8.0 N*m, above the drive's "
                     "6.0 N*m capability)");
            tp_plant_set_jam(plant, 8.0f);
        }
        if (t_ms == 1200u) {
            /* y = 300 mm maps to a 1000 mrad move through TP_APP_MM_TO_MRAD. */
            (void)tp_app_inject_target(app, 0, 300, 185, 95u, 6u);
            (void)tp_app_start_pick(app);
        }
        break;

    case 'G':
        /* Emergency stop at 1.2 s, mid-approach. */
        if (t_ms == 1200u) {
            LOG_INFO(TP_MOD_TEST, "scenario G: emergency stop asserted");
            plant->estop_asserted = 1;
        }
        break;

    case 'A':
    default:
        /* Nominal: one bud near the centre of the workspace. */
        if (t_ms == 1200u) {
            (void)tp_app_inject_target(app, 0, 15, 185, 95u, 1u);
            (void)tp_app_start_pick(app);
        }
        break;
    }
}

/**
 * @brief Parse the command line.
 *
 * @param[in]  argc  Argument count.
 * @param[in]  argv  Argument vector.
 * @param[out] opt   Receives the parsed options.
 * @return 0 on success, 1 when the caller asked for help, 2 on a bad argument.
 */
static int host_parse_args(int argc, char **argv, host_options_t *opt)
{
    int i;

    opt->duration_s = 8u;
    opt->max_cycles = 1u;
    opt->csv_path = NULL;
    opt->scenario = 'A';
    opt->quiet = 0;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            return 1;
        } else if ((strcmp(argv[i], "--duration") == 0) && ((i + 1) < argc)) {
            long v = strtol(argv[++i], NULL, 10);
            if ((v <= 0) || (v > (long)HOST_MAX_DURATION_S)) {
                (void)fprintf(stderr, "duration must be 1..%u seconds\n",
                              (unsigned)HOST_MAX_DURATION_S);
                return 2;
            }
            opt->duration_s = (uint32_t)v;
        } else if ((strcmp(argv[i], "--cycles") == 0) && ((i + 1) < argc)) {
            long v = strtol(argv[++i], NULL, 10);
            if ((v <= 0) || (v > (long)HOST_MAX_CYCLES)) {
                (void)fprintf(stderr, "cycles must be 1..%u\n",
                              (unsigned)HOST_MAX_CYCLES);
                return 2;
            }
            opt->max_cycles = (uint32_t)v;
        } else if ((strcmp(argv[i], "--csv") == 0) && ((i + 1) < argc)) {
            opt->csv_path = argv[++i];
        } else if ((strcmp(argv[i], "--scenario") == 0) && ((i + 1) < argc)) {
            char c = argv[++i][0];
            if ((c >= 'a') && (c <= 'z')) {
                c = (char)(c - 'a' + 'A');
            }
            if ((c < 'A') || (c > 'G')) {
                (void)fprintf(stderr, "scenario must be A..G\n");
                return 2;
            }
            opt->scenario = c;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            opt->quiet = 1;
        } else {
            (void)fprintf(stderr, "unknown argument: %s\n", argv[i]);
            return 2;
        }
    }
    return 0;
}

/**
 * @brief Print usage.
 *
 * @param[in] prog  Program name.
 */
static void host_usage(const char *prog)
{
    (void)printf("Tea-picking robot firmware - host simulation build\n\n");
    (void)printf("Usage: %s [options]\n\n", prog);
    (void)printf("  --duration S     simulated seconds to run (1..%u, default 8)\n",
                 (unsigned)HOST_MAX_DURATION_S);
    (void)printf("  --cycles N       pick cycles to attempt (1..%u, default 1)\n",
                 (unsigned)HOST_MAX_CYCLES);
    (void)printf("  --scenario X     A..G (default A)\n");
    (void)printf("                     A nominal pick        B target position error\n");
    (void)printf("                     C sensor noise        D excessive grip force\n");
    (void)printf("                     E communication loss  F motor stall\n");
    (void)printf("                     G emergency stop\n");
    (void)printf("  --csv PATH       write a per-period trace\n");
    (void)printf("  --quiet          suppress the periodic status lines\n");
    (void)printf("  --help           this message\n");
}

/**
 * @brief Stand in for the PC on the other end of the host link.
 *
 * On the target the vision tool reads this link; in the host build nothing
 * does, so the platform's transmit ring fills up within a few hundred
 * milliseconds and every telemetry frame is reported as dropped 閳?a warning
 * that is an artefact of the test harness rather than of the firmware.
 *
 * This function is the reading half of the link: it drains the transmit rings,
 * counts the bytes, and re-parses the frames with the *same* parser the robot
 * uses.  Doing it that way means the host build exercises the encoder and the
 * decoder against each other on every run, which is a cheap and genuine
 * end-to-end check of the frame codec.
 */
typedef struct {
    tp_frame_parser_t parser;     /**< Host-side parser for the inbound stream.*/
    uint8_t           buffer[512];/**< Drain buffer.                          */
    uint32_t          bytes;      /**< Bytes consumed from the robot.          */
    uint32_t          frames;     /**< Valid frames decoded.                   */
    uint32_t          telemetry;  /**< TELEMETRY frames seen.                  */
    uint32_t          status;     /**< STATUS_RPT frames seen.                 */
    uint32_t          crc_errors; /**< Frames that failed their checksum.      */
    uint32_t          last_telemetry_ms; /**< Timestamp of the last telemetry. */
} host_peer_t;

/**
 * @brief Drain the robot's transmit rings and decode what came out.
 *
 * @param[in,out] peer  Host-side link state.
 */
static void host_peer_service(host_peer_t *peer)
{
    static const tp_bus_t buses[] = { TP_BUS_VISION_UART, TP_BUS_DEBUG_UART };
    uint32_t i;

    for (i = 0u; i < TP_ARRAY_SIZE(buses); ++i) {
        uint32_t n;
        while ((n = tp_host_platform_take_tx(buses[i], peer->buffer,
                                             sizeof(peer->buffer))) > 0u) {
            uint32_t k;
            peer->bytes += n;
            for (k = 0u; k < n; ++k) {
                tp_frame_t frame;
                tp_status_t st = tp_frame_parser_feed(&peer->parser,
                                                      peer->buffer[k],
                                                      tp_platform_time_ms(),
                                                      &frame);
                if (st == TP_OK) {
                    peer->frames++;
                    if (frame.cmd == TP_CMD_TELEMETRY) {
                        peer->telemetry++;
                        peer->last_telemetry_ms = tp_platform_time_ms();
                    } else if (frame.cmd == TP_CMD_STATUS_RPT) {
                        peer->status++;
                    }
                } else if (st == TP_ERR_CRC) {
                    peer->crc_errors++;
                }
            }
        }
    }
}

/**
 * @brief Program entry point.
 *
 * @param[in] argc  Argument count.
 * @param[in] argv  Argument vector.
 * @return 0 when the run completed, 1 when it failed or the self-check failed.
 */
int main(int argc, char **argv)
{
    static tp_app_context_t app;
    static host_trace_row_t row;
    static host_peer_t peer;
    host_options_t opt;
    FILE *csv = NULL;
    uint32_t t;
    uint32_t next_status_ms = 0u;
    uint32_t settle_until_ms = 0u;
    int arg_rc;
    int exit_code = 0;
    char status[320];
    tp_status_t st;

    arg_rc = host_parse_args(argc, argv, &opt);
    if (arg_rc == 1) {
        host_usage(argv[0]);
        return 0;
    }
    if (arg_rc != 0) {
        host_usage(argv[0]);
        return 2;
    }

    /* --- logging ------------------------------------------------------- */
    tp_log_set_sink(tp_log_stdout_sink, NULL);
    tp_log_set_time_source(tp_platform_time_ms);
    tp_log_set_level(opt.quiet ? TP_LOG_WARN : TP_LOG_INFO);

    /* --- CSV ------------------------------------------------------------ */
    if (opt.csv_path != NULL) {
        csv = fopen(opt.csv_path, "w");
        if (csv == NULL) {
            (void)fprintf(stderr, "cannot open %s for writing\n", opt.csv_path);
            return 2;
        }
        host_write_csv_header(csv);
    }

    (void)printf("===== Tea-Picking Robot: firmware simulation (host build) =====\n");
    (void)printf("scenario %c, %u simulated second(s), %u cycle(s) requested\n\n",
                 opt.scenario, (unsigned)opt.duration_s, (unsigned)opt.max_cycles);

    /* --- bring the firmware up ----------------------------------------- */
    st = tp_app_init(&app);
    if (st != TP_OK) {
        (void)printf("application init returned %s\n", tp_status_str(st));
        /* A failed self-check is a valid run: the fault must still be
         * reportable, which is exactly what the remaining loop does. */
        exit_code = 1;
    }

    (void)tp_app_format_status(&app, status, sizeof(status));
    (void)printf("after init: %s\n\n", status);

    /* Put the machine into AUTO so a target can actually be picked.  On a real
     * robot this is the host's `SET_MODE` command; the simulator sends the same
     * request through the same function the command handler calls. */
    (void)tp_app_set_mode(&app, TP_MODE_AUTO);
    tp_frame_parser_init(&peer.parser);

    /* --- main simulated loop ------------------------------------------- */
    for (t = 0u; (t / 1000u) < opt.duration_s; ++t) {
        /* Advance simulated time and the plant *before* running the tasks, in
         * the same order the bare-metal scheduler uses.  Doing it the other way
         * round would make every task observe a clock that lags its own inputs
         * by one period, which shows up as a one-sample delay in the loop
         * transfer function and a small but real phase-margin difference from
         * the target. */
        tp_host_platform_advance(1000u);
        tp_app_tick_1ms(&app);
        host_peer_service(&peer);

        if (csv != NULL) {
            host_capture_row(&app, &row);
            host_write_csv_row(csv, &row);
        }

        (void)host_apply_scenario(&app, opt.scenario, t);

        if (opt.quiet == 0) {
            if (t >= next_status_ms) {
                next_status_ms = t + 250u;
                (void)tp_app_format_status(&app, status, sizeof(status));
                (void)printf("%s\n", status);
            }
        }

        /* Stop early once the requested number of cycles has completed, plus a
         * short settle window so the trace and the log end on a quiescent state
         * rather than mid-transition. */
        if ((app.cycle_count >= opt.max_cycles) && (settle_until_ms == 0u)) {
            settle_until_ms = t + 300u;
            (void)printf("\ncycle %u completed at t=%lu ms\n",
                         (unsigned)app.cycle_count,
                         (unsigned long)tp_platform_time_ms());
        }
        if ((settle_until_ms != 0u) && (t >= settle_until_ms)) {
            break;
        }
    }

    /* --- report --------------------------------------------------------- */
    (void)tp_app_shutdown(&app);
    if (csv != NULL) {
        (void)fclose(csv);
        (void)printf("\ntrace written to %s\n", opt.csv_path);
    }

    (void)printf("\n===== summary =====\n");
    (void)printf("scenario            : %c\n", opt.scenario);
    (void)printf("simulated time      : %lu ms\n",
                 (unsigned long)tp_platform_time_ms());
    (void)printf("control cycles      : %lu\n", (unsigned long)app.control_cycles);
    (void)printf("sensor cycles       : %lu\n", (unsigned long)app.sensor_cycles);
    (void)printf("comm cycles         : %lu\n", (unsigned long)app.comm_cycles);
    (void)printf("robot cycles        : %lu\n", (unsigned long)app.robot_cycles);
    (void)printf("safety cycles       : %lu\n", (unsigned long)app.safety_cycles);
    (void)printf("telemetry frames    : %lu\n", (unsigned long)app.telemetry_sent);
    (void)printf("deadline misses     : %lu\n", (unsigned long)app.deadline_misses);
    (void)printf("pick cycles         : %lu\n", (unsigned long)app.cycle_count);
    (void)printf("final state         : %s\n",
                 tp_state_name(tp_fsm_state(&app.fsm)));
    (void)printf("final grip state    : %s\n",
                 tp_grip_state_name(tp_force_control_state(&app.grip_ctl)));
    (void)printf("peak grip force     : %.3f N (hard limit %.2f N)\n",
                 (double)tp_force_control_peak_force(&app.grip_ctl),
                 (double)TP_FORCE_MAX_N);
    (void)tp_fault_format_active(&app.faults, status, sizeof(status));
    (void)printf("faults              : %s\n", status);
    (void)printf("force sensor        : %s, %lu spike(s)\n",
                 tp_force_sensor_is_faulty(&app.force) ? "FAULTY" : "ok",
                 (unsigned long)tp_force_sensor_spike_count(&app.force));
    (void)printf("log                 : %lu emitted, %lu rate-limited\n",
                 (unsigned long)tp_log_get_emitted(),
                 (unsigned long)tp_log_get_dropped());
    (void)printf("host link (rx side) : %lu B, %lu frames, %lu telemetry, "
                 "%lu status, %lu CRC error(s)\n",
                 (unsigned long)peer.bytes, (unsigned long)peer.frames,
                 (unsigned long)peer.telemetry, (unsigned long)peer.status,
                 (unsigned long)peer.crc_errors);

    /* --- verdict --------------------------------------------------------
     * The expectation comes from the scenario table, not from "did a cycle
     * finish": for the fault-injection scenarios a *completed* cycle is the
     * failure. */
    {
        const host_scenario_expect_t *exp = host_scenario_lookup(opt.scenario);
        tp_fault_code_t top = tp_fault_highest(&app.faults);
        int cycle_ok;
        int fault_ok;
        int code_ok;
        int state_ok;
        int ok;

        cycle_ok = (exp->expect_cycle != 0) ? (app.cycle_count >= 1u)
                                            : (app.cycle_count == 0u);
        if (exp->expect_fault == 0) {
            fault_ok = (top == TP_FAULT_NONE);
        } else if (exp->expect_fault == 1) {
            /* A latched fault whose condition has since cleared is still a
             * correct outcome, so the history is checked rather than only the
             * live latch. */
            fault_ok = (tp_fault_any_latched(&app.faults) != 0) ||
                       (app.faults.raise_total > 0u);
        } else {
            fault_ok = 1;
        }
        if (exp->expect_code == TP_FAULT_NONE) {
            code_ok = 1;
        } else {
            /* Two codes may be acceptable: see scenario F, where the over-current
             * and stall protections race and either outcome is correct. */
            int hit_primary =
                (tp_fault_is_active(&app.faults, exp->expect_code) != 0) ||
                (tp_fault_is_latched(&app.faults, exp->expect_code) != 0) ||
                (app.faults.record[(int)exp->expect_code].count > 0u);
            int hit_alt = 0;
            if (exp->alt_code != TP_FAULT_NONE) {
                hit_alt =
                    (tp_fault_is_active(&app.faults, exp->alt_code) != 0) ||
                    (tp_fault_is_latched(&app.faults, exp->alt_code) != 0) ||
                    (app.faults.record[(int)exp->alt_code].count > 0u);
            }
            code_ok = (hit_primary || hit_alt);
        }
        state_ok = (exp->expect_state == TP_STATE_COUNT) ||
                   (tp_fsm_state(&app.fsm) == exp->expect_state);
        ok = cycle_ok && fault_ok && code_ok && state_ok;

        (void)printf("\n--- scenario %c: %s ---\n", exp->letter, exp->title);
        (void)printf("expected cycle completion : %s (got %lu)\n",
                     exp->expect_cycle ? "yes" : "no",
                     (unsigned long)app.cycle_count);
        (void)printf("expected fault            : %s\n",
                     (exp->expect_fault == 0) ? "none"
                     : ((exp->expect_fault == 1)
                            ? tp_fault_name(exp->expect_code)
                            : "either"));
        (void)printf("expected final state      : %s\n",
                     (exp->expect_state == TP_STATE_COUNT) ? "unconstrained"
                     : tp_state_name(exp->expect_state));
        (void)printf("criteria                  : cycle=%s fault=%s code=%s state=%s\n",
                     cycle_ok ? "ok" : "NOK", fault_ok ? "ok" : "NOK",
                     code_ok ? "ok" : "NOK", state_ok ? "ok" : "NOK");
        if (ok) {
            (void)printf("\nSimulation: PASS\n");
        } else {
            (void)printf("\nSimulation: FAIL\n");
            exit_code = 1;
        }
    }
    (void)printf("==============================================================\n");
    return exit_code;
}