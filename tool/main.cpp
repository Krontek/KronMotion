/*===========================================================================
 * KronMotion Test Tool
 *
 * Dear ImGui + ImPlot GUI for motion profile analysis. Drives the NC engine
 * directly (no MC function blocks, no CiA402). Includes random excitation
 * modes and discontinuity/overshoot logging.
 *===========================================================================*/

#include <cstdio>
#include <cstring>
#include <cmath>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <random>
#include <deque>
#include <string>
#include <fstream>
#include <ctime>
#include <filesystem>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>

extern "C" {
#include "kronmotion.h"
#include "kron_nc.h"
#include "kron_pi.h"
}

/* ── Globals required by kron_pi.h ──────────────────────────────────────── */
KRON_PROCESS_IMAGE Kron_PI;
KRON_HAL_Driver   *Kron_HAL = nullptr;

/* ── Log entry ──────────────────────────────────────────────────────────── */
struct LogEntry {
    float       time;
    float       pos;
    float       vel;
    std::string msg;
    ImVec4      color;
};

/* ── Plot data ──────────────────────────────────────────────────────────── */
static constexpr int MAX_POINTS = 200000;

struct PlotData {
    std::vector<float> time;
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> acceleration;
    std::vector<float> jerk_plot;    /* numerical jerk (da/dt) */
    std::mutex         mtx;
    float              t_elapsed = 0.0f;

    /* previous values for discontinuity detection */
    float prev_vel  = 0.0f;
    float prev_acc  = 0.0f;
    float prev_jerk = 0.0f;

    void clear() {
        std::lock_guard<std::mutex> lk(mtx);
        time.clear(); position.clear(); velocity.clear();
        acceleration.clear(); jerk_plot.clear();
        t_elapsed = 0.0f;
        prev_vel = prev_acc = prev_jerk = 0.0f;
    }

    void push(float t, float p, float v, float a, float j) {
        std::lock_guard<std::mutex> lk(mtx);
        if ((int)time.size() >= MAX_POINTS) {
            time.erase(time.begin());
            position.erase(position.begin());
            velocity.erase(velocity.begin());
            acceleration.erase(acceleration.begin());
            jerk_plot.erase(jerk_plot.begin());
        }
        time.push_back(t);
        position.push_back(p);
        velocity.push_back(v);
        acceleration.push_back(a);
        jerk_plot.push_back(j);
    }
};

/* ── Publish a command directly to NC engine ────────────────────────────── */
static void publish_cmd(AXIS_REF *axis, NC_CMD_TYPE cmd,
                        float target, float vel, float acc, float dec, float jerk)
{
    axis->cmd_Cmd       = cmd;
    axis->cmd_TargetPos = target;
    axis->cmd_TargetVel = vel;
    axis->cmd_Accel     = acc;
    axis->cmd_Decel     = dec;
    axis->cmd_Jerk      = jerk;
    KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
}

/* ── Command cache for smart anomaly detection ─────────────────────────── */
struct CommandCache {
    /* ── Command identity ──────────────────────────────────────────────── */
    uint16_t    seq;                /* latched_seq at latch time           */
    NC_CMD_TYPE cmd;                /* command type                        */

    /* ── Commanded limits ──────────────────────────────────────────────── */
    float   target_pos;             /* position target (abs/rel)           */
    float   v_max;                  /* velocity limit                      */
    float   acc_limit;              /* acceleration limit                  */
    float   dec_limit;              /* deceleration limit                  */
    float   jerk_limit;             /* jerk limit                          */

    /* ── State at command latch (initial conditions) ───────────────────── */
    float   start_pos;              /* position when command was latched   */
    float   start_vel;              /* velocity when command was latched   */
    float   start_acc;              /* acceleration when command was latched*/
    float   start_time;             /* time when command was latched       */

    /* ── Tracking flags ────────────────────────────────────────────────── */
    bool    vel_settled;            /* |vel| has been <= v_max at least once*/
    bool    acc_settled;            /* |acc| has been <= acc_limit once     */
    bool    target_crossed;         /* position crossed target (overshoot) */
    int     suppress_cycles;        /* cycles to suppress disc. checks     */

    /* ── Peak tracking for log context ─────────────────────────────────── */
    float   peak_vel;               /* max |vel| seen in this command      */
    float   peak_acc;               /* max |acc| seen in this command      */
    float   peak_vel_time;          /* time when peak vel occurred         */
    float   peak_acc_time;          /* time when peak acc occurred         */

    /* ── Rate limiting: only log first occurrence + summary on clear ──── */
    bool    vel_overshoot_active;   /* currently in vel overshoot state    */
    float   vel_os_start_time;      /* when vel overshoot started          */
    float   vel_os_peak;            /* peak |vel| during this overshoot    */
    float   vel_os_peak_time;       /* time of peak vel overshoot          */
    int     vel_os_cycles;          /* how many cycles in overshoot        */

    bool    acc_overshoot_active;   /* currently in acc overshoot state    */
    float   acc_os_start_time;
    float   acc_os_peak;
    float   acc_os_peak_time;
    int     acc_os_cycles;
    bool    acc_os_was_speeding_up; /* acc or dec at start of overshoot    */
};

/* ── Discontinuity / overshoot detection ────────────────────────────────── */
struct AnalysisParams {
    float vel_disc_threshold;    /* max allowed dv/dt jump vs expected */
    float acc_disc_threshold;    /* max allowed da/dt jump vs expected */
    float vel_overshoot_pct;     /* % overshoot tolerance for vel      */
    float acc_overshoot_pct;     /* % overshoot tolerance for acc      */
    float pos_overshoot_abs;     /* absolute position overshoot [u]    */
};

/* ── Helper: command type to readable string ───────────────────────────── */
static const char *cmd_type_str(NC_CMD_TYPE cmd)
{
    switch (cmd) {
    case NC_CMD_NONE:     return "NONE";
    case NC_CMD_MOVE_ABS: return "MoveAbs";
    case NC_CMD_MOVE_REL: return "MoveRel";
    case NC_CMD_MOVE_VEL: return "MoveVel";
    case NC_CMD_HALT:     return "Halt";
    case NC_CMD_STOP:     return "Stop";
    default:              return "Unknown";
    }
}

/* ── Format command context block for log ───────────────────────────────── */
static std::string fmt_cmd_context(const CommandCache &cc, float t_now)
{
    char buf[512];
    snprintf(buf, sizeof(buf),
             "\n    CMD: %s (seq=%u) latched at t=%.3fs"
             "\n    Limits: v_max=%.2f acc=%.2f dec=%.2f jerk=%.0f target_pos=%.2f"
             "\n    Initial: pos=%.4f vel=%.4f acc=%.4f"
             "\n    Peaks: |vel|=%.4f@%.3fs |acc|=%.4f@%.3fs"
             "\n    Elapsed: %.3fs since latch",
             cmd_type_str(cc.cmd), cc.seq, cc.start_time,
             cc.v_max, cc.acc_limit, cc.dec_limit, cc.jerk_limit, cc.target_pos,
             cc.start_pos, cc.start_vel, cc.start_acc,
             cc.peak_vel, cc.peak_vel_time, cc.peak_acc, cc.peak_acc_time,
             t_now - cc.start_time);
    return buf;
}

static void check_anomalies(PlotData *plot, NC_AXIS *nc, float dt,
                            const AnalysisParams &ap, CommandCache &cc,
                            std::deque<LogEntry> &log, std::mutex &log_mtx)
{
    float pos  = nc->priv.cmd_pos;
    float vel  = nc->priv.cmd_vel;
    float acc  = nc->priv.cmd_acc;
    float t    = plot->t_elapsed;

    /* ── Detect new command latch → update cache ───────────────────────── */
    bool new_cmd = (nc->priv.latched_seq != cc.seq);
    if (new_cmd) {
        /* Close any active overshoot events from previous command */
        if (cc.vel_overshoot_active) {
            float duration = t - cc.vel_os_start_time;
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] VEL_OVERSHOOT_END: interrupted by new CMD after %.3fs (%d cycles)"
                     "\n    Peak |vel|=%.4f at t=%.3fs, limit=%.4f"
                     "%s",
                     t, duration, cc.vel_os_cycles,
                     cc.vel_os_peak, cc.vel_os_peak_time, cc.v_max,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(0.8f, 0.8f, 0.2f, 1)});
        }
        if (cc.acc_overshoot_active) {
            float duration = t - cc.acc_os_start_time;
            const char *label = cc.acc_os_was_speeding_up ? "ACC" : "DEC";
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] %s_OVERSHOOT_END: interrupted by new CMD after %.3fs (%d cycles)"
                     "\n    Peak |acc|=%.4f at t=%.3fs, limit=%.4f"
                     "%s",
                     t, label, duration, cc.acc_os_cycles,
                     cc.acc_os_peak, cc.acc_os_peak_time,
                     cc.acc_os_was_speeding_up ? cc.acc_limit : cc.dec_limit,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(0.7f, 0.5f, 0.2f, 1)});
        }

        cc.seq          = nc->priv.latched_seq;
        cc.cmd          = nc->priv.latched_cmd;
        cc.target_pos   = nc->priv.target_pos;
        cc.v_max        = nc->priv.v_max;
        cc.acc_limit    = nc->priv.acc;
        cc.dec_limit    = nc->priv.dec;
        cc.jerk_limit   = nc->priv.jerk;
        cc.start_pos    = pos;
        cc.start_vel    = vel;
        cc.start_acc    = acc;
        cc.start_time   = t;
        cc.peak_vel     = std::fabs(vel);
        cc.peak_acc     = std::fabs(acc);
        cc.peak_vel_time = t;
        cc.peak_acc_time = t;

        /* Mark as NOT settled if starting above limits (inherited from prev motion) */
        cc.vel_settled  = (std::fabs(vel) <= cc.v_max + 1e-4f);
        cc.acc_settled  = (std::fabs(acc) <= std::fmax(cc.acc_limit, cc.dec_limit) + 1e-4f);
        cc.target_crossed = false;

        /* Reset rate limiting state */
        cc.vel_overshoot_active = false;
        cc.acc_overshoot_active = false;

        /* Suppress discontinuity for 2 cycles after transition */
        cc.suppress_cycles = 2;
    }

    /* ── Update peak tracking ──────────────────────────────────────────── */
    if (std::fabs(vel) > cc.peak_vel) {
        cc.peak_vel = std::fabs(vel);
        cc.peak_vel_time = t;
    }
    if (std::fabs(acc) > cc.peak_acc) {
        cc.peak_acc = std::fabs(acc);
        cc.peak_acc_time = t;
    }

    /* ── Suppress discontinuity checks after cmd transition ────────────── */
    bool suppress_disc = (cc.suppress_cycles > 0);
    if (cc.suppress_cycles > 0) cc.suppress_cycles--;

    /* numerical jerk for discontinuity detection */
    float jerk_num = (acc - plot->prev_acc) / dt;

    /* ══════════════════════════════════════════════════════════════════════
     * 1. VELOCITY DISCONTINUITY — |dv| jumps beyond expected |a*dt|
     * ══════════════════════════════════════════════════════════════════════ */
    if (!suppress_disc && plot->t_elapsed > dt * 2.0f) {
        float dv = vel - plot->prev_vel;
        float expected_dv = plot->prev_acc * dt;
        float vel_err = std::fabs(dv - expected_dv);
        if (vel_err > ap.vel_disc_threshold) {
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] VEL_DISCONTINUITY: dv=%.6f expected_dv=%.6f err=%.6f"
                     "\n    State: pos=%.4f vel=%.4f acc=%.4f prev_vel=%.4f prev_acc=%.4f"
                     "%s",
                     t, dv, expected_dv, vel_err,
                     pos, vel, acc, plot->prev_vel, plot->prev_acc,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(1, 0.4f, 0.4f, 1)});
        }
    }

    /* ══════════════════════════════════════════════════════════════════════
     * 2. ACCELERATION DISCONTINUITY — |da| jumps beyond expected jerk*dt
     * ══════════════════════════════════════════════════════════════════════ */
    if (!suppress_disc && plot->t_elapsed > dt * 2.0f) {
        float da = acc - plot->prev_acc;
        float expected_da_mag = cc.jerk_limit * dt;
        /* Natural arrival: vel≈0, acc→0, prev had motion */
        bool at_arrival = std::fabs(vel) < 1e-3f && std::fabs(acc) < 1e-3f
                          && std::fabs(plot->prev_vel) > 1e-3f;
        if (expected_da_mag > 0.0f
            && std::fabs(da) > expected_da_mag + ap.acc_disc_threshold
            && !at_arrival) {
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] ACC_DISCONTINUITY: da=%.6f max_expected=%.6f"
                     "\n    State: pos=%.4f vel=%.4f acc=%.4f prev_acc=%.4f"
                     "%s",
                     t, da, expected_da_mag,
                     pos, vel, acc, plot->prev_acc,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(1, 0.6f, 0.2f, 1)});
        }
    }

    /* ══════════════════════════════════════════════════════════════════════
     * 3. VELOCITY OVERSHOOT — smart, rate-limited.
     *    - Inherited overshoot (from previous command) is ignored until settled.
     *    - Only logs ONCE when overshoot starts, then a SUMMARY when it clears.
     *    - Direction aware: positive/negative velocity tracked separately.
     * ══════════════════════════════════════════════════════════════════════ */
    {
        float abs_vel = std::fabs(vel);
        float vel_limit = cc.v_max;
        float tolerance = vel_limit * (ap.vel_overshoot_pct / 100.0f);

        /* Track settling: once |vel| drops to/below limit, mark settled */
        if (!cc.vel_settled && abs_vel <= vel_limit + 1e-4f) {
            cc.vel_settled = true;
        }

        /* Determine if velocity is in overshoot right now */
        bool in_vel_overshoot = (vel_limit > 0.0f
                                 && cc.vel_settled
                                 && abs_vel > vel_limit + tolerance);

        if (in_vel_overshoot && !cc.vel_overshoot_active) {
            /* ── Overshoot just started → log once ──────────────────────── */
            cc.vel_overshoot_active = true;
            cc.vel_os_start_time = t;
            cc.vel_os_peak = abs_vel;
            cc.vel_os_peak_time = t;
            cc.vel_os_cycles = 1;

            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] VEL_OVERSHOOT_START: |vel|=%.4f limit=%.4f (+%.1f%% tol=%.4f)"
                     "\n    Direction: vel=%+.4f acc=%+.4f"
                     "\n    Started with: vel=%+.4f (settled=%s)"
                     "%s",
                     t, abs_vel, vel_limit, ap.vel_overshoot_pct, tolerance,
                     vel, acc,
                     cc.start_vel, cc.vel_settled ? "yes" : "no",
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(1, 1, 0.2f, 1)});

        } else if (in_vel_overshoot && cc.vel_overshoot_active) {
            /* ── Still in overshoot → track peak silently ────────────── */
            cc.vel_os_cycles++;
            if (abs_vel > cc.vel_os_peak) {
                cc.vel_os_peak = abs_vel;
                cc.vel_os_peak_time = t;
            }

        } else if (!in_vel_overshoot && cc.vel_overshoot_active) {
            /* ── Overshoot just cleared → log summary ───────────────── */
            cc.vel_overshoot_active = false;
            float duration = t - cc.vel_os_start_time;

            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] VEL_OVERSHOOT_END: cleared after %.3fs (%d cycles)"
                     "\n    Peak |vel|=%.4f at t=%.3fs, limit=%.4f"
                     "\n    Current: vel=%+.4f acc=%+.4f"
                     "%s",
                     t, duration, cc.vel_os_cycles,
                     cc.vel_os_peak, cc.vel_os_peak_time, vel_limit,
                     vel, acc,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(0.8f, 0.8f, 0.2f, 1)});
        }
    }

    /* ══════════════════════════════════════════════════════════════════════
     * 4. ACCELERATION OVERSHOOT — smart, rate-limited.
     *    Uses acc_limit when speeding up, dec_limit when slowing down.
     *    Ignores inherited high-acc from previous command until settled.
     *    Logs once at start, once at end with summary.
     * ══════════════════════════════════════════════════════════════════════ */
    {
        float abs_acc = std::fabs(acc);
        bool speeding_up = (vel > 0.0f && acc > 0.0f) || (vel < 0.0f && acc < 0.0f);
        float limit = speeding_up ? cc.acc_limit : cc.dec_limit;
        float tolerance = limit * (ap.acc_overshoot_pct / 100.0f);

        /* Track settling */
        float max_limit = std::fmax(cc.acc_limit, cc.dec_limit);
        if (!cc.acc_settled && abs_acc <= max_limit + 1e-4f) {
            cc.acc_settled = true;
        }

        /* Determine if acc is in overshoot right now */
        bool in_acc_overshoot = (limit > 0.0f
                                 && cc.acc_settled
                                 && abs_acc > limit + tolerance);

        if (in_acc_overshoot && !cc.acc_overshoot_active) {
            /* ── Overshoot just started → log once ──────────────────────── */
            cc.acc_overshoot_active = true;
            cc.acc_os_was_speeding_up = speeding_up;
            cc.acc_os_start_time = t;
            cc.acc_os_peak = abs_acc;
            cc.acc_os_peak_time = t;
            cc.acc_os_cycles = 1;

            const char *label = speeding_up ? "ACC" : "DEC";
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] %s_OVERSHOOT_START: |acc|=%.4f limit=%.4f (+%.1f%% tol=%.4f)"
                     "\n    Direction: vel=%+.4f acc=%+.4f jerk=%+.1f"
                     "\n    Started with: acc=%+.4f (settled=%s)"
                     "%s",
                     t, label, abs_acc, limit, ap.acc_overshoot_pct, tolerance,
                     vel, acc, jerk_num,
                     cc.start_acc, cc.acc_settled ? "yes" : "no",
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf,
                           speeding_up ? ImVec4(1, 0.8f, 0.2f, 1)
                                       : ImVec4(1, 0.5f, 0.8f, 1)});

        } else if (in_acc_overshoot && cc.acc_overshoot_active) {
            /* ── Still in overshoot → track peak silently ────────────── */
            cc.acc_os_cycles++;
            if (abs_acc > cc.acc_os_peak) {
                cc.acc_os_peak = abs_acc;
                cc.acc_os_peak_time = t;
            }

        } else if (!in_acc_overshoot && cc.acc_overshoot_active) {
            /* ── Overshoot just cleared → log summary ───────────────── */
            cc.acc_overshoot_active = false;
            float duration = t - cc.acc_os_start_time;
            const char *label = cc.acc_os_was_speeding_up ? "ACC" : "DEC";

            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] %s_OVERSHOOT_END: cleared after %.3fs (%d cycles)"
                     "\n    Peak |acc|=%.4f at t=%.3fs, limit=%.4f"
                     "\n    Current: vel=%+.4f acc=%+.4f"
                     "%s",
                     t, label, duration, cc.acc_os_cycles,
                     cc.acc_os_peak, cc.acc_os_peak_time,
                     cc.acc_os_was_speeding_up ? cc.acc_limit : cc.dec_limit,
                     vel, acc,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(0.7f, 0.5f, 0.2f, 1)});
        }
    }

    /* ══════════════════════════════════════════════════════════════════════
     * 5. POSITION OVERSHOOT — detect when position crosses target during
     *    discrete motion (MoveAbs/MoveRel).  Direction-aware: checks if
     *    pos went past target in the direction of travel.
     *
     *    Only fires once per command (target_crossed flag).
     * ══════════════════════════════════════════════════════════════════════ */
    if (!cc.target_crossed
        && (cc.cmd == NC_CMD_MOVE_ABS || cc.cmd == NC_CMD_MOVE_REL)
        && nc->ref->sts_State == MC_AXIS_DISCRETE_MOTION) {
        float tgt = cc.target_pos;
        float tol = ap.pos_overshoot_abs;
        /* Determine original direction of travel */
        float dir = tgt - cc.start_pos;  /* positive = moving positive */

        /* Overshoot: position passed target in the direction of travel */
        bool overshoot = false;
        float overshoot_amount = 0.0f;
        if (dir > 1e-6f) {
            /* Moving positive: overshoot if pos > target + tolerance */
            overshoot = (pos > tgt + tol);
            overshoot_amount = pos - tgt;
        } else if (dir < -1e-6f) {
            /* Moving negative: overshoot if pos < target - tolerance */
            overshoot = (pos < tgt - tol);
            overshoot_amount = tgt - pos;
        }

        if (overshoot) {
            cc.target_crossed = true;
            char buf[768];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] POS_OVERSHOOT: pos=%.6f target=%.6f overshoot=%.6f"
                     "\n    Direction: %s (start=%.4f -> target=%.4f)"
                     "\n    State: vel=%+.4f acc=%+.4f"
                     "%s",
                     t, pos, tgt, overshoot_amount,
                     dir > 0 ? "positive" : "negative", cc.start_pos, tgt,
                     vel, acc,
                     fmt_cmd_context(cc, t).c_str());
            std::lock_guard<std::mutex> lk(log_mtx);
            log.push_back({t, pos, vel, buf, ImVec4(1, 0.2f, 1, 1)});
        }
    }

    plot->prev_vel  = vel;
    plot->prev_acc  = acc;
    plot->prev_jerk = jerk_num;
}

/* ── NC fast task thread ────────────────────────────────────────────────── */
static std::atomic<bool> g_running{true};

/* shared state for analysis thresholds */
static std::atomic<float> g_target_pos{0.0f};
static std::atomic<float> g_max_vel{50.0f};
static std::atomic<float> g_max_acc{200.0f};
static std::atomic<float> g_max_dec{200.0f};
static std::atomic<float> g_pos_tolerance{0.01f}; /* settling tolerance */

static void nc_thread_func(NC_AXIS *nc, PlotData *plot, float dt,
                           std::deque<LogEntry> *log, std::mutex *log_mtx,
                           const AnalysisParams *ap)
{
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds((int)(dt * 1e6f));

    MC_AXIS_STATE prev_state = MC_AXIS_STANDSTILL;

    /* Initialize command cache */
    CommandCache cc{};
    cc.seq = nc->priv.latched_seq;

    while (g_running.load(std::memory_order_relaxed)) {
        auto t0 = clock::now();

        NC_ProcessOne(nc, dt);

        float t   = plot->t_elapsed;
        float pos = nc->ref->CommandedPosition;
        float vel = nc->ref->CommandedVelocity;
        float acc = nc->priv.cmd_acc;
        float jrk = (plot->t_elapsed > dt) ? (acc - plot->prev_acc) / dt : 0.0f;

        /* anomaly check with command cache */
        check_anomalies(plot, nc, dt, *ap, cc, *log, *log_mtx);

        /* position settling check: discrete motion -> standstill
         * Uses cached target (not g_target_pos) to avoid race with random thread */
        MC_AXIS_STATE cur_state = nc->ref->sts_State;
        if (prev_state == MC_AXIS_DISCRETE_MOTION && cur_state == MC_AXIS_STANDSTILL) {
            float tgt = cc.target_pos;
            float tol = g_pos_tolerance.load();
            float err = std::fabs(pos - tgt);
            if (err > tol) {
                char buf[768];
                snprintf(buf, sizeof(buf),
                         "[%.3fs] POS_SETTLE_ERROR: pos=%.6f target=%.6f err=%.6f"
                         "\n    Should have reached target but didn't."
                         "\n    Direction: %s (start=%.4f -> target=%.4f)"
                         "%s",
                         t, pos, tgt, err,
                         (tgt > cc.start_pos) ? "positive" : "negative",
                         cc.start_pos, tgt,
                         fmt_cmd_context(cc, t).c_str());
                std::lock_guard<std::mutex> lk(*log_mtx);
                log->push_back({t, pos, vel, buf, ImVec4(1, 0.2f, 1, 1)});
            }
        }
        prev_state = cur_state;

        plot->push(t, pos, vel, acc, jrk);
        plot->t_elapsed += dt;

        auto elapsed = clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
}

/* ── Random excitation thread ───────────────────────────────────────────── */
static std::atomic<bool> g_random_pos{false};
static std::atomic<bool> g_random_vel{false};

struct RandomParams {
    float pos_min, pos_max;
    float vel_min, vel_max;
    float acc_min, acc_max;
    float dec_min, dec_max;
    float jerk_min, jerk_max;
    float interval_min, interval_max;  /* seconds between triggers */
};

static void random_thread_func(AXIS_REF *axis, const RandomParams *rp,
                               PlotData *plot,
                               std::deque<LogEntry> *log, std::mutex *log_mtx)
{
    std::mt19937 rng(std::random_device{}());

    while (g_running.load(std::memory_order_relaxed)) {
        bool do_pos = g_random_pos.load(std::memory_order_relaxed);
        bool do_vel = g_random_vel.load(std::memory_order_relaxed);

        if (!do_pos && !do_vel) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        /* random interval */
        std::uniform_real_distribution<float> dist_interval(rp->interval_min, rp->interval_max);
        float wait = dist_interval(rng);

        /* random parameters */
        std::uniform_real_distribution<float> dist_pos(rp->pos_min, rp->pos_max);
        std::uniform_real_distribution<float> dist_vel(rp->vel_min, rp->vel_max);
        std::uniform_real_distribution<float> dist_acc(rp->acc_min, rp->acc_max);
        std::uniform_real_distribution<float> dist_dec(rp->dec_min, rp->dec_max);
        std::uniform_real_distribution<float> dist_jerk(rp->jerk_min, rp->jerk_max);

        float vel  = dist_vel(rng);
        float acc  = dist_acc(rng);
        float dec  = dist_dec(rng);
        float jerk = dist_jerk(rng);

        float cur_pos = axis->CommandedPosition;
        float cur_vel = axis->CommandedVelocity;
        float t       = plot->t_elapsed;

        if (do_pos) {
            float tgt = dist_pos(rng);
            g_target_pos.store(tgt);
            g_max_vel.store(vel);
            g_max_acc.store(acc);
            g_max_dec.store(dec);
            publish_cmd(axis, NC_CMD_MOVE_ABS, tgt, vel, acc, dec, jerk);

            char buf[384];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] CMD MoveAbs: target=%.2f vel=%.2f acc=%.2f dec=%.2f jerk=%.0f "
                     "(cur_pos=%.4f cur_vel=%.4f)",
                     t, tgt, vel, acc, dec, jerk, cur_pos, cur_vel);
            std::lock_guard<std::mutex> lk(*log_mtx);
            log->push_back({t, cur_pos, cur_vel, buf, ImVec4(0.4f, 0.7f, 1, 1)});
        } else if (do_vel) {
            std::uniform_real_distribution<float> dist_svel(-rp->vel_max, rp->vel_max);
            float svel = dist_svel(rng);
            g_max_vel.store(std::fabs(svel));
            g_max_acc.store(acc);
            g_max_dec.store(dec);
            publish_cmd(axis, NC_CMD_MOVE_VEL, 0.0f, svel, acc, dec, jerk);

            char buf[384];
            snprintf(buf, sizeof(buf),
                     "[%.3fs] CMD MoveVel: target_vel=%.2f acc=%.2f dec=%.2f jerk=%.0f "
                     "(cur_pos=%.4f cur_vel=%.4f)",
                     t, svel, acc, dec, jerk, cur_pos, cur_vel);
            std::lock_guard<std::mutex> lk(*log_mtx);
            log->push_back({t, cur_pos, cur_vel, buf, ImVec4(0.4f, 0.7f, 1, 1)});
        }

        /* sleep for random interval, checking exit flag */
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds((int)(wait * 1000.0f));
        while (std::chrono::steady_clock::now() < deadline &&
               g_running.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

/* ── Save log to file ───────────────────────────────────────────────────── */
static std::string save_log(const std::deque<LogEntry> &entries,
                            const PlotData &plot, const char *tool_dir)
{
    time_t now = std::time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    char fname[128];
    snprintf(fname, sizeof(fname), "%s/%04d-%02d-%02d_%02d-%02d-%02d.log",
             tool_dir,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);

    std::ofstream f(fname);
    if (!f.is_open()) return {};

    f << "# KronMotion Analysis Log\n";
    f << "# Date: " << fname << "\n\n";

    /* anomaly summary */
    int cnt_vel_disc = 0, cnt_acc_disc = 0;
    int cnt_vel_os_start = 0, cnt_vel_os_end = 0;
    int cnt_acc_os_start = 0, cnt_acc_os_end = 0;
    int cnt_pos_over = 0, cnt_pos_settle = 0, cnt_cmd = 0;
    for (auto &e : entries) {
        if (e.msg.find("VEL_DISCONTINUITY") != std::string::npos) cnt_vel_disc++;
        else if (e.msg.find("ACC_DISCONTINUITY") != std::string::npos) cnt_acc_disc++;
        else if (e.msg.find("VEL_OVERSHOOT_START") != std::string::npos) cnt_vel_os_start++;
        else if (e.msg.find("VEL_OVERSHOOT_END") != std::string::npos) cnt_vel_os_end++;
        else if (e.msg.find("ACC_OVERSHOOT_START") != std::string::npos ||
                 e.msg.find("DEC_OVERSHOOT_START") != std::string::npos) cnt_acc_os_start++;
        else if (e.msg.find("ACC_OVERSHOOT_END") != std::string::npos ||
                 e.msg.find("DEC_OVERSHOOT_END") != std::string::npos) cnt_acc_os_end++;
        else if (e.msg.find("POS_OVERSHOOT") != std::string::npos) cnt_pos_over++;
        else if (e.msg.find("POS_SETTLE_ERROR") != std::string::npos) cnt_pos_settle++;
        else if (e.msg.find("CMD ") != std::string::npos) cnt_cmd++;
    }

    f << "# ── Summary ──\n";
    f << "# Commands issued:         " << cnt_cmd << "\n";
    f << "# Vel discontinuities:     " << cnt_vel_disc << "\n";
    f << "# Acc discontinuities:     " << cnt_acc_disc << "\n";
    f << "# Vel overshoot events:    " << cnt_vel_os_start
      << " (cleared: " << cnt_vel_os_end << ")\n";
    f << "# Acc/Dec overshoot events:" << cnt_acc_os_start
      << " (cleared: " << cnt_acc_os_end << ")\n";
    f << "# Pos overshoot (motion):  " << cnt_pos_over << "\n";
    f << "# Pos settle error:        " << cnt_pos_settle << "\n";
    f << "#\n";
    f << "# NOTE: Each anomaly includes full command context (CMD type, limits,\n";
    f << "#   initial conditions, peak values) for root cause analysis.\n";
    f << "#   'settled=no' means the value was inherited from a previous command.\n\n";

    /* anomaly log */
    f << "# ── Events ──\n";
    for (auto &e : entries) {
        f << e.msg << "\n\n";
    }

    /* plot data: time, pos, vel */
    f << "\n# ── Plot Data ──\n";
    f << "# time(s)\tposition\tvelocity\tacceleration\tjerk\n";
    /* snapshot under lock already taken by caller — but we receive copies */
    {
        std::lock_guard<std::mutex> lk(const_cast<PlotData &>(plot).mtx);
        int n = (int)plot.time.size();
        for (int i = 0; i < n; i++) {
            f << plot.time[i] << "\t"
              << plot.position[i] << "\t"
              << plot.velocity[i] << "\t"
              << plot.acceleration[i] << "\t"
              << plot.jerk_plot[i] << "\n";
        }
    }

    f.close();
    return fname;
}

/* ── Axis state name ────────────────────────────────────────────────────── */
static const char *axis_state_name(MC_AXIS_STATE s)
{
    switch (s) {
    case MC_AXIS_DISABLED:            return "Disabled";
    case MC_AXIS_STANDSTILL:          return "Standstill";
    case MC_AXIS_HOMING:              return "Homing";
    case MC_AXIS_STOPPING:            return "Stopping";
    case MC_AXIS_DISCRETE_MOTION:     return "Discrete Motion";
    case MC_AXIS_CONTINUOUS_MOTION:   return "Continuous Motion";
    case MC_AXIS_SYNCHRONIZED_MOTION: return "Synchronized Motion";
    case MC_AXIS_ERRORSTOP:           return "Error Stop";
    default:                          return "Unknown";
    }
}

/* ── Helper: draw horizontal reference line ─────────────────────────────── */
static void plot_hline(const char *label, float y, float t_min, float t_max)
{
    float xs[2] = {t_min, t_max};
    float ys[2] = {y, y};
    ImPlot::PlotLine(label, xs, ys, 2);
}

/* ── Main ───────────────────────────────────────────────────────────────── */
int main()
{
    /* ── Init GLFW ──────────────────────────────────────────────────────── */
    if (!glfwInit()) {
        fprintf(stderr, "Failed to init GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow *window = glfwCreateWindow(1600, 900, "KronMotion Analysis Tool", nullptr, nullptr);
    if (!window) {
        glfwDefaultWindowHints();
        window = glfwCreateWindow(1600, 900, "KronMotion Analysis Tool", nullptr, nullptr);
        if (!window) {
            fprintf(stderr, "Failed to create window\n");
            glfwTerminate();
            return 1;
        }
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    /* ── Init ImGui + ImPlot ────────────────────────────────────────────── */
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    /* ── Init KronMotion - bypass MC layer entirely ─────────────────────── */
    memset(&Kron_PI, 0, sizeof(Kron_PI));

    AXIS_REF axis_ref;
    AXIS_REF_Init(&axis_ref, 0, nullptr);
    axis_ref.Simulation = true;
    axis_ref.VelFactor  = 1.0f;
    axis_ref.AccFactor  = 1.0f;
    axis_ref.JerkFactor = 1.0f;

    NC_AXIS nc_axis;
    NC_Init(&nc_axis, &axis_ref);

    /* Force into ready state - skip power/home ceremony */
    nc_axis.priv.power_requested = true;
    nc_axis.priv.op_enabled      = true;
    axis_ref.sts_State           = MC_AXIS_STANDSTILL;
    axis_ref.IsHomed             = true;

    /* ── Data structures ────────────────────────────────────────────────── */
    PlotData plot;
    plot.time.reserve(MAX_POINTS);
    plot.position.reserve(MAX_POINTS);
    plot.velocity.reserve(MAX_POINTS);
    plot.acceleration.reserve(MAX_POINTS);
    plot.jerk_plot.reserve(MAX_POINTS);

    std::deque<LogEntry> log_entries;
    std::mutex log_mtx;
    static constexpr int MAX_LOG = 500;

    /* ── Motion parameters ──────────────────────────────────────────────── */
    float param_position     = 100.0f;
    float param_velocity     = 50.0f;
    float param_acceleration = 200.0f;
    float param_deceleration = 200.0f;
    float param_jerk         = 1000.0f;
    float param_vel_target   = 30.0f;

    float nc_dt = 0.001f;
    float plot_window = 10.0f;

    /* ── Analysis params ────────────────────────────────────────────────── */
    AnalysisParams analysis = {
        .vel_disc_threshold  = 0.5f,
        .acc_disc_threshold  = 5.0f,
        .vel_overshoot_pct   = 1.0f,
        .acc_overshoot_pct   = 1.0f,
        .pos_overshoot_abs   = 0.01f,
    };

    /* ── Random params ──────────────────────────────────────────────────── */
    RandomParams rparams = {
        .pos_min = -200.0f, .pos_max = 200.0f,
        .vel_min = 10.0f,   .vel_max = 100.0f,
        .acc_min = 50.0f,   .acc_max = 500.0f,
        .dec_min = 50.0f,   .dec_max = 500.0f,
        .jerk_min = 0.0f,    .jerk_max = 5000.0f,
        .interval_min = 0.5f, .interval_max = 3.0f,
    };

    /* ── Clear old log files ──────────────────────────────────────────── */
    if (std::filesystem::exists("tool")) {
        for (auto &entry : std::filesystem::directory_iterator("tool")) {
            if (entry.path().extension() == ".log")
                std::filesystem::remove(entry.path());
        }
    }

    /* ── Start threads ──────────────────────────────────────────────────── */
    std::thread nc_thread(nc_thread_func, &nc_axis, &plot, nc_dt,
                          &log_entries, &log_mtx, &analysis);
    std::thread rand_thread(random_thread_func, &axis_ref, &rparams,
                            &plot, &log_entries, &log_mtx);

    (void)0;

    /* ── Main loop ──────────────────────────────────────────────────────── */
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int win_w, win_h;
        glfwGetWindowSize(window, &win_w, &win_h);
        float panel_w = 400.0f;

        /* ================================================================
         * LEFT PANEL
         * ================================================================ */
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(panel_w, (float)win_h));
        ImGui::Begin("Control", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);

        /* ── Live Status ────────────────────────────────────────────────── */
        ImGui::SeparatorText("Status");
        ImGui::Text("State:    %s", axis_state_name(axis_ref.sts_State));
        ImGui::Text("Position: %.4f", axis_ref.CommandedPosition);
        ImGui::Text("Velocity: %.4f", axis_ref.CommandedVelocity);
        ImGui::Text("Accel:    %.4f", nc_axis.priv.cmd_acc);

        /* ── Active NC Parameters (latched by NC engine, updated by random) */
        ImGui::SeparatorText("Active Profile");
        ImGui::Text("Target:   %.2f", nc_axis.priv.target_pos);
        ImGui::Text("V_max:    %.2f", nc_axis.priv.v_max);
        ImGui::Text("Acc:      %.2f", nc_axis.priv.acc);
        ImGui::Text("Dec:      %.2f", nc_axis.priv.dec);
        ImGui::Text("Jerk:     %.0f", nc_axis.priv.jerk);

        /* ── Parameters ─────────────────────────────────────────────────── */
        ImGui::SeparatorText("Manual Parameters");
        ImGui::InputFloat("Position",     &param_position,     1.0f, 10.0f, "%.1f");
        ImGui::InputFloat("Velocity",     &param_velocity,     1.0f, 10.0f, "%.1f");
        ImGui::InputFloat("Acceleration", &param_acceleration, 10.0f, 100.0f, "%.1f");
        ImGui::InputFloat("Deceleration", &param_deceleration, 10.0f, 100.0f, "%.1f");
        ImGui::InputFloat("Jerk",         &param_jerk, 100.0f, 1000.0f, "%.0f");
        ImGui::InputFloat("Vel Target",   &param_vel_target,   1.0f, 10.0f, "%.1f");

        /* ── Manual Commands ────────────────────────────────────────────── */
        ImGui::SeparatorText("Commands");
        float btn_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.0f;

        if (ImGui::Button("Move Absolute", ImVec2(btn_w, 32))) {
            g_target_pos.store(param_position);
            g_max_vel.store(param_velocity);
            g_max_acc.store(param_acceleration);
            g_max_dec.store(param_deceleration);
            publish_cmd(&axis_ref, NC_CMD_MOVE_ABS,
                        param_position, param_velocity,
                        param_acceleration, param_deceleration, param_jerk);
        }
        ImGui::SameLine();
        if (ImGui::Button("Move Relative", ImVec2(btn_w, 32))) {
            float target = axis_ref.CommandedPosition + param_position;
            g_target_pos.store(target);
            g_max_vel.store(param_velocity);
            g_max_acc.store(param_acceleration);
            g_max_dec.store(param_deceleration);
            publish_cmd(&axis_ref, NC_CMD_MOVE_REL,
                        param_position, param_velocity,
                        param_acceleration, param_deceleration, param_jerk);
        }

        if (ImGui::Button("Move Velocity", ImVec2(btn_w, 32))) {
            g_max_vel.store(std::fabs(param_vel_target));
            g_max_acc.store(param_acceleration);
            g_max_dec.store(param_deceleration);
            publish_cmd(&axis_ref, NC_CMD_MOVE_VEL,
                        0.0f, param_vel_target,
                        param_acceleration, param_deceleration, param_jerk);
        }
        ImGui::SameLine();
        if (ImGui::Button("Halt", ImVec2(btn_w, 32))) {
            publish_cmd(&axis_ref, NC_CMD_HALT,
                        0.0f, 0.0f,
                        param_acceleration, param_deceleration, param_jerk);
        }

        /* ── Random Excitation ──────────────────────────────────────────── */
        ImGui::SeparatorText("Random Excitation");

        bool rp = g_random_pos.load();
        bool rv = g_random_vel.load();

        if (rp) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
        if (ImGui::Button(rp ? "Random Pos: ON" : "Random Pos: OFF", ImVec2(btn_w, 32))) {
            if (rp) {
                /* turning OFF -> save log */
                std::lock_guard<std::mutex> lk(log_mtx);
                std::string path = save_log(log_entries, plot, "tool");
                if (!path.empty())
                    log_entries.push_back({plot.t_elapsed, 0, 0,
                        "Saved: " + path, ImVec4(0.3f, 1, 0.3f, 1)});
            }
            g_random_pos.store(!rp);
        }
        if (rp) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (rv) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
        if (ImGui::Button(rv ? "Random Vel: ON" : "Random Vel: OFF", ImVec2(btn_w, 32))) {
            if (rv) {
                /* turning OFF -> save log */
                std::lock_guard<std::mutex> lk(log_mtx);
                std::string path = save_log(log_entries, plot, "tool");
                if (!path.empty())
                    log_entries.push_back({plot.t_elapsed, 0, 0,
                        "Saved: " + path, ImVec4(0.3f, 1, 0.3f, 1)});
            }
            g_random_vel.store(!rv);
        }
        if (rv) ImGui::PopStyleColor();

        if (ImGui::TreeNode("Random Settings")) {
            ImGui::DragFloatRange2("Pos Range", &rparams.pos_min, &rparams.pos_max, 1.0f, -1000, 1000);
            ImGui::DragFloatRange2("Vel Range", &rparams.vel_min, &rparams.vel_max, 1.0f, 0, 1000);
            ImGui::DragFloatRange2("Acc Range", &rparams.acc_min, &rparams.acc_max, 1.0f, 0, 5000);
            ImGui::DragFloatRange2("Dec Range", &rparams.dec_min, &rparams.dec_max, 1.0f, 0, 5000);
            ImGui::DragFloatRange2("Jerk Range", &rparams.jerk_min, &rparams.jerk_max, 10.0f, 0, 50000);
            ImGui::DragFloatRange2("Interval (s)", &rparams.interval_min, &rparams.interval_max, 0.1f, 0.1f, 30.0f);
            ImGui::TreePop();
        }

        /* ── Analysis Thresholds ────────────────────────────────────────── */
        ImGui::SeparatorText("Analysis");
        if (ImGui::TreeNode("Thresholds")) {
            ImGui::InputFloat("Vel disc.", &analysis.vel_disc_threshold, 0.01f, 0.1f, "%.3f");
            ImGui::InputFloat("Acc disc.", &analysis.acc_disc_threshold, 0.1f, 1.0f, "%.2f");
            ImGui::InputFloat("Vel overshoot %%", &analysis.vel_overshoot_pct, 0.1f, 1.0f, "%.1f");
            ImGui::InputFloat("Acc overshoot %%", &analysis.acc_overshoot_pct, 0.1f, 1.0f, "%.1f");
            ImGui::InputFloat("Pos overshoot [u]", &analysis.pos_overshoot_abs, 0.001f, 0.01f, "%.4f");
            float ptol = g_pos_tolerance.load();
            if (ImGui::InputFloat("Pos settle tol", &ptol, 0.001f, 0.01f, "%.4f"))
                g_pos_tolerance.store(ptol);
            ImGui::TreePop();
        }

        /* ── Plot Settings ──────────────────────────────────────────────── */
        ImGui::SeparatorText("Plot");
        ImGui::SliderFloat("Window (s)", &plot_window, 1.0f, 60.0f, "%.0f");
        if (ImGui::Button("Clear Plot & Log", ImVec2(-1, 0))) {
            plot.clear();
            std::lock_guard<std::mutex> lk(log_mtx);
            log_entries.clear();
        }

        /* ── Log ────────────────────────────────────────────────────────── */
        ImGui::SeparatorText("Log");
        {
            float log_h = ImGui::GetContentRegionAvail().y;
            ImGui::BeginChild("LogScroll", ImVec2(0, log_h), ImGuiChildFlags_Borders);
            std::lock_guard<std::mutex> lk(log_mtx);
            /* trim old */
            while ((int)log_entries.size() > MAX_LOG)
                log_entries.pop_front();
            for (auto &e : log_entries) {
                ImGui::PushStyleColor(ImGuiCol_Text, e.color);
                ImGui::TextWrapped("%s", e.msg.c_str());
                ImGui::PopStyleColor();
            }
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f)
                ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
        }

        ImGui::End();

        /* ================================================================
         * RIGHT PANEL - PLOTS
         * ================================================================ */
        ImGui::SetNextWindowPos(ImVec2(panel_w, 0));
        ImGui::SetNextWindowSize(ImVec2((float)win_w - panel_w, (float)win_h));
        ImGui::Begin("Plots", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);

        float plot_h = (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y) / 2.0f;
        float plot_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.0f;

        /* snapshot */
        std::vector<float> t_s, p_s, v_s, a_s, j_s;
        {
            std::lock_guard<std::mutex> lk(plot.mtx);
            t_s = plot.time;
            p_s = plot.position;
            v_s = plot.velocity;
            a_s = plot.acceleration;
            j_s = plot.jerk_plot;
        }

        float t_now = plot.t_elapsed;
        float t_min = t_now - plot_window;
        if (t_min < 0.0f) t_min = 0.0f;
        float t_max = t_now;
        if (t_max < plot_window) t_max = plot_window;

        int n = (int)t_s.size();

        /* Find visible range index bounds */
        int i_start = 0, i_end = n;
        for (int i = 0; i < n; i++) {
            if (t_s[i] >= t_min) { i_start = i; break; }
        }
        for (int i = n - 1; i >= 0; i--) {
            if (t_s[i] <= t_max) { i_end = i + 1; break; }
        }

        /* Compute min/max for visible data + reference lines, with 10% margin */
        auto calc_limits = [&](const std::vector<float> &data,
                               float ref_lo, float ref_hi,
                               float &out_min, float &out_max)
        {
            out_min = ref_lo;
            out_max = ref_hi;
            for (int i = i_start; i < i_end; i++) {
                if (data[i] < out_min) out_min = data[i];
                if (data[i] > out_max) out_max = data[i];
            }
            float range = out_max - out_min;
            if (range < 1e-6f) range = 1.0f;
            float margin = range * 0.1f;
            out_min -= margin;
            out_max += margin;
        };

        float y_lo, y_hi;
        float tgt = g_target_pos.load();
        float mv  = g_max_vel.load();
        float ma  = g_max_acc.load();

        /* ── Row 1: Position | Acceleration ─────────────────────────────── */
        calc_limits(p_s, std::fmin(0.0f, tgt), std::fmax(0.0f, tgt), y_lo, y_hi);
        if (ImPlot::BeginPlot("Position", ImVec2(plot_w, plot_h))) {
            ImPlot::SetupAxes("Time (s)", "Position (u)");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImPlotCond_Always);
            if (n > 0)
                ImPlot::PlotLine("Cmd Pos", t_s.data(), p_s.data(), n);
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 0.3f, 0.3f, 0.6f));
            plot_hline("Target", tgt, t_min, t_max);
            ImPlot::PopStyleColor();
            ImPlot::EndPlot();
        }

        ImGui::SameLine();

        calc_limits(a_s, -ma, ma, y_lo, y_hi);
        if (ImPlot::BeginPlot("Acceleration", ImVec2(plot_w, plot_h))) {
            ImPlot::SetupAxes("Time (s)", "Accel (u/s^2)");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImPlotCond_Always);
            if (n > 0)
                ImPlot::PlotLine("Cmd Acc", t_s.data(), a_s.data(), n);
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 0.3f, 0.3f, 0.6f));
            plot_hline("+Amax", ma, t_min, t_max);
            plot_hline("-Amax", -ma, t_min, t_max);
            ImPlot::PopStyleColor();
            ImPlot::EndPlot();
        }

        /* ── Row 2: Velocity | Jerk ─────────────────────────────────────── */
        calc_limits(v_s, -mv, mv, y_lo, y_hi);
        if (ImPlot::BeginPlot("Velocity", ImVec2(plot_w, plot_h))) {
            ImPlot::SetupAxes("Time (s)", "Velocity (u/s)");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImPlotCond_Always);
            if (n > 0)
                ImPlot::PlotLine("Cmd Vel", t_s.data(), v_s.data(), n);
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 0.3f, 0.3f, 0.6f));
            plot_hline("+Vmax", mv, t_min, t_max);
            plot_hline("-Vmax", -mv, t_min, t_max);
            ImPlot::PopStyleColor();
            ImPlot::EndPlot();
        }

        ImGui::SameLine();

        calc_limits(j_s, 0.0f, 0.0f, y_lo, y_hi);
        if (ImPlot::BeginPlot("Jerk", ImVec2(plot_w, plot_h))) {
            ImPlot::SetupAxes("Time (s)", "Jerk (u/s^3)");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImPlotCond_Always);
            if (n > 0)
                ImPlot::PlotLine("Num Jerk", t_s.data(), j_s.data(), n);
            ImPlot::EndPlot();
        }

        ImGui::End();

        /* ── Render ─────────────────────────────────────────────────────── */
        ImGui::Render();
        int fb_w, fb_h;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.08f, 0.08f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    /* ── Cleanup ────────────────────────────────────────────────────────── */
    g_running.store(false);
    g_random_pos.store(false);
    g_random_vel.store(false);
    nc_thread.join();
    rand_thread.join();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
