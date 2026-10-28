/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * xpbench -- X-Plane plugin half of xplane-bench.
 *
 * Everything here is exposed as datarefs and commands under "xpbench/", so
 * the driver reaches it through X-Plane's own web API and needs no other
 * channel. The plugin does only what has to happen inside X-Plane's frame
 * loop or needs the XPLM SDK:
 *
 *   fms     load / save an X-Plane .fms (v1100) flight plan, including
 *           procedures (XPLMLoadFMSFlightPlan), read and set the active entry
 *   speed   hold an indicated airspeed on the throttle. The aircraft we fly
 *           (SR22, C172) have no autothrottle, and altitude hold without a
 *           speed loop stalls the aircraft (SIM0, 2026-10-28)
 *   guard   pause the sim and record why, the moment the airspeed goes below
 *           a floor or the autopilot servos drop -- instead of letting
 *           X-Plane crash and silently reset the flight
 *   log     write the truth log (X-Plane's own position and navigator) as
 *           JSON lines at 5 Hz
 *   state   one float array with what the driver polls, so it is one HTTP
 *           call per poll, not twenty
 */

#define _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "XPLMDataAccess.h"
#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#define XPBENCH_VERSION 1
#define PLAN_MAX 32768
#define PATH_MAX_LEN 1024
#define IDS_MAX 8192
#define FPL xplm_Fpl_Pilot_Primary
#define FPL_APP xplm_Fpl_Pilot_Approach
/* which flight plan xpbench/fms/dest and /direct_to act on: 0 primary, 2 approach */
static int fpl_sel = xplm_Fpl_Pilot_Primary;

/* ---- sim datarefs we read or write -------------------------------------- */

static XPLMDataRef d_lat, d_lon, d_elev, d_ias, d_gs, d_trk, d_hdg_mag, d_roll,
    d_pitch, d_vs, d_paused, d_throttle, d_mixture, d_prop, d_servos,
    d_ap_mode, d_hmode, d_alt_status, d_gps_xtk, d_gps_crs, d_gps_hdef,
    d_gps_fromto, d_gps_dme, d_gps_id, d_on_ground, d_net_time, d_wind_dir, d_wind_kt;
static XPLMCommandRef c_pause, c_hdg_up, c_hdg_down;
static XPLMDataRef d_hdg_dial;

/* ---- our own state ------------------------------------------------------ */

static char plan_in[PLAN_MAX];
static int plan_in_len = 0;
static char plan_out[PLAN_MAX];
static int plan_out_len = 0;
static char entry_ids[IDS_MAX];
static int entry_ids_len = 0;

static float speed_target = 0.0f; /* KIAS, <= 0 = off */
static float speed_integ = 0.0f;
static float guard_min_kias = 0.0f; /* <= 0 = off */
static int guard_watch_servos = 0;
static int guard_tripped = 0; /* 0 none, 1 low speed, 2 servos dropped, 3 on ground */
static int guard_watch_ground = 0;
/* heading bug target (deg mag), < 0 = off. The bug is walked there with the
 * cockpit's own heading_up/heading_down commands, never written directly:
 * a direct write can make X-Plane's HDG mode turn the long way round
 * (seen twice, 2026-10-28), while stepping always turns the way it moves. */
static float bug_target = -1.0f;

static char log_path[PATH_MAX_LEN];
static int log_path_len = 0;
static FILE *log_fp = NULL;
static float log_accum = 0.0f;

static XPLMDataRef our_refs[32];
static int n_our_refs = 0;
static XPLMCommandRef c_load, c_save, c_log_start, c_log_stop, c_guard_reset;

/* ---- helpers ------------------------------------------------------------ */

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int append_plan(int pos, int plan, const char *tag)
{
    int n = XPLMCountFMSFlightPlanEntries(plan);
    pos += snprintf(entry_ids + pos, IDS_MAX - pos, "%s%s", pos ? ";" : "", tag);
    for (int i = 0; i < n && pos < IDS_MAX - 300; i++) {
        char id[256] = {0};
        XPLMNavType type = xplm_Nav_Unknown;
        XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
        int alt = 0;
        float lat = 0, lon = 0;
        XPLMGetFMSFlightPlanEntryInfo(plan, i, &type, id, &ref, &alt, &lat, &lon);
        pos += snprintf(entry_ids + pos, IDS_MAX - pos, "%s%d:%s:%d:%.6f:%.6f",
                        i ? "," : "|", (int)type, id, alt, lat, lon);
    }
    return pos;
}

/* "P|type:id:alt:lat:lon,...;A|..." -- the primary and the approach plans */
static void refresh_entry_ids(void)
{
    int pos = 0;
    entry_ids[0] = 0;
    pos = append_plan(pos, FPL, "P");
    pos = append_plan(pos, FPL_APP, "A");
    entry_ids_len = pos;
}

/* byte-array accessors: one pair per buffer */
static int get_bytes(const char *buf, int len, void *out, int offset, int max)
{
    if (out == NULL) return len;
    if (offset >= len) return 0;
    int n = len - offset < max ? len - offset : max;
    memcpy(out, buf + offset, n);
    return n;
}

static int r_plan_in(void *ref, void *out, int off, int max) { return get_bytes(plan_in, plan_in_len, out, off, max); }
static void w_plan_in(void *ref, void *in, int off, int n)
{
    /* the web API writes the whole value at offset 0: treat that as replace */
    if (off == 0) plan_in_len = 0;
    if (off + n >= PLAN_MAX) n = PLAN_MAX - 1 - off;
    if (n <= 0) return;
    memcpy(plan_in + off, in, n);
    if (off + n > plan_in_len) plan_in_len = off + n;
    plan_in[plan_in_len] = 0;
}
static int r_plan_out(void *ref, void *out, int off, int max) { return get_bytes(plan_out, plan_out_len, out, off, max); }
static int r_entry_ids(void *ref, void *out, int off, int max)
{
    refresh_entry_ids();
    return get_bytes(entry_ids, entry_ids_len, out, off, max);
}
static int r_log_path(void *ref, void *out, int off, int max) { return get_bytes(log_path, log_path_len, out, off, max); }
static void w_log_path(void *ref, void *in, int off, int n)
{
    if (off == 0) log_path_len = 0;
    if (off + n >= PATH_MAX_LEN) n = PATH_MAX_LEN - 1 - off;
    if (n <= 0) return;
    memcpy(log_path + off, in, n);
    if (off + n > log_path_len) log_path_len = off + n;
    log_path[log_path_len] = 0;
}

static int r_version(void *ref) { return XPBENCH_VERSION; }
static int r_count(void *ref) { return XPLMCountFMSFlightPlanEntries(fpl_sel); }
static int r_dest(void *ref) { return XPLMGetDestinationFMSFlightPlanEntry(fpl_sel); }
static void w_dest(void *ref, int v) { XPLMSetDestinationFMSFlightPlanEntry(fpl_sel, v); }
static int r_direct(void *ref) { return XPLMGetDestinationFMSFlightPlanEntry(fpl_sel); }
static void w_direct(void *ref, int v) { XPLMSetDirectToFMSFlightPlanEntry(fpl_sel, v); }
static int r_sel(void *ref) { return fpl_sel; }
static void w_sel(void *ref, int v) { fpl_sel = v; }
static float r_speed(void *ref) { return speed_target; }
static void w_speed(void *ref, float v) { speed_target = v; speed_integ = 0.0f; }
static float r_gmin(void *ref) { return guard_min_kias; }
static void w_gmin(void *ref, float v) { guard_min_kias = v; }
static int r_gservo(void *ref) { return guard_watch_servos; }
static void w_gservo(void *ref, int v) { guard_watch_servos = v; }
static int r_gground(void *ref) { return guard_watch_ground; }
static void w_gground(void *ref, int v) { guard_watch_ground = v; }
static int r_tripped(void *ref) { return guard_tripped; }
static float r_bug(void *ref) { return bug_target; }
static void w_bug(void *ref, float v) { bug_target = v < 0.0f ? -1.0f : fmodf(v, 360.0f); }

/* state vector -- keep the order in sync with driver/xpbench/xplane.py STATE */
#define N_STATE 22
static int r_state(void *ref, float *out, int off, int max)
{
    if (out == NULL) return N_STATE;
    char id[64] = {0};
    float s[N_STATE];
    s[0] = (float)XPLMGetDatad(d_lat);
    s[1] = (float)XPLMGetDatad(d_lon);
    s[2] = (float)(XPLMGetDatad(d_elev) * 3.28084);
    s[3] = XPLMGetDataf(d_ias);
    s[4] = XPLMGetDataf(d_gs) * 1.943844f;
    s[5] = XPLMGetDataf(d_trk);
    s[6] = XPLMGetDataf(d_hdg_mag);
    s[7] = XPLMGetDataf(d_gps_xtk);
    s[8] = XPLMGetDataf(d_gps_crs);
    s[9] = (float)XPLMGetDestinationFMSFlightPlanEntry(FPL);
    s[10] = (float)XPLMGetDatai(d_servos);
    s[11] = (float)XPLMGetDatai(d_hmode);
    s[12] = (float)XPLMGetDatai(d_alt_status);
    s[13] = (float)XPLMGetDatai(d_paused);
    s[14] = (float)guard_tripped;
    s[15] = XPLMGetDataf(d_roll);
    s[16] = XPLMGetDataf(d_vs);
    s[17] = XPLMGetDataf(d_gps_dme);
    s[18] = XPLMGetDataf(d_gps_hdef);
    s[19] = (float)XPLMGetDatai(d_gps_fromto);
    s[20] = (float)XPLMGetDestinationFMSFlightPlanEntry(FPL_APP);
    s[21] = (float)XPLMCountFMSFlightPlanEntries(FPL_APP);
    (void)id;
    int n = 0;
    for (int i = off; i < N_STATE && n < max; i++) out[n++] = s[i];
    return n;
}

/* ---- commands ----------------------------------------------------------- */

static int cmd_cb(XPLMCommandRef cmd, XPLMCommandPhase phase, void *ref)
{
    if (phase != xplm_CommandBegin) return 0;
    if (cmd == c_load) {
        XPLMLoadFMSFlightPlan(0, plan_in, (unsigned int)plan_in_len);
        XPLMDebugString("xpbench: flight plan loaded\n");
    } else if (cmd == c_save) {
        unsigned int need = XPLMSaveFMSFlightPlan(0, plan_out, PLAN_MAX);
        plan_out_len = need > PLAN_MAX ? PLAN_MAX : (need ? (int)need - 1 : 0);
    } else if (cmd == c_log_start) {
        if (log_fp) fclose(log_fp);
        log_fp = log_path_len ? fopen(log_path, "a") : NULL;
        log_accum = 0.0f;
        XPLMDebugString(log_fp ? "xpbench: truth log started\n" : "xpbench: truth log FAILED to open\n");
    } else if (cmd == c_log_stop) {
        if (log_fp) fclose(log_fp);
        log_fp = NULL;
    } else if (cmd == c_guard_reset) {
        guard_tripped = 0;
    }
    return 0;
}

/* ---- frame loop --------------------------------------------------------- */

static void write_log_row(void)
{
    char id[64] = {0};
    XPLMGetDatab(d_gps_id, id, 0, sizeof(id) - 1);
    fprintf(log_fp,
            "{\"t\":%.2f,\"lat\":%.7f,\"lon\":%.7f,\"alt_ft\":%.1f,\"ias\":%.1f,"
            "\"gs_kt\":%.1f,\"trk\":%.2f,\"hdg_mag\":%.2f,\"roll\":%.1f,\"pitch\":%.1f,"
            "\"vs_fpm\":%.0f,\"gps_id\":\"%s\",\"gps_crs_mag\":%.3f,\"gps_xtk_nm\":%.4f,"
            "\"gps_hdef_dot\":%.4f,\"gps_fromto\":%d,\"gps_dme_nm\":%.4f,\"dest\":%d,\"app_dest\":%d,"
            "\"servos\":%d,\"hmode\":%d,\"alt_status\":%d,\"thr\":%.3f,\"paused\":%d,"
            "\"tripped\":%d,\"wind_dir\":%.0f,\"wind_kt\":%.1f}\n",
            XPLMGetDataf(d_net_time), XPLMGetDatad(d_lat), XPLMGetDatad(d_lon),
            XPLMGetDatad(d_elev) * 3.28084, XPLMGetDataf(d_ias), XPLMGetDataf(d_gs) * 1.943844f,
            XPLMGetDataf(d_trk), XPLMGetDataf(d_hdg_mag), XPLMGetDataf(d_roll),
            XPLMGetDataf(d_pitch), XPLMGetDataf(d_vs), id, XPLMGetDataf(d_gps_crs),
            XPLMGetDataf(d_gps_xtk), XPLMGetDataf(d_gps_hdef), XPLMGetDatai(d_gps_fromto),
            XPLMGetDataf(d_gps_dme), XPLMGetDestinationFMSFlightPlanEntry(FPL),
            XPLMGetDestinationFMSFlightPlanEntry(FPL_APP),
            XPLMGetDatai(d_servos), XPLMGetDatai(d_hmode), XPLMGetDatai(d_alt_status),
            XPLMGetDataf(d_throttle), XPLMGetDatai(d_paused), guard_tripped,
            d_wind_dir ? XPLMGetDataf(d_wind_dir) : -1.0f, d_wind_kt ? XPLMGetDataf(d_wind_kt) : -1.0f);
    fflush(log_fp);
}

static void trip(int why, const char *msg)
{
    if (guard_tripped) return;
    guard_tripped = why;
    if (!XPLMGetDatai(d_paused)) XPLMCommandOnce(c_pause);
    XPLMDebugString(msg);
}

static float flight_loop(float since_last, float since_loop, int counter, void *ref)
{
    if (XPLMGetDatai(d_paused)) return -1.0f;
    float dt = since_last > 0.0f && since_last < 1.0f ? since_last : 0.05f;
    float ias = XPLMGetDataf(d_ias);

    /* speed on throttle: PI with anti-windup; mixture and prop stay full */
    if (speed_target > 0.0f) {
        float err = speed_target - ias;
        speed_integ = clampf(speed_integ + err * dt * 0.02f, -0.6f, 0.6f);
        float thr = clampf(0.55f + 0.04f * err + speed_integ, 0.05f, 1.0f);
        XPLMSetDataf(d_throttle, thr);
        XPLMSetDataf(d_mixture, 1.0f);
        XPLMSetDataf(d_prop, 1.0f);
    }

    /* heading bug: up to 10 one-degree steps per frame toward the target */
    if (bug_target >= 0.0f) {
        for (int i = 0; i < 10; i++) {
            float cur = XPLMGetDataf(d_hdg_dial);
            float diff = fmodf(bug_target - cur + 540.0f, 360.0f) - 180.0f;
            if (fabsf(diff) < 0.75f) break;
            XPLMCommandOnce(diff > 0.0f ? c_hdg_up : c_hdg_down);
        }
    }

    /* guard */
    if (guard_min_kias > 0.0f && ias < guard_min_kias)
        trip(1, "xpbench: GUARD low airspeed -- paused\n");
    if (guard_watch_servos && XPLMGetDatai(d_servos) == 0)
        trip(2, "xpbench: GUARD autopilot servos dropped -- paused\n");
    if (guard_watch_ground && XPLMGetDatai(d_on_ground))
        trip(3, "xpbench: GUARD on ground -- paused\n");

    /* truth log at 5 Hz */
    if (log_fp) {
        log_accum += dt;
        if (log_accum >= 0.2f) {
            log_accum = 0.0f;
            write_log_row();
        }
    }
    return -1.0f;
}

/* ---- registration ------------------------------------------------------- */

static void reg(XPLMDataRef r) { if (n_our_refs < 32) our_refs[n_our_refs++] = r; }

/* XPLMRegisterDataAccessor takes, after name/type/writable, seven get/set
 * pairs in this order -- int, float, double, int[], float[], bytes -- then
 * the two refcons. */
static XPLMDataRef reg_bytes(const char *name, XPLMGetDatab_f r, XPLMSetDatab_f w)
{
    XPLMDataRef d = XPLMRegisterDataAccessor(name, xplmType_Data, w != NULL,
                                             NULL, NULL, NULL, NULL, NULL, NULL,
                                             NULL, NULL, NULL, NULL, r, w, NULL, NULL);
    reg(d);
    return d;
}
static XPLMDataRef reg_int(const char *name, XPLMGetDatai_f r, XPLMSetDatai_f w)
{
    XPLMDataRef d = XPLMRegisterDataAccessor(name, xplmType_Int, w != NULL,
                                             r, w, NULL, NULL, NULL, NULL,
                                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    reg(d);
    return d;
}
static XPLMDataRef reg_float(const char *name, XPLMGetDataf_f r, XPLMSetDataf_f w)
{
    XPLMDataRef d = XPLMRegisterDataAccessor(name, xplmType_Float, w != NULL,
                                             NULL, NULL, r, w, NULL, NULL,
                                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    reg(d);
    return d;
}

static XPLMCommandRef mk_cmd(const char *name, const char *desc)
{
    XPLMCommandRef c = XPLMCreateCommand(name, desc);
    XPLMRegisterCommandHandler(c, cmd_cb, 1, NULL);
    return c;
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strcpy(outName, "xpbench");
    strcpy(outSig, "org.aerocommons.xpbench");
    strcpy(outDesc, "xplane-bench: FMS load, speed hold, guard and truth log for pilotless test flights.");

    d_lat = XPLMFindDataRef("sim/flightmodel/position/latitude");
    d_lon = XPLMFindDataRef("sim/flightmodel/position/longitude");
    d_elev = XPLMFindDataRef("sim/flightmodel/position/elevation");
    d_ias = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
    d_gs = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    d_trk = XPLMFindDataRef("sim/flightmodel/position/hpath");
    d_hdg_mag = XPLMFindDataRef("sim/flightmodel/position/mag_psi");
    d_roll = XPLMFindDataRef("sim/flightmodel/position/phi");
    d_pitch = XPLMFindDataRef("sim/flightmodel/position/theta");
    d_vs = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm");
    d_paused = XPLMFindDataRef("sim/time/paused");
    d_throttle = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
    d_mixture = XPLMFindDataRef("sim/cockpit2/engine/actuators/mixture_ratio_all");
    d_prop = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio_all");
    d_servos = XPLMFindDataRef("sim/cockpit2/autopilot/servos_on");
    d_ap_mode = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");
    d_hmode = XPLMFindDataRef("sim/cockpit2/autopilot/heading_mode");
    d_alt_status = XPLMFindDataRef("sim/cockpit2/autopilot/altitude_hold_status");
    d_gps_xtk = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_xtk");
    d_gps_crs = XPLMFindDataRef("sim/cockpit/radios/gps_course_degtm");
    d_gps_hdef = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_hdef_dots_pilot");
    d_gps_fromto = XPLMFindDataRef("sim/cockpit/radios/gps_fromto");
    d_gps_dme = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_dme_distance_nm");
    d_gps_id = XPLMFindDataRef("sim/cockpit2/radios/indicators/gps_nav_id");
    d_on_ground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    d_net_time = XPLMFindDataRef("sim/network/misc/network_time_sec");
    d_wind_dir = XPLMFindDataRef("sim/weather/aircraft/wind_now_direction_degt");
    d_wind_kt = XPLMFindDataRef("sim/cockpit2/gauges/indicators/wind_speed_kts");
    c_pause = XPLMFindCommand("sim/operation/pause_toggle");
    c_hdg_up = XPLMFindCommand("sim/autopilot/heading_up");
    c_hdg_down = XPLMFindCommand("sim/autopilot/heading_down");
    d_hdg_dial = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot");

    reg_int("xpbench/version", r_version, NULL);
    reg_bytes("xpbench/fms/plan_in", r_plan_in, w_plan_in);
    reg_bytes("xpbench/fms/plan_out", r_plan_out, NULL);
    reg_bytes("xpbench/fms/entries", r_entry_ids, NULL);
    reg_int("xpbench/fms/count", r_count, NULL);
    reg_int("xpbench/fms/dest", r_dest, w_dest);
    reg_int("xpbench/fms/direct_to", r_direct, w_direct);
    reg_int("xpbench/fms/plan_sel", r_sel, w_sel);
    reg_float("xpbench/speed/target_kias", r_speed, w_speed);
    reg_float("xpbench/guard/min_kias", r_gmin, w_gmin);
    reg_int("xpbench/guard/watch_servos", r_gservo, w_gservo);
    reg_int("xpbench/guard/watch_ground", r_gground, w_gground);
    reg_int("xpbench/guard/tripped", r_tripped, NULL);
    reg_float("xpbench/ap/bug_target", r_bug, w_bug);
    reg_bytes("xpbench/log/path", r_log_path, w_log_path);
    reg(XPLMRegisterDataAccessor("xpbench/state", xplmType_FloatArray, 0,
                                 NULL, NULL, NULL, NULL, NULL, NULL,
                                 NULL, NULL, r_state, NULL, NULL, NULL, NULL, NULL));

    c_load = mk_cmd("xpbench/fms/load", "Load xpbench/fms/plan_in into the pilot FMS/GPS");
    c_save = mk_cmd("xpbench/fms/save", "Save the pilot FMS/GPS plan into xpbench/fms/plan_out");
    c_log_start = mk_cmd("xpbench/log/start", "Start the truth log at xpbench/log/path");
    c_log_stop = mk_cmd("xpbench/log/stop", "Stop the truth log");
    c_guard_reset = mk_cmd("xpbench/guard/reset", "Clear xpbench/guard/tripped");

    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    if (log_fp) fclose(log_fp);
    log_fp = NULL;
    for (int i = 0; i < n_our_refs; i++) XPLMUnregisterDataAccessor(our_refs[i]);
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param) {}
