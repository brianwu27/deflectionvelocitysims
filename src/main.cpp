// Purpose:
//   Given a vertical-velocity time history (from OpenRocket) this program
//   generates a deflection sequence for fins/ailerons that advances through a
//   user-defined list of target angles. It enforces a minimum dwell time and a
//   velocity-change trigger so you get good coverage across velocities, and it
//   rate-limits the command to be physically achievable by your servo.
//
// How to read this file:
//   - "SequencerConfig": user-tunable parameters (sequence, Δv, dwell, servo rate)
//   - "run_sequence":    the step-by-step algorithm
//   - "main":            loads CSV -> runs the sequencer -> writes deflection_log.csv
//
// Firmware notes (what you’ll eventually port):
//   - Keep the SAME state machine + slew limiter (advance rule + rate limit)
//   - Replace the CSV velocity with the estimator’s v_z (m/s) from avionics
//   - Call the update logic at a fixed control rate (e.g., 100 Hz) with dt
//   - Map the single "cmd_deg" to your 4 servos (signs/phases per your layout)
//   - Add safety: arming conditions, angle clamps, validity fallback, logging


#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// One data sample from the input CSV.
struct Sample {
    double t;   // seconds (monotonic increasing)
    double vz;  // m/s (vertical velocity; + upward)
};

// Load a CSV: first line is a header (ignored).
// Each subsequent line is expected to have at least two comma-separated fields:
//   column 1: time [s]
//   column 2: vertical velocity [m/s]
static std::vector<Sample> load_velocity_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open: " + path);
    }
    std::vector<Sample> data;
    std::string line;
    bool header_skipped = false;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (!header_skipped) { header_skipped = true; continue; } // skip header
        std::stringstream ss(line);
        std::string tok_t, tok_v;
        if (!std::getline(ss, tok_t, ',')) continue;
        if (!std::getline(ss, tok_v, ',')) continue;
        try {
            double t  = std::stod(tok_t);
            double vz = std::stod(tok_v);
            data.push_back({t, vz});
        } catch (...) {
            // Skip malformed rows quietly
        }
    }
    std::sort(data.begin(), data.end(),
              [](const Sample& a, const Sample& b){ return a.t < b.t; });
    return data;
}

// ========================== USER TUNABLE CONFIG ===============================
// This is where you set the sweep shape and behavior. For SIM you can hardcode.
// For FIRMWARE you can keep these as constants, read from a params file/flash,
// or expose via GCS/ground-command if you support that.
struct SequencerConfig {
    // The sequence of *target* angles [deg] to step through (will loop forever).
    // You chose a ramp up to +8, back to 0, down to -8, back to 0 (1° steps).
    // Change this vector to change the sweep shape.
    std::vector<double> deflection_targets_deg {
        0.0, 4.0, 8.0,
        4.0, 0.0,
        -2.0, -4.0, -6.0, -8.0,
        -6.0, -4.0, -2.0, 0.0
    }; // cycle

    // Δv threshold [m/s]: advance to the next target only after the *absolute*
    // change in vertical velocity since the last switch exceeds this value.
    // Lower => more frequent switches (denser velocity coverage).
    double dv_per_step = 20;        // m/s change in |vz| to advance state

    // Minimum dwell time [s]: don’t switch faster than this even if Δv is met.
    // Prevents chattering at high acceleration and respects servo/logger limits.
    double min_dwell_s = 0.4;        // s minimum time to hold a state

    // Servo angular speed limit [deg/s] from datasheet/bench test (slew rate).
    // Your servo is ~0.11–0.14 s per 60° => ~430–545 deg/s no-load.
    double slew_rate_deg_per_s = 500; // actuator rate limit
};

// Internal state of the sequencer; carry this across time steps in firmware.
struct SequencerState {
    size_t idx = 0;               // which entry in deflection_targets_deg
    double target_deg = 0.0;      // current target angle [deg] (ideal goal)
    double cmd_deg = 0.0;         // current commanded angle [deg] (slew-limited)
    double last_switch_time = 0.0;// last time we advanced to a new target [s]
    double vz_at_last_switch = 0.0;// vz value at that last switch [m/s]
};

// Core algorithm (SIM version):
// Walks through time samples, updates the slew-limited command toward the
// current target, and advances the target when BOTH dwell >= min_dwell_s AND
// |Δv| >= dv_per_step.
//
// Firmware mapping:
//   - Call this logic at your control loop rate with (dt, v_z from estimator).
//   - Replace the CSV traversal with your periodic tick.
//   - "out << ..." is SIM-only; in firmware you’ll send cmd_deg to PWM instead.
static void run_sequence(
    const std::vector<Sample>& traj,
    const SequencerConfig& cfg,
    const std::string& out_csv_path
) {
    if (traj.empty()) throw std::runtime_error("No input samples.");

    // Initialize the state at the first sample.
    SequencerState st;
    st.idx = 0;
    st.target_deg = cfg.deflection_targets_deg[st.idx];
    st.cmd_deg = 0.0; // start at 0 deg command (adjust if you have a bias)
    st.last_switch_time = traj.front().t;
    st.vz_at_last_switch = traj.front().vz;

    // Ensure output folder exists (SIM convenience).
    std::filesystem::create_directories(std::filesystem::path(out_csv_path).parent_path());

    // Output CSV columns:
    // time_s, vel_z_mps = input; deflection_deg = slew-limited command; target_deg = ideal step
    std::ofstream out(out_csv_path);
    if (!out) throw std::runtime_error("Failed to open output: " + out_csv_path);
    out << "time_s,vel_z_mps,deflection_deg,target_deg\n";

    for (size_t i = 0; i < traj.size(); ++i) {
        const auto& s = traj[i];

        // dt is used for the servo rate limiting. In firmware this will be your
        // control loop period (e.g., 0.01 s for 100 Hz), not derived from CSV.
        double dt = (i > 0) ? std::max(0.0, traj[i].t - traj[i-1].t) : 0.0;

        // --- Slew toward target with servo speed limit -----------------------
        // This enforces physical reality: the commanded angle cannot "teleport".
        const double max_step = cfg.slew_rate_deg_per_s * dt; // [deg] allowed this tick
        const double err = st.target_deg - st.cmd_deg;
        if (std::fabs(err) <= max_step) {
            st.cmd_deg = st.target_deg; // reached target within this tick
        } else {
            st.cmd_deg += (err > 0 ? +max_step : -max_step); // move toward target
        }

        // --- Advance rule (state machine) ------------------------------------
        // Conditions to step to the next target:
        // 1) Dwell long enough, AND
        // 2) Accumulated |Δv| since last switch is large enough.

        // Firmware tip:
        //   - Optionally add a max_dwell_s cap to guarantee progress even if Δv stalls.
        //   - If v_z becomes invalid, fall back to a fixed-period step.
        const double dv_since = std::fabs(s.vz - st.vz_at_last_switch);
        const double dwell = s.t - st.last_switch_time;
        if (dwell >= cfg.min_dwell_s && dv_since >= cfg.dv_per_step) {
            st.idx = (st.idx + 1) % cfg.deflection_targets_deg.size(); // wrap around
            st.target_deg = cfg.deflection_targets_deg[st.idx];
            st.last_switch_time = s.t;
            st.vz_at_last_switch = s.vz;
        }

        // --- Log current sample (SIM only) -----------------------------------
        out << s.t << "," << s.vz << "," << st.cmd_deg << "," << st.target_deg << "\n";
    }

    std::cout << "Wrote: " << out_csv_path << "\n";
}

int main(int argc, char** argv) {
    try {
        // Input/Output paths (SIM only). In firmware you'll read estimator v_z
        // each control tick and write to telemetry instead of a file.
        std::string in_path  = (argc > 1) ? argv[1] : "data/velocitydata.csv";
        std::string out_path = (argc > 2) ? argv[2] : "out/deflection_log.csv";

        // Load velocity profile from OpenRocket (SIM).
        auto traj = load_velocity_csv(in_path);

        // Create config using struct defaults. For SIM you can tweak defaults
        // above; for firmware these will likely come from constants/params.
        SequencerConfig cfg;

        // Run the simulation (CSV in -> CSV out).
        run_sequence(traj, cfg, out_path);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
