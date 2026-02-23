#pragma once

/// Interactive PID tuning routines for the robodash selector.
/// Each routine runs a specific test movement, prints per-loop telemetry,
/// and reports overshoot / settle time at the end.

void tunePID_Angular();
void tunePID_Linear();
void tunePID_Heading();
