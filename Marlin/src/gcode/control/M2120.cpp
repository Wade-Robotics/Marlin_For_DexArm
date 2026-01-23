/**
 * M2120: XY Interlock (Safe Travel Mode) Configuration
 *
 * When enabled, the arm uses "safe travel" pattern for moves:
 *   1. Lift to safe Z height
 *   2. Move XY to target
 *   3. Lower to final Z
 *
 * This protects against collisions when moving across the workspace.
 * Also applies to homing (M1112) when enabled.
 *
 * Usage:
 *   M2120 S1       - Enable XY interlock
 *   M2120 S0       - Disable XY interlock
 *   M2120 Z50      - Set safe travel height to 50mm
 *   M2120 S1 Z50   - Enable and set height in one command
 *   M2120          - Report current settings
 *
 * The safe travel height is bounded by firmware limits (ZMINARM to ZMAXARM).
 *
 * When enabled:
 *   - G0/G1 with T1 uses safe travel pattern (explicit)
 *   - M1112 (home) uses safe travel pattern (automatic)
 *   - Regular G0/G1 without T1 still moves directly (for jogging)
 */

#include "../gcode.h"
#include "../../module/dexarm/dexarm.h"
#include "../../module/dexarm/dexarm_position_reachable.h"  // For ZMINARM, ZMAXARM

// Default to Z=0 (home/base level) - user should set appropriate height for their setup
#define XY_INTERLOCK_Z_DEFAULT 0.0f

void GcodeSuite::M2120() {
  // Check for S parameter (enable/disable)
  if (parser.seenval('S')) {
    int s_val = parser.value_int();
    xy_interlock_enabled = (s_val != 0);
  }

  // Check for Z parameter (safe height)
  if (parser.seenval('Z')) {
    float new_z = parser.value_float();
    // Clamp to DexArm actual limits
    if (new_z < ZMINARM) {
      SERIAL_ECHOPGM("Warning: Z clamped to min (");
      SERIAL_ECHO(ZMINARM);
      SERIAL_ECHOLNPGM("mm)");
      new_z = ZMINARM;
    }
    if (new_z > ZMAXARM) {
      SERIAL_ECHOPGM("Warning: Z clamped to max (");
      SERIAL_ECHO(ZMAXARM);
      SERIAL_ECHOLNPGM("mm)");
      new_z = ZMAXARM;
    }
    xy_interlock_safe_z = new_z;
  }

  // SAFETY: Validate current value is within DexArm limits
  // If garbage detected, reset to safe default (Z=0, base/home level)
  if (xy_interlock_safe_z < ZMINARM || xy_interlock_safe_z > ZMAXARM) {
    SERIAL_ECHOLNPGM("Warning: Invalid safe_z detected, resetting to 0mm");
    xy_interlock_safe_z = XY_INTERLOCK_Z_DEFAULT;
  }

  // Report current settings
  SERIAL_ECHOPGM("XY Interlock: ");
  SERIAL_ECHO(xy_interlock_enabled ? "ON" : "OFF");
  SERIAL_ECHOPGM(", Safe Z: ");
  SERIAL_ECHO(xy_interlock_safe_z);
  SERIAL_ECHOLNPGM("mm");
}
