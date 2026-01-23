/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"

#include "../../MarlinCore.h"

#if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"
#include "../../module/planner.h"  // for planner.synchronize() in XY interlock

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/stepper.h"
#endif

#include "../../module/dexarm/dexarm.h"
#include "../../module/dexarm/dexarm_position_reachable.h"  // For ZMINARM, ZMAXARM

extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
void GcodeSuite::G0_G1(
  #if IS_SCARA || IS_DEXARM || defined(G0_FEEDRATE)
    const bool fast_move/*=false*/
  #endif
) {

  if (IsRunning()
    #if ENABLED(NO_MOTION_BEFORE_HOMING)
      && !axis_unhomed_error(
          (parser.seen('X') ? _BV(X_AXIS) : 0)
        | (parser.seen('Y') ? _BV(Y_AXIS) : 0)
        | (parser.seen('Z') ? _BV(Z_AXIS) : 0) )
    #endif
  ) {

    // Safe Travel Mode: G0/G1 with T1 parameter
    // Sequence: 1) Move Z to safe height, 2) Move XY + E, 3) Move Z to final destination
    // Usage: G0 X100 Y50 T1      - safe travel to XY
    //        G0 X100 Y50 Z-20 T1 - safe travel to XY, then Z
    //        G0 X100 Y50 E45 T1  - safe travel to XY with rotation
    // Without T1: direct movement (normal behavior, good for jogging)
    if (parser.seen('T') && parser.value_int() == 1) {
      const float safe_z = xy_interlock_safe_z;
      // SAFETY: Validate safe_z is within DexArm limits (ZMINARM to ZMAXARM)
      const bool safe_z_valid = (safe_z >= ZMINARM && safe_z <= ZMAXARM);
      
      if (safe_z_valid) {
        const bool has_xy_move = parser.seen('X') || parser.seen('Y');
        const bool z_not_at_safe = (current_position.z < safe_z - 0.1f) || (current_position.z > safe_z + 0.1f);
        
        if (has_xy_move && z_not_at_safe) {
          // Save user's desired final Z (if specified) and feedrate
          const bool has_z_target = parser.seenval('Z');
          const float final_z = has_z_target ? parser.value_float() : current_position.z;
          const feedRate_t user_feedrate = parser.seenval('F') ? MMM_TO_MMS(parser.value_float()) : feedrate_mm_s;
          
          // Get XY targets
          const float target_x = parser.seenval('X') ? parser.value_float() : current_position.x;
          const float target_y = parser.seenval('Y') ? parser.value_float() : current_position.y;
          
          // Get E (rotation) target - don't silently drop this!
          const bool has_e_target = parser.seenval('E');
          const float target_e = has_e_target ? parser.value_float() : current_position.e;
          
          // SAFETY: Check if XY target is reachable at safe_z height
          // At high Z, the reachable radius shrinks significantly
          xyz_pos_t check_pos = { target_x, target_y, safe_z };
          if (!dexarm_position_is_reachable(check_pos)) {
            SERIAL_ECHOLNPGM("Error: XY target unreachable at safe travel height, using direct move");
            // Fall through to normal G0/G1 processing
          } else {
            // Step 1: Move Z to safe height first
            destination = current_position;
            destination.z = safe_z;
            feedrate_mm_s = MMM_TO_MMS(3000);  // Z travel at 3000 mm/min
            prepare_line_to_destination();
            planner.synchronize();
            current_position.z = safe_z;
            
            // Step 2: Move XY (and E if specified) at safe height
            destination = current_position;
            destination.x = target_x;
            destination.y = target_y;
            destination.e = target_e;  // Include rotation
            feedrate_mm_s = user_feedrate;
            prepare_line_to_destination();
            planner.synchronize();
            current_position.x = target_x;
            current_position.y = target_y;
            current_position.e = target_e;
            
            // Step 3: Move Z to final destination (if user specified Z, or stay at safe_z)
            if (has_z_target) {
              destination = current_position;
              destination.z = final_z;
              feedrate_mm_s = MMM_TO_MMS(3000);  // Z travel at 3000 mm/min
              prepare_line_to_destination();
              planner.synchronize();
              current_position.z = final_z;
            }
            
            // Restore feedrate and exit - we handled the complete move
            feedrate_mm_s = user_feedrate;
            return;  // Don't execute normal G0/G1 processing
          }
        }
      }
      // If safe_z invalid, XY unreachable, or no XY move needed, fall through to normal processing
    }

    #ifdef G0_FEEDRATE
      feedRate_t old_feedrate;
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        if (fast_move) {
          old_feedrate = feedrate_mm_s;             // Back up the (old) motion mode feedrate
          feedrate_mm_s = fast_move_feedrate;       // Get G0 feedrate from last usage
        }
      #endif
    #endif

    get_destination_from_command();                 // Process X Y Z E F parameters

    #ifdef G0_FEEDRATE
      if (fast_move) {
        #if ENABLED(VARIABLE_G0_FEEDRATE)
          fast_move_feedrate = feedrate_mm_s;       // Save feedrate for the next G0
        #else
          old_feedrate = feedrate_mm_s;             // Back up the (new) motion mode feedrate
          feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);  // Get the fixed G0 feedrate
        #endif
      }
    #endif

    #if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)

      if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
        // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves
        if (fwretract.autoretract_enabled && parser.seen('E') && !(parser.seen('X') || parser.seen('Y') || parser.seen('Z'))) {
          const float echange = destination.e - current_position.e;
          // Is this a retract or recover move?
          if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[active_extruder] == (echange > 0.0)) {
            current_position.e = destination.e;       // Hide a G1-based retract/recover from calculations
            sync_plan_position_e();                   // AND from the planner
            return fwretract.retract(echange < 0.0);  // Firmware-based retract/recover (double-retract ignored)
          }
        }
      }

    #endif // FWRETRACT

    if (G0_MOVE_MODE == FAST_MODE)
    {
      #if IS_SCARA || IS_DEXARM
        fast_move ? prepare_fast_move_to_destination() : prepare_line_to_destination();
      #else
        prepare_line_to_destination();
      #endif
    }else if (G0_MOVE_MODE == LINE_MODE){
      prepare_line_to_destination();
    }

    #ifdef G0_FEEDRATE
      // Restore the motion mode feedrate
      if (fast_move) feedrate_mm_s = old_feedrate;
    #endif

    #if ENABLED(NANODLP_Z_SYNC)
      #if ENABLED(NANODLP_ALL_AXIS)
        #define _MOVE_SYNC parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z')  // For any move wait and output sync message
      #else
        #define _MOVE_SYNC parser.seenval('Z')  // Only for Z move
      #endif
      if (_MOVE_SYNC) {
        planner.synchronize();
        SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
      }
    #endif
  }
}
