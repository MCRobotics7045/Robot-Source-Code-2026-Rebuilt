// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.AllianceShiftConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftUtil {
  public enum ShiftState {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME
  }

  public record ShiftInfo(
      ShiftState state,
      boolean active,
      double blockElapsed, // seconds into current contiguous active/inactive block
      double blockRemaining, // seconds left in current contiguous active/inactive block
      boolean gameDataValid // false if FMS game data not received
      ) {}

  private static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static ShiftInfo getShiftInfo() {
    if (DriverStation.isDisabled()) {
      return new ShiftInfo(ShiftState.DISABLED, false, 0, 0, true);
    }
    if (DriverStation.isAutonomousEnabled()) {
      return new ShiftInfo(ShiftState.AUTO, true, 0, 0, true);
    }
    if (!DriverStation.isTeleopEnabled()) {
      return new ShiftInfo(ShiftState.DISABLED, false, 0, 0, true);
    }

    // Parse FMS game data
    String gameData = DriverStation.getGameSpecificMessage();

    // In simulation, allow mocking FMS game data for testing

    boolean gameDataValid =
        !gameData.isEmpty() && (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B');
    boolean shift1Active;
    if (!gameDataValid) {
      shift1Active = true; // assume active if data missing (matches existing behavior)
    } else {
      boolean redInactiveFirst = (gameData.charAt(0) == 'R');
      shift1Active = isRed() ? !redInactiveFirst : redInactiveFirst;
    }
    boolean gameDataPresent = !gameData.isEmpty(); // for reporting

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) {
      // Match timer not available (practice without FMS)
      return new ShiftInfo(ShiftState.TRANSITION, true, 0, 10, gameDataPresent && gameDataValid);
    }

    // Determine current state
    ShiftState state;
    if (matchTime > TRANSITION_TIME) state = ShiftState.TRANSITION;
    else if (matchTime > SHIFT_1_TIME) state = ShiftState.SHIFT_1;
    else if (matchTime > SHIFT_2_TIME) state = ShiftState.SHIFT_2;
    else if (matchTime > SHIFT_3_TIME) state = ShiftState.SHIFT_3;
    else if (matchTime > END_GAME_TIME) state = ShiftState.SHIFT_4;
    else state = ShiftState.END_GAME;

    boolean active =
        switch (state) {
          case TRANSITION, END_GAME -> true;
          case SHIFT_1, SHIFT_3 -> shift1Active;
          case SHIFT_2, SHIFT_4 -> !shift1Active;
          default -> false;
        };

    // Block boundaries: only merge TRANSITION+SHIFT_1 when shift1Active=true
    double blockStart;
    double blockEnd;
    switch (state) {
      case TRANSITION -> {
        blockStart = 140.0;
        blockEnd = shift1Active ? SHIFT_1_TIME : TRANSITION_TIME;
      }
      case SHIFT_1 -> {
        blockStart = shift1Active ? 140.0 : TRANSITION_TIME;
        blockEnd = SHIFT_1_TIME;
      }
      case SHIFT_2 -> {
        blockStart = SHIFT_1_TIME;
        blockEnd = SHIFT_2_TIME;
      }
      case SHIFT_3 -> {
        blockStart = SHIFT_2_TIME;
        blockEnd = SHIFT_3_TIME;
      }
      case SHIFT_4 -> {
        blockStart = SHIFT_3_TIME;
        blockEnd = END_GAME_TIME;
      }
      default -> { // END_GAME
        blockStart = END_GAME_TIME;
        blockEnd = 0.0;
      }
    }

    double blockElapsed = Math.max(0, blockStart - matchTime);
    double blockRemaining = Math.max(0, matchTime - blockEnd);

    // Log shift info to SmartDashboard for driver heads-up display
    logShiftInfo(state, active, shift1Active);

    return new ShiftInfo(
        state, active, blockElapsed, blockRemaining, gameDataPresent && gameDataValid);
  }

  private static void logShiftInfo(ShiftState state, boolean active, boolean shift1Active) {
    String shiftName = state.toString();
    String whoseShift;

    if (state == ShiftState.TRANSITION || state == ShiftState.END_GAME) {
      whoseShift = "BOTH (Neutral)";
    } else {
      whoseShift = active ? "BLUE (Ours)" : "RED (Opponent)";
      // Adjust based on actual alliance
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
          whoseShift = active ? "RED (Ours)" : "BLUE (Opponent)";
        }
      }
    }

    // Determine upcoming shift and which alliance has it
    String upcomingShiftInfo = getUpcomingShiftInfo(state, shift1Active);

    // Get match time remaining
    double matchTimeRemaining = DriverStation.getMatchTime();

    SmartDashboard.putString("Shift/State", shiftName);
    SmartDashboard.putBoolean("Shift/Active", active);
    SmartDashboard.putString("Shift/WhoseHub", whoseShift);
    SmartDashboard.putString("Shift/Upcoming", upcomingShiftInfo);
    SmartDashboard.putNumber("MatchTime", matchTimeRemaining);
  }

  private static String getUpcomingShiftInfo(ShiftState state, boolean shift1Active) {
    // Determine which alliance will have the upcoming shift
    String upcomingAlliance;
    ShiftState upcomingState;

    switch (state) {
      case TRANSITION -> {
        upcomingState = ShiftState.SHIFT_1;
        upcomingAlliance = !shift1Active ? "BLUE" : "RED";
      }
      case SHIFT_1 -> {
        upcomingState = ShiftState.SHIFT_2;
        upcomingAlliance = shift1Active ? "BLUE" : "RED";
      }
      case SHIFT_2 -> {
        upcomingState = ShiftState.SHIFT_3;
        upcomingAlliance = !shift1Active ? "BLUE" : "RED";
      }
      case SHIFT_3 -> {
        upcomingState = ShiftState.SHIFT_4;
        upcomingAlliance = shift1Active ? "BLUE" : "RED";
      }
      default -> {
        return ""; // No meaningful upcoming shift
      }
    }

    return "NEXT SHIFT: " + upcomingAlliance;
  }
}
