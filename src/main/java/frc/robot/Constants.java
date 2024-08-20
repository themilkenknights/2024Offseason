// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeElevator {
    public static class Intakes {
      public static final int topIntakeCANID = 22;
      public static final int midIntakeCANID = 27;
      public static final int bottomIntakeCANID = 23;
    }

    public static class Elevator {
      public static int leftMotorCANID = 20;
      public static int rightMotorCANID = 21;
    }
  }

  public static class Climb {
    public static final int ClimbCANID = 24;
    public static final int ClimbServoPORT = 0;

    public static class setpoints {
      public static double upSetpoint = 70;
      public static double downSetpoint = -1;

      public static double servoLockedSetpoint = 0.6;
      public static double servoOpenSetpoint = 0.3;
      public static final double servoDelay = 0.2;
    }

    public static class gains {
      public static double GainUpP = 0.9;
      public static double GainDownP = 2.25;
      public static double gainI = 0;
      public static double gainD = 0.1;

      public static double tolerance = 3;
    }
  }

  public static final BooleanSupplier isFlipped = () -> // flip supplier
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
}
