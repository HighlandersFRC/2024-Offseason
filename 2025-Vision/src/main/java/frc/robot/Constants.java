// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

public final class Constants {
  public static final class Autonomous {
    // how far forward to look when the linear radius and the angular radius equal
    // their constants
    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 0.48;
    public static final double AUTONOMOUS_END_ACCURACY = 0.25;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1.0;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;
    public static final String[] paths = new String[] {
        "4 Far.polarauto",
        "4 Far 123.polarauto",
        "4 Far 231.polarauto",
        "5 piece.polarauto",
        "3 amp.polarauto",
        "3 amp 231.polarauto",
        "1 Exit.polarauto",
        "Middle Note.polarauto",
        "Far 321.polarauto"
    };

    public static int getSelectedPathIndex() {
      if (!OI.autoChooser.getRawButton(7)) {
        if (OI.autoChooser.getRawButton(1)) {
          return 3;
        }
        if (OI.autoChooser.getRawButton(2)) {
          return 2;
        }
        if (OI.autoChooser.getRawButton(3)) {
          return 0;
        }
        if (OI.autoChooser.getRawButton(4)) {
          return 1;
        }
        if (OI.autoChooser.getRawButton(5)) {
          return 4;
        }
      } else {
        if (OI.autoChooser.getRawButton(1)) {
          return 5;
        }
        if (OI.autoChooser.getRawButton(2)) {
          return 6;
        }
        if (OI.autoChooser.getRawButton(3)) {
          return 7;
        }
        if (OI.autoChooser.getRawButton(4)) {
          return 8;
        }
      }
      return -1;
    }

  }

  public static void periodic() {
    int index = Autonomous.getSelectedPathIndex();
    if (index == -1) {
      Logger.recordOutput("Selected Auto", "Do Nothing");
    } else {
      Logger.recordOutput("Selected Auto", Autonomous.paths[index]);
    }
  }

  // Physical constants (e.g. field and robot dimensions)
  public static final class Physical {
    public static final double FIELD_WIDTH = 8.2;
    public static final double FIELD_LENGTH = 16.63;
    public static final double ROBOT_RADIUS = inchesToMeters(15.429);
    public static final double WHEEL_DIAMETER = inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;

    public static final double TOP_SPEED = feetToMeters(25);

    public static final double ROBOT_LENGTH = inchesToMeters(25);
    public static final double ROBOT_WIDTH = inchesToMeters(28.5);
    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final double GRAVITY_ACCEL_MS2 = 9.806;
  }

  // Subsystem setpoint constants
  public static final class SetPoints {
    public static final double ELEVATOR_BOTTOM_POSITION_M = 0.0;
    public static final double ELEVATOR_MID_POSITION_M = 0.22;
    public static final double ELEVATOR_TOP_POSITION_M = 0.43;

    public enum ElevatorPosition {
      kDOWN(ELEVATOR_BOTTOM_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_BOTTOM_POSITION_M)),
      kMID(ELEVATOR_MID_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_MID_POSITION_M)),
      kUP(ELEVATOR_TOP_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_TOP_POSITION_M));

      public final double meters;
      public final double rotations;

      private ElevatorPosition(double meters, double rotations) {
        this.meters = meters;
        this.rotations = rotations;
      }
    }
  }

  // Vision constants (e.g. camera offsets)
  public static final class Vision {
    // Poses of all 16 AprilTags, {x, y, z, theta}, in meters and radians
    public static final double[][] TAG_POSES = {
        { 15.079502159004317, 0.2458724917449835, 1.3558547117094235, 2.0943951023931953 },
        { 16.18516637033274, 0.8836677673355348, 1.3558547117094235, 2.0943951023931953 },
        { 16.57937515875032, 4.982727965455931, 1.4511049022098046, 3.141592653589793 },
        { 16.57937515875032, 5.547879095758192, 1.4511049022098046, 3.141592653589793 },
        { 14.700787401574804, 8.204216408432817, 1.3558547117094235, 4.71238898038469 },
        { 1.841503683007366, 8.204216408432817, 1.3558547117094235, 4.71238898038469 },
        { -0.038100076200152405, 5.547879095758192, 1.4511049022098046, 0.0 },
        { -0.038100076200152405, 4.982727965455931, 1.4511049022098046, 0.0 },
        { 0.35610871221742446, 0.8836677673355348, 1.3558547117094235, 1.0471975511965976 },
        { 1.4615189230378463, 0.2458724917449835, 1.3558547117094235, 1.0471975511965976 },
        { 11.90474980949962, 3.713233426466853, 1.3208026416052834, 5.235987755982989 },
        { 11.90474980949962, 4.4983489966979935, 1.3208026416052834, 1.0471975511965976 },
        { 11.220218440436883, 4.105156210312421, 1.3208026416052834, 3.141592653589793 },
        { 5.320802641605283, 4.105156210312421, 1.3208026416052834, 0.0 },
        { 4.641351282702566, 4.4983489966979935, 1.3208026416052834, 2.0943951023931953 },
        { 4.641351282702566, 3.713233426466853, 1.3208026416052834, 4.1887902047863905 }
    };

    // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
    // radians
    public static final double[] FRONT_CAMERA_POSE = { 0.0, 0.0, 0.5, 0.0, 0.0, 0.0 };

    // Standard deviation adjustments
    public static final double STANDARD_DEVIATION_SCALAR = 1;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR = 1;
    public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE = 3;
    public static final double TAG_STANDARD_DEVIATION_DISTANCE = 2; // meters
    public static final double TAG_STANDARD_DEVIATION_FLATNESS = 5;

    // Standard deviation regressions
    /**
     * Calculates the standard deviation scalar based on the distance from the tag.
     *
     * @param dist The distance from the tag.
     * @return The standard deviation scalar.
     */
    public static double getTagDistStdDevScalar(double dist) {
      double a = TAG_STANDARD_DEVIATION_FLATNESS;
      double b = 1 - a * Math.pow(TAG_STANDARD_DEVIATION_DISTANCE, 2);
      return Math.max(1, a * Math.pow(dist, 2) + b);
    }

    /**
     * Calculates the standard deviation scalar based on the number of detected
     * tags.
     *
     * @param numTags The number of detected tags.
     * @return The standard deviation scalar.
     */
    public static double getNumTagStdDevScalar(int numTags) {
      if (numTags == 0) {
        return 99999;
      } else if (numTags == 1) {
        return 2;
      } else if (numTags == 2) {
        return 1;
      } else {
        return 0.75;
      }
    }

    /**
     * Calculates the standard deviation of the x-coordinate based on the given
     * offsets.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation of the x-coordinate.
     */
    public static double getTagStdDevX(double xOffset, double yOffset) {
      return Math.max(0, 0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset) - 0.010807566510145635)
          * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation of the y-coordinate based on the given
     * offsets.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation of the y-coordinate.
     */
    public static double getTagStdDevY(double xOffset, double yOffset) {
      return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626)
          * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation in the x-coordinate for triangulation
     * measurements.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation in the x-coordinate.
     */
    public static double getTriStdDevX(double xOffset, double yOffset) {
      return Math.max(0, 0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset) - 0.01955724864971872)
          * STANDARD_DEVIATION_SCALAR;
    }

    /**
     * Calculates the standard deviation in the y-coordinate for triangulation
     * measurements.
     *
     * @param xOffset The x-coordinate offset.
     * @param yOffset The y-coordinate offset.
     * @return The standard deviation in the y-coordinate.
     */
    public static double getTriStdDevY(double xOffset, double yOffset) {
      return Math.max(0, 0.002615358015002413 * (xOffset * xOffset + yOffset * yOffset) - 0.008955462032388808)
          * STANDARD_DEVIATION_SCALAR;
    }
  }

  // Gear ratios and conversions
  public static final class Ratios {
    // drive
    public static final double DRIVE_GEAR_RATIO = 5.9;
    public static final double STEER_GEAR_RATIO = 21.43;

    // elevator
    public static final double ELEVATOR_GEAR_RATIO = 23.52;
    public static final double ELEVATOR_MOTOR_ROTATIONS_PER_METER = 219.254;

    public static double elevatorRotationsToMeters(double rotations) {
      return rotations / ELEVATOR_MOTOR_ROTATIONS_PER_METER;
    }

    public static double elevatorMetersToRotations(double meters) {
      return meters * ELEVATOR_MOTOR_ROTATIONS_PER_METER;
    }
  }

  // Can info such as IDs
  public static final class CANInfo {
    public static final String CANBUS_NAME = "Canivore";

    // drive
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 2;
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 8;
    public static final int FRONT_RIGHT_MODULE_CANCODER_ID = 1;
    public static final int FRONT_LEFT_MODULE_CANCODER_ID = 2;
    public static final int BACK_LEFT_MODULE_CANCODER_ID = 3;
    public static final int BACK_RIGHT_MODULE_CANCODER_ID = 4;

    // Lights
    public static final int CANDLE_ID = 0;
  }

  // Misc. controller values
  public static final class OperatorConstants {
    public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
    public static final double LEFT_TRIGGER_DEADZONE = 0.1;
  }

  /**
   * Converts inches to meters.
   *
   * @param inches The length in inches to be converted.
   * @return The equivalent length in meters.
   */
  public static double inchesToMeters(double inches) {
    return inches / 39.37;
  }

  public static double metersToInches(double meters) {
    return meters * 39.37;
  }

  /**
   * Converts feet to meters.
   *
   * @param inches The length in feet to be converted.
   * @return The equivalent length in meters.
   */
  public static double feetToMeters(double feet) {
    return feet / 3.281;
  }

  /**
   * Calculates the Euclidean distance between two points in a 2D plane.
   *
   * @param x1 The x-coordinate of the first point.
   * @param y1 The y-coordinate of the first point.
   * @param x2 The x-coordinate of the second point.
   * @param y2 The y-coordinate of the second point.
   * @return The Euclidean distance between the two points.
   */
  public static double getDistance(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  public static double getAngleToPoint(double x1, double y1, double x2, double y2) {
    // System.out.println("x1: " + x1 + ", y1: " + y1 + ", x2: " + x2 + ", y2: " +
    // y2);
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;

    double angleInRadians = Math.atan2(deltaY, deltaX);

    double angleInDegrees = Math.toDegrees(angleInRadians);

    double standardizeAngleDegrees = standardizeAngleDegrees(angleInDegrees);

    // if (y1 > y2) {
    // System.out.println("running");
    return 180 + standardizeAngleDegrees;
    // }
    // System.out.println("2");
    // double temp = 180 - standardizeAngleDegrees;
    // double j = 180 - temp;
    // return 180 + j;
    // }
  }

  /**
   * Converts a quantity in rotations to radians.
   *
   * @param rotations The quantity in rotations to be converted.
   * @return The equivalent quantity in radians.
   */
  public static double rotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  /**
   * Converts a quantity in degrees to rotations.
   *
   * @param rotations The quantity in degrees to be converted.
   * @return The equivalent quantity in rotations.
   */
  public static double degreesToRotations(double degrees) {
    return degrees / 360;
  }

  /**
   * Converts a quantity in rotations to degrees.
   *
   * @param rotations The quantity in rotations to be converted.
   * @return The equivalent quantity in degrees.
   */
  public static double rotationsToDegrees(double rotations) {
    return rotations * 360;
  }

  /**
   * Converts a quantity in degrees to radians.
   *
   * @param rotations The quantity in degrees to be converted.
   * @return The equivalent quantity in radians.
   */
  public static double degreesToRadians(double degrees) {
    return degrees * Math.PI / 180;
  }

  /**
   * Standardizes an angle to be within the range [0, 360) degrees.
   *
   * @param angleDegrees The input angle in degrees.
   * @return The standardized angle within the range [0, 360) degrees.
   */
  public static double standardizeAngleDegrees(double angleDegrees) {
    if (angleDegrees >= 0 && angleDegrees < 360) {
      return angleDegrees;
    } else if (angleDegrees < 0) {
      while (angleDegrees < 0) {
        angleDegrees += 360;
      }
      return angleDegrees;
    } else if (angleDegrees >= 360) {
      while (angleDegrees >= 360) {
        angleDegrees -= 360;
      }
      return angleDegrees;
    } else {
      // System.out.println("Weird ErroR");
      return angleDegrees;
    }
  }

  /**
   * Calculates the x-component of a unit vector given an angle in radians.
   *
   * @param angle The angle in radians.
   * @return The x-component of the unit vector.
   */
  public static double angleToUnitVectorI(double angle) {
    return (Math.cos(angle));
  }

  /**
   * Calculates the y-component of a unit vector given an angle in radians.
   *
   * @param angle The angle in radians.
   * @return The y-component of the unit vector.
   */
  public static double angleToUnitVectorJ(double angle) {
    return (Math.sin(angle));
  }

  /**
   * Converts revolutions per minute (RPM) to revolutions per second (RPS).
   *
   * @param RPM The value in revolutions per minute (RPM) to be converted.
   * @return The equivalent value in revolutions per second (RPS).
   */
  public static double RPMToRPS(double RPM) {
    return RPM / 60;
  }

  /**
   * Converts revolutions per second (RPS) to revolutions per minute (RPM).
   *
   * @param RPM The value in revolutions per second (RPS) to be converted.
   * @return The equivalent value in revolutions per minute (RPM).
   */
  public static double RPSToRPM(double RPS) {
    return RPS * 60;
  }
}
