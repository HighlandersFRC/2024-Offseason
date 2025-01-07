package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Peripherals {
  // private PhotonCamera frontCam = new PhotonCamera("9281_Front"); //TODO: uncomment when using camera

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);

  public Peripherals() {
  }

  /**
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
    // Set the mount pose configuration for the IMU
    pigeonConfig.MountPose.MountPosePitch = 0.3561480641365051;
    pigeonConfig.MountPose.MountPoseRoll = -0.10366992652416229;
    pigeonConfig.MountPose.MountPoseYaw = -0.24523599445819855;

    // Apply the IMU configuration
    pigeon.getConfigurator().apply(pigeonConfig);

    // Zero the IMU angle
    zeroPigeon();
  }

  // public double getFrontCamYaw() { //TODO: uncomment when using camera
  //   double yaw = 0.0;
  //   var result = frontCam.getLatestResult();
  //   Logger.recordOutput("has target", result.hasTargets());
  //   if (result.hasTargets()) {
  //     PhotonTrackedTarget target = result.getBestTarget();
  //     yaw = target.getYaw();
  //   }
  //   return yaw;
  // }

  // public double getFrontCamPitch() {
  //   double pitch = 0.0;
  //   var result = frontCam.getLatestResult();
  //   if (result.hasTargets()) {
  //     PhotonTrackedTarget target = result.getBestTarget();
  //     pitch = target.getPitch();
  //   }

  //   return pitch;
  // }

  private Pose2d getRobotPoseViaTrig(PhotonTrackedTarget trackedTarget, double[] cameraPositionOnRobot,
      double robotAngle) {
    double pitch = trackedTarget.getPitch();
    double yaw = trackedTarget.getYaw();
    int id = trackedTarget.getFiducialId();
    double cameraXOffset = cameraPositionOnRobot[0];
    double cameraYOffset = cameraPositionOnRobot[1];
    double cameraZOffset = cameraPositionOnRobot[2];
    double cameraRYOffset = cameraPositionOnRobot[4];
    double cameraRZOffset = cameraPositionOnRobot[5];

    double[] tagPose = Constants.Vision.TAG_POSES[id - 1];
    double tagHeight = tagPose[2];
    double tagX = tagPose[0];
    double tagY = tagPose[1];

    double distToTag = (tagHeight - cameraZOffset) / Math.tan(Math.toRadians(pitch + cameraRYOffset));
    Logger.recordOutput("Distance to Tag", distToTag);
    Logger.recordOutput("yaw to Tag", yaw);
    // double txProjOntoGroundPlane = Math.atan((Math.tan(yaw)) / Math.cos(pitch));
    double xFromTag = distToTag * Math.cos(Math.toRadians(yaw + robotAngle + cameraRZOffset));
    double yFromTag = distToTag * Math.sin(Math.toRadians(yaw + robotAngle + cameraRZOffset));
    Logger.recordOutput("x to Tag", xFromTag);
    Logger.recordOutput("y to Tag", yFromTag);

    double fieldPoseX = xFromTag + tagX - cameraXOffset;
    double fieldPoseY = yFromTag + tagY - cameraYOffset;
    Pose2d pose = new Pose2d(fieldPoseX, fieldPoseY, new Rotation2d(getPigeonAngle()));
    return pose;
  }

  // public Pose2d getFrontCamTrigPose() { //TODO: uncomment when using camera
  //   var result = frontCam.getLatestResult();
  //   if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.3) {
  //     PhotonTrackedTarget target = result.getBestTarget();
  //     Pose2d robotPose = getRobotPoseViaTrig(target, Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
  //     Logger.recordOutput("Trig Localiazation", robotPose);
  //     return robotPose;
  //   } else {
  //     Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  //     return defaultPose;
  //   }
  // }

  // public Pose3d getFrontCamPnPPose() {
  //   var result = frontCam.getLatestResult();
  //   Optional<EstimatedRobotPose> multiTagResult = photonPoseEstimator.update(result);
  //   if (multiTagResult.isPresent()) {
  //     Pose3d robotPose = multiTagResult.get().estimatedPose;
  //     Logger.recordOutput("multitag result", robotPose);
  //     return robotPose;
  //   } else {
  //     Pose3d robotPose = new Pose3d();
  //     return robotPose;
  //   }
  // }

  // public double getFrontCamLatency() {
  //   return frontCam.getLatestResult().getTimestampSeconds();
  // }

  /**
   * Sets the IMU angle to 0
   */
  public void zeroPigeon() {
    setPigeonAngle(0.0);
  }

  /**
   * Sets the angle of the IMU
   * 
   * @param degrees - Angle to be set to the IMU
   */
  public void setPigeonAngle(double degrees) {
    pigeon.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * 
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in device
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in device
   *         coordinates.
   *         The value is in degrees per second.
   */
  public double getPigeonAngularVelocity() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in world
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in world
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocityW() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Retrieves the acceleration vector of the robot
   * 
   * @return Current acceleration vector of the robot
   */
  public Vector getPigeonLinAccel() {
    Vector accelVector = new Vector();
    accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    return accelVector;
  }

  public void periodic() {
    // getFrontCamPnPPose(); //TODO: uncomment when using camera
    // var result = frontCam.getLatestResult();
    // if (result.hasTargets()) {
    //   PhotonTrackedTarget target = result.getBestTarget();
    //   Pose2d robotPose = getRobotPoseViaTrig(target, Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
    //   Logger.recordOutput("Trig Localiazation", robotPose);
    // }
  }
}