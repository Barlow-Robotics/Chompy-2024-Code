// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;

public class Drive extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            "frontLeft",
            ElectronicsIDs.FrontLeftDriveMotorID,
            ElectronicsIDs.FrontLeftTurnMotorID,
            ElectronicsIDs.FrontLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontLeftMagnetOffsetInRadians) / 360.0,
            false);

    private final SwerveModule frontRight = new SwerveModule(
            "frontRight",
            ElectronicsIDs.FrontRightDriveMotorID,
            ElectronicsIDs.FrontRightTurnMotorID,
            ElectronicsIDs.FrontRightTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontRightMagnetOffsetInRadians) / 360.0,
            true);

    private final SwerveModule backLeft = new SwerveModule(
            "backLeft",
            ElectronicsIDs.BackLeftDriveMotorID,
            ElectronicsIDs.BackLeftTurnMotorID,
            ElectronicsIDs.BackLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.BackLeftMagnetOffsetInRadians) / 360.0,
            false);

    private final SwerveModule backRight = new SwerveModule(
            "backRight",
            ElectronicsIDs.BackRightDriveMotorID,
            ElectronicsIDs.BackRightTurnMotorID,
            ElectronicsIDs.BackRightTurnEncoderID,
            Math.toDegrees(DriveConstants.BackRightMagnetOffsetInRadians) / 360.0,
            true);

    private AHRS navX;

    private final SwerveDriveOdometry odometry;

    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[4];

    private final Vision visionSub;

    // private final AprilTagFieldLayout aprilTagFieldLayout;

    public Drive(Vision visionSub) {

        navX = new AHRS(Port.kMXP);
        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // zeroHeading();
        // } catch (Exception e) {
        // }
        // }).start();

        this.visionSub = visionSub;

        odometry = new SwerveDriveOdometry(
                DriveConstants.kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        // wpk The pose estimator functionality was addapted from the
        // SwerveDrivePoseEstimator example provided
        // with WPILib
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition() },
                new Pose2d() ,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                // VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        visionSub.photonEstimator.setReferencePose(this.getPose());

        // AprilTagFieldLayout layout;
        // try {
        //     layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        //     var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //         layout.setOrigin(
        //                 DriverStation.getAlliance().get() == Alliance.Blue
        //                         ? OriginPosition.kBlueAllianceWallRightSide
        //                         : OriginPosition.kRedAllianceWallRightSide);
        //     } else {
        //         // default this for now
        //         layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        //     }

        // } catch (IOException e) {
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        //     layout = null;
        // }
        // this.aprilTagFieldLayout = layout;

    }

    @Override
    public void periodic() {
        odometry.update(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        poseEstimator.update(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        var photonEstimate = visionSub.getEstimatedGlobalPose();

        // Vision estimate is not stable. Just use pure odometry for now
        // Don't re-enable until we have done more thorough testing of the
        // vision calibration and initialization.
        
        if (photonEstimate.isPresent()) {
            poseEstimator.addVisionMeasurement(photonEstimate.get().estimatedPose.toPose2d(),
                    photonEstimate.get().timestampSeconds);
        }
        

        Logger.recordOutput("Drive/PoseEstimate", poseEstimator.getEstimatedPosition());
        if (photonEstimate.isPresent()) {
            Logger.recordOutput("Drive/PhotonPoseEstimate", photonEstimate.get().estimatedPose.toPose2d());
        }

        SwerveModuleState[] swerveModuleActualStates = new SwerveModuleState[] { frontLeft.getState(),
                frontRight.getState(), backLeft.getState(), backRight.getState() };
        logData(swerveModuleActualStates);
    }

    private void logData(SwerveModuleState[] swerveModuleActualStates) {
        Logger.recordOutput("Drive/StatesActual", swerveModuleActualStates);
        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/PoseEstimate", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Heading", getHeading());
        Logger.recordOutput("Drive/Odometry/X", odometry.getPoseMeters().getX());
        Logger.recordOutput("Drive/Odometry/Y", odometry.getPoseMeters().getY());
        Logger.recordOutput("Drive/CurrentSupply/FrontLeftDrive", frontLeft.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontLeftTurn", frontLeft.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontRightDrive", frontRight.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontRightTurn", frontRight.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackLeftDrive", backLeft.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackLeftTurn", backLeft.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackRightDrive", backRight.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackRightTurn", backRight.getTurnCurrent());
    }

    // // wpk consider deleting after pose estimation is tested.
    // public Pose2d getPoseWithoutVision() {
    //     return odometry.getPoseMeters();
    // }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        // return poseEstimator.getEstimatedPosition();
    }

    // public Pose2d getPoseWithVision() {
    // return poseEstimator.getEstimatedPosition();
    // }

    public void resetOdometry(Pose2d pose) {
        double allienceOffset = 0;
        // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //     navX.setAngleAdjustment(180);
        // } else {
        //     navX.setAngleAdjustment(0);
        // }
        
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                allienceOffset = 180;
        }            

        navX.setAngleAdjustment(0);
        double currentHeading = navX.getAngle();
        double targetHeading = pose.getRotation().getDegrees() + allienceOffset;
        Logger.recordOutput("Drive/Auto/CurrentHeading", currentHeading);
        Logger.recordOutput("Drive/Auto/TargetHeading", targetHeading);
        Logger.recordOutput("Drive/Auto/AngleAdjustment", targetHeading - currentHeading);
        Logger.recordOutput("Drive/Auto/AngleAdjustmentV2", currentHeading - (targetHeading - currentHeading));

        navX.setAngleAdjustment(-(targetHeading - currentHeading));

        odometry.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);

        poseEstimator.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    // public void drive(double xSpeed, double ySpeed, double rot, boolean
    // fieldRelative) {
    // var swerveModuleDesiredStates =
    // DriveConstants.kinematics.toSwerveModuleStates(
    // fieldRelative
    // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
    // navX.getRotation2d())
    // : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates,
    // DriveConstants.MaxDriveableVelocity);
    // frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
    // frontRight.setDesiredState(swerveModuleDesiredStates[1]);
    // backLeft.setDesiredState(swerveModuleDesiredStates[2]);
    // backRight.setDesiredState(swerveModuleDesiredStates[3]);

    // Logger.recordOutput("Drive/StatesDesired", swerveModuleDesiredStates);

    // }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleDesiredStates = DriveConstants.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                navX.getRotation2d())
                        : ChassisSpeeds.discretize(
                                new ChassisSpeeds(xSpeed, ySpeed, rot),
                                DriveConstants.TimestepDurationInSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates,
                DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
        frontRight.setDesiredState(swerveModuleDesiredStates[1]);
        backLeft.setDesiredState(swerveModuleDesiredStates[2]);
        backRight.setDesiredState(swerveModuleDesiredStates[3]);

        Logger.recordOutput("Drive/StatesDesired", swerveModuleDesiredStates);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
                DriveConstants.TimestepDurationInSeconds);
        SwerveModuleState[] targetStates = DriveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
                Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void zeroHeading() {
        navX.setAngleAdjustment(0);
        navX.reset();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnDegPSec() {
        return navX.getRate() * (DriveConstants.GyroReversed ? -1.0 : 1.0); // degrees per second
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /* SIMULATION */

    public void simulationInit() {
        frontLeft.simulationInit();
        frontRight.simulationInit();
        backLeft.simulationInit();
        backRight.simulationInit();

        previousPositions[0] = frontLeft.getPosition();
        previousPositions[1] = frontRight.getPosition();
        previousPositions[2] = backLeft.getPosition();
        previousPositions[3] = backRight.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        var modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };

        var moduleDeltas = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
            var current = modulePositions[index];
            var previous = previousPositions[index];

            moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters,
                    current.angle);
            previous.distanceMeters = current.distanceMeters;
        }
        var twist = DriveConstants.kinematics.toTwist2d(moduleDeltas);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(navX.getAngle() - Units.radiansToDegrees(twist.dtheta));
    }

}
