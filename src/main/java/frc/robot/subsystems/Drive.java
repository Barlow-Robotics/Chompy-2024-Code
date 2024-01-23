// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicIDs;
import org.littletonrobotics.junction.Logger;
import java.lang.Math;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;


public class Drive extends SubsystemBase {

    /*********************************************************************/
    /***************************** CONSTANTS *****************************/

    public static final double MaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final boolean GyroReversed = false;
    public static final double kTrackWidth = 0.762;

    public static final double kWheelBase = 0.762; // Distance between right and left wheels
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
    
    /*******************************************************************************/
    /*******************************************************************************/

    private final SwerveModule frontLeft = new SwerveModule(
            "frontLeft",
            ElectronicIDs.FrontLeftDriveMotorID,
            ElectronicIDs.FrontLeftTurnMotorID,
            ElectronicIDs.FrontLeftTurnEncoderID,
            Math.toDegrees(1.5171039327979088) / 360.0, 
            false
    );

    private final SwerveModule frontRight = new SwerveModule(
            "frontRight",
            ElectronicIDs.FrontRightDriveMotorID,
            ElectronicIDs.FrontRightTurnMotorID,
            ElectronicIDs.FrontRightTurnEncoderID,
            Math.toDegrees(1.7456666082143784) / 360.0, 
            true
    );

    private final SwerveModule backLeft = new SwerveModule(
            "backLeft",
            ElectronicIDs.BackLeftDriveMotorID,
            ElectronicIDs.BackLeftTurnMotorID,
            ElectronicIDs.BackLeftTurnEncoderID,
            Math.toDegrees(-2.7626938149333) / 360.0, 
            false
    );

    private final SwerveModule backRight = new SwerveModule(
            "backRight",
            ElectronicIDs.BackRightDriveMotorID,
            ElectronicIDs.BackRightTurnMotorID,
            ElectronicIDs.BackRightTurnEncoderID,
            Math.toDegrees(-2.305568464100361) / 360.0,
            true
    );

    private AHRS navX;

    private final SwerveDriveOdometry odometry;

    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[4] ;


    public Drive() {
        navX = new AHRS(Port.kMXP);
        navX.reset();

        odometry = new SwerveDriveOdometry(
            kinematics,
            navX.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    
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

        Logger.recordOutput("Pose", odometry.getPoseMeters());
        SwerveModuleState[] swerveModuleActualStates = new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
        Logger.recordOutput("SwerveStates/ActualStates", swerveModuleActualStates);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    public void scoreAmp() {
        System.out.println("*******************************Shot in amp");
        return;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        fieldRelative = false ;  // wpk fix this later
        var swerveModuleDesiredStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                navX.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates, DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
        frontRight.setDesiredState(swerveModuleDesiredStates[1]);
        backLeft.setDesiredState(swerveModuleDesiredStates[2]);
        backRight.setDesiredState(swerveModuleDesiredStates[3]);

        Logger.recordOutput("SwerveStates/DesiredStates", swerveModuleDesiredStates);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void zeroHeading() {
        navX.reset();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return navX.getRate() * (GyroReversed ? -1.0 : 1.0); // degrees per second
    }

    public SwerveModuleState[] getModuleStates() {

        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };

        // SwerveModuleState[] states = new SwerveModuleState[modules.length];
        // for (int i = 0; i < modules.length; i++) {
        //   states[i] = modules[i].getState();
        // }
        // return states;
      }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

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
            } ;
    
        var moduleDeltas = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
                var current = modulePositions[index];
                var previous = previousPositions[index];

                moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
        }
        var twist = kinematics.toTwist2d(moduleDeltas);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(navX.getAngle() - Units.radiansToDegrees(twist.dtheta));
    }    
    
}
