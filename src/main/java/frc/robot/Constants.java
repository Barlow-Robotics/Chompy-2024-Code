// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double SecondsPerMinute = 60;

    public static final double Neo550MaxRPM = 11000;
    public static final double NeoMaxRPM = 5676;
    public static final double Falcon500MaxRPM = 6300;
    public static final double KrakenX60MaxRPM = 6000;

    public static final double toleranceLimit = 0.05;
    public static final double LowerToleranceLimit = 1 - toleranceLimit;
    public static final double UpperToleranceLimit = 1 + toleranceLimit;

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ElectronicsIDs {

        public static final int DriverControllerPort = 1;
        public static final int OperatorControllerPort = 2;

        /***************************** DRIVE *****************************/

        // Encoder = 1{locationOnBot}
        public static final int FrontLeftTurnEncoderID = 11;
        public static final int FrontRightTurnEncoderID = 12;
        public static final int BackLeftTurnEncoderID = 13;
        public static final int BackRightTurnEncoderID = 14;

        // DriveMotorID = 2{locationOnBot} // Base
        public static final int FrontLeftDriveMotorID = 21;
        public static final int FrontRightDriveMotorID = 22;
        public static final int BackLeftDriveMotorID = 23;
        public static final int BackRightDriveMotorID = 24;

        // TurnMotorID = 3{locationOnBot} // Side
        public static final int FrontLeftTurnMotorID = 31;
        public static final int FrontRightTurnMotorID = 32;
        public static final int BackLeftTurnMotorID = 33;
        public static final int BackRightTurnMotorID = 34;

        /***************************** SHOOTER *****************************/

        // ShooterMotorID = 4{locationOnBot}
        public static final int LowerShooterMotorID = 41;
        public static final int UpperShooterMotorID = 42;
        public static final int AngleMotorID = 43;
        public static final int IndexMotorID = 44;
        public static final int BreakBeamID = 4;

        /***************************** FLOOR INTAKE *****************************/

        // FloorMotorID = 5{locationOnBot}
        public static final int FloorMotorID = 51;

        /***************************** ELEVATOR *****************************/

        public static final int LeftElevatorMotorID = 61;
        public static final int RightElevatorMotorID = 62;

        public static final int BottomHallEffectID = 6;
        public static final int TopHallEffectID = 7;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {
        /** radians */
        public static final double MaxAngularSpeed = Math.PI; // 1/2 rotation per second

        public static final boolean GyroReversed = false;
        public static final double TrackWidth = 0.762;

        public static final double WheelBase = 0.762; // CHANGE - Distance between right and left wheels
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2),
                new Translation2d(WheelBase / 2, -TrackWidth / 2),
                new Translation2d(-WheelBase / 2, TrackWidth / 2),
                new Translation2d(-WheelBase / 2, -TrackWidth / 2));
            public static final boolean GyroReversed = false;
            public static final double TrackWidth = Units.inchesToMeters(27); // Distance between left and right wheels
            public static final double WheelBase = Units.inchesToMeters(25); // Distance between front and back wheels
            public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2), // front left 
                new Translation2d(WheelBase / 2, -TrackWidth / 2), // front right
                new Translation2d(-WheelBase / 2, TrackWidth / 2), // back left
                new Translation2d(-WheelBase / 2, -TrackWidth / 2) // back right
            );

        public static final double MaxAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxDriveableVelocity = 3.6; // m/s? (CHANGE if this is the wrong unit)

        public static final double PhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1); // 15.1 f/s from Mr. K's
                                                                                               // spreadsheet
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE

        public static final double FrontLeftMagnetOffsetInRadians = 1.5171039327979088;
        public static final double FrontRightMagnetOffsetInRadians = 1.7456666082143784;
        public static final double BackLeftMagnetOffsetInRadians = -2.7626938149333;
        public static final double BackRightMagnetOffsetInRadians = -2.305568464100361;

        public static final double TimestepDurationInSeconds = 0.02;
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = DriveConstants.PhysicalMaxSpeedMetersPerSecond / 4; // CHANGE
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.PhysicalMaxAngularSpeedRadiansPerSecond/ 10; // Default is 540 degress
        // public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAutoAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // default: 720 deg
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class SwerveConstants {
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        public static final double maxModuleSpeed = 4.5; // M/S

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                maxModuleSpeed,
                flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

        public static final double WheelRadius = Units.inchesToMeters(2.0);
        public static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
        public static final double GearRatio = 6.75;
        public static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute
                / GearRatio;

        public static final double MaxRPM = 5820;
        public static final double MaxVelocityPerSecond = MaxRPM * VelocityConversionFactor;

        /* DRIVE ENCODER */
        public static final double DriveKP = 0.04; // REQUIRES TUNING
        public static final double DriveKI = 0.0015;
        public static final double DriveKD = 0;
        public static final double DriveIZone = 0.15;
        public static final double DriveFF = 1.0 / MaxVelocityPerSecond;

        /* TURN ENCODER */
        public static final int CANCoderResolution = 4096;
        public static final double PositionConversionFactor = WheelCircumference / GearRatio;
        public static final double TurnKP = 1; // Need to change
        public static final double TurnKI = 0;
        public static final double TurnKD = 0;

        public static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per
                                                                                   // revolution (rad/sec)
        public static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterConstants {
        public static final double FlywheelGearRatio = 1; // CHANGE
        public static final double IndexGearRatio = 1; // CHANGE

        public static final double jKgMetersSquared = 0.0005;

        public static final double SpeakerRPM = 4000; // CHANGE
        public static final double AmpRPM = 2000; // CHANGE
        public static final double SourceRPM = -2000; // CHANGE
        public static final double FloorRPM = -1000; // CHANGE
        public static final double TrapRPM = 2000; // CHANGE

        public static final double ShooterKP = 0.5; // An error of 1 rotation/sec results in 2V output
        public static final double ShooterKI = 0.5; // An error of 1 rotation/sec increases output by 0.5V every sec
        public static final double ShooterKD = 0.0001; // A change of 1 rotation/sec squared results in 0.01V output // CHANGE ?
        public static final double ShooterKV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        public static final double PeakShooterForwardVoltage = 8; // Peak output of 8 volt
        public static final double PeakShooterReverseVoltage = -8;

        public static final double IndexRPM = 1000;

        public static final double IndexKP = 0.5; // CHANGE
        public static final double IndexKI = 0; // CHANGE
        public static final double IndexKD = 0; // CHANGE
        public static final double IndexIZone = 0; // CHANGE
        public static final double IndexFF = 0.12; // CHANGE
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterPositionConstants {

        public static final double ElevatorGearRatio = 15;
        public static final double ShooterAngleGearRatio = 46.67; // From K's spreadsheet

        public static final double ElevatorSprocketDiameter = Units.inchesToMeters(2.36);
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;

        public static final double ShooterAngleMaxSpeed = (Falcon500MaxRPM / 60 * 360) / ElevatorGearRatio; // deg/sec
        public static final double ElevatorMaxSpeed = (Falcon500MaxRPM / 60 / ElevatorGearRatio)
                * ElevatorSprocketCircumference; // m/s
        public static final double RotationsPerElevatorInch = ElevatorGearRatio
                / Units.metersToInches(ElevatorSprocketCircumference);

        public static final double ElevatorKP = 0.5;
        public static final double ElevatorKI = 0;
        public static final double ElevatorKD = 0;
        public static final double ElevatorIZone = 0;
        public static final double ElevatorFF = 0.12;

        public static final double AngleKP = 0.5;
        public static final double AngleKI = 0;
        public static final double AngleKD = 0;
        public static final double AngleIZone = 0;
        public static final double AngleFF = 0.12;

        public static final double SpeakerAngle = 30; // CHANGE
        public static final double SpeakerHeight = 0; // CHANGE

        public static final double AmpAngle = 60; // CHANGE
        public static final double AmpHeight = 10; // CHANGE

        public static final double SourceIntakeAngle = 90; // CHANGE
        public static final double SourceIntakeHeight = 0; // CHANGE

        public static final double FloorIntakeAngle = 120; // CHANGE
        public static final double FloorIntakeHeight = 0; // CHANGE

        public static final double TrapAngle = 150; // CHANGE
        public static final double TrapHeight = 10; // CHANGE
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class FloorIntakeConstants {
        // upper = 1_ | lower = 2_

        /* PID CONTROLLER */
        public static final double KP = 0.5; // CHANGE
        public static final double KI = 0.1; // CHANGE
        public static final double KD = 0.1; // CHANGE
        public static final double IZone = 0.1; // CHANGE
        public static final double FF = 1; // CHANGE

        public static final double MotorVelocity = 2000; // CHANGE
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public final class LogitechDAConstants {
        public static final int LeftStickX = 0; // LDA = Logitech Dual Action
        public static final int LeftStickY = 1;
        public static final int RightStickX = 2;
        public static final int RightStickY = 3;
        public static final int LeftTrigger = 7; // Speaker
        public static final int RightTrigger = 8; // Amp
        public static final int ButtonA = 2; // Move Source
        public static final int ButtonB = 3; // Trapdoor
        public static final int ButtonX = 1; // Move Floor
        public static final int ButtonY = 4; // Move Trap
        public static final int LeftBumper = 5; // Floor Intake
        public static final int RightBumper = 6; // Source Intake
        public static final int BackButton = 9; // Climb
        public static final int StartButton = 10;
        public static final int LeftStick = 11; // Move Speaker
        public static final int RightStick = 12; // Move Amp
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class RadioMasterConstants {
        public static final int LeftGimbalX = 0;
        public static final int LeftGimbalY = 1;
        public static final int RightGimbalX = 3;
        public static final int RightGimbalY = 2;
        public static final int SliderF = 5;
        public static final int SliderE = 4;
        public static final int SliderC = 6;
        public static final int ButtonD = 2;
        public static final int ButtonA = 1;
        public static final double FowardAxisAttenuation = 1.0;
        public static final double LateralAxisAttenuation = 1.0;
        public static final double YawAxisAttenuation = 0.6;
    }

    public final class XboxControllerConstants {
        public static final int LeftStickX = 0;
        public static final int LeftStickY = 1;
        public static final int LeftTrigger = 2;
        public static final int RightTrigger = 4;
        public static final int RightStickX = 4;
        public static final int RightStickY = 5;
        // Angle Trap?
        public static final int ButtonA = 1; // Shoot Trap
        public static final int ButtonB = 2; // Climb
        public static final int ButtonX = 3; // Floor Intake
        public static final int ButtonY = 4; // Angle Speaker
        public static final int LeftBumper = 5; // Shoot Amp
        public static final int RightBumper = 6; // Shoot Speaker
        // public static final int BackButton = 7;
        public static final int StartButton = 8; // Angle Amp
        public static final int LeftStick = 9; // Angle Source
        public static final int RightStick = 10; // Source Intake
        public static final int WindowButton = 7; // Angle Floor

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}
