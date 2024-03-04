// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

public class Constants {

    public static final double SecondsPerMinute = 60;
    public static final double jKgMetersSquared = 0.0005;
    public static final double Neo550MaxRPM = 11000;
    public static final double NeoMaxRPM = 5820;
    public static final double Falcon500MaxRPM = 6300;
    public static final double KrakenX60MaxRPM = 6000;

    public static final double TalonFXResolution = 2048;
    public static final double CANCoderResolution = 4096;

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ElectronicsIDs {

        public static final int DriverControllerPort = 1;
        public static final int OperatorControllerPort = 2;

        /***************************** DRIVE *****************************/

        // CANCoder = 1{locationOnBot}
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
        public static final int LeftShooterMotorID = 41;
        public static final int RightShooterMotorID = 42;
        public static final int IndexMotorID = 43;
        public static final int BreakBeamID = 4;

        /***************************** ELEVATOR *****************************/

        public static final int LeftElevatorMotorID = 51;
        public static final int RightElevatorMotorID = 52;
        public static final int AngleMotorID = 53;
        public static final int AngleEncoderID = 54;

        public static final int BottomHallEffectID = 5;
        // public static final int TopHallEffectID = 6;

        /***************************** FLOOR INTAKE *****************************/

        // FloorMotorID = 5{locationOnBot}
        public static final int FloorMotorID = 61;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {

        public static final boolean GyroReversed = false;

        public static final double TrackWidth = Units.inchesToMeters(22); // Distance between left and right wheels
        public static final double WheelBase = Units.inchesToMeters(20); // Distance between front and back wheels
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2), // front left
                new Translation2d(WheelBase / 2, -TrackWidth / 2), // front right
                new Translation2d(-WheelBase / 2, TrackWidth / 2), // back left
                new Translation2d(-WheelBase / 2, -TrackWidth / 2) // back right
        );

        public static final double WheelRadius = Units.inchesToMeters(2.0);
        public static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
        public static final double GearRatio = 6.12;

        public static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute / GearRatio;

        public static final double MaxAngularRadiansPerSecond = Math.PI; // 1/2 rotation per second
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE

        public static final double MaxAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxDriveableVelocity = 4.5; // m/s
        public static final double PhysicalMaxMetersPerSecond =  NeoMaxRPM * VelocityConversionFactor;
 
        public static final double FrontLeftMagnetOffsetInRadians = 1.5171039327979088;
        public static final double FrontRightMagnetOffsetInRadians = 1.7456666082143784;
        public static final double BackLeftMagnetOffsetInRadians = -2.7626938149333;
        public static final double BackRightMagnetOffsetInRadians = -2.305568464100361;

        public static final double TimestepDurationInSeconds = 0.02;
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        /* DRIVE ENCODER */
        public static final double DriveKP = 0.04; // REQUIRES TUNING
        public static final double DriveKI = 0.0015;
        public static final double DriveKD = 0;
        public static final double DriveIZone = 0.15;
        public static final double DriveFF = 1.0 / PhysicalMaxMetersPerSecond;

        public static final double AutoAlignRotKP = 0.08; //CHANGE
        public static final double AutoAlignRotKI = 0.0;
        public static final double AutoAlignRotKD = 0.0;

        public static final double AutoAlignLatKP = 0.5; //CHANGE
        public static final double AutoAlignLatKI = 0.0;
        public static final double AutoAlignLatKD = 0;

        /* TURN ENCODER */
        public static final int CANCoderResolution = 4096;
        public static final double PositionConversionFactor = WheelCircumference / GearRatio;
        public static final double TurnKP = 1; // Need to change
        public static final double TurnKI = 0;
        public static final double TurnKD = 0;

        public static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per revolution (rad/sec)
        public static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared
        public static final double MaxModuleMetersPerSecond = 4.5;

        public static final int StallLimit = 40;
        public static final int FreeLimit = 40;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0, 0), // Translation constants
            new PIDConstants(5.0, 0, 0), // Rotation constants
            MaxModuleMetersPerSecond,
            flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
            new ReplanningConfig());
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = DriveConstants.MaxModuleMetersPerSecond / 4; // CHANGE
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.PhysicalMaxAngularSpeedRadiansPerSecond
                / 10; // Default is 540 degress
        // public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // default: 720 deg
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterConstants {
       
        public static final double SupplyCurrentLimit = 30;

        /* FLYWHEELS */

        public static final double FlywheelVelocityTolerance = 25;

        public static final double FlywheelGearRatio = 1.5; // 36 gears on motor, 24 on rollers --> 1.5:1 (as of 2/15)

        public static final double LeftSpeakerRPM = 2500;
        public static final double RightSpeakerRPM = 2500;

        public static final double LeftAmpRPM = 500; 
        public static final double RightAmpRPM = 500; 

        public static final double LeftIntakeRPM = -1000;
        public static final double RightIntakeRPM = -1000;

        public static final double LeftTrapRPM = 250;
        public static final double RightTrapRPM = 250;

        public static final double FlywheelLeftKP = 0.25; 
        public static final double FlywheelLeftKI = 0; 
        public static final double FlywheelLeftKD = 0; 
        public static final double FlywheelLeftFF = 0.13; 

        public static final double FlywheelRightKP = 0.25; 
        public static final double FlywheelRightKI = 0; 
        public static final double FlywheelRightKD = 0; 
        public static final double FlywheelRightFF = 0.13; 

        /* INDEX */

        public static final double IndexVelocityTolerance = 5;

        public static final double IndexGearRatio = 1; // 1:1 ratio on Index per WK as of 2/15

        public static final double IndexRPM = 500; //750

        public static final double IndexKP = 0.15; //0.25
        public static final double IndexKI = 0; 
        public static final double IndexKD = 0; 
        public static final double IndexFF = 0.5; //0.2
        public static final double IndexKS = 0.05;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterMountConstants {

        public static final double SupplyCurrentLimit = 40;       
       
        /* HEIGHT / ANGLE PAIRS */

        public static final double AngleTolerance = 1.5; 
        public static final double HeightTolerance = 0.25;

        public static final double MaxAngleDegrees = 45;
        public static final double MinAngleDegrees = -63.5;
        public static final double MaxHeightInches = 47.5;
        public static final double StartingHeight = 19.75;  

        public static final double SpeakerAngle = 44; 
        public static final double SpeakerHeight = StartingHeight; 

        public static final double AmpAngle = -30;
        public static final double AmpHeight = 40;

        public static final double SourceIntakeAngle = 32; 
        public static final double SourceIntakeHeight = 27; 

        public static final double FloorIntakeAngle = MinAngleDegrees; 
        public static final double FloorIntakeHeight = StartingHeight; 

        public static final double TrapAngle = 0; // CHANGE
        public static final double TrapHeight = 10; // CHANGE

        /* ANGLE */

        public static final double AngleKP = 26;
        public static final double AngleKI = 0;
        public static final double AngleKD = 0.0;
        // public static final double AngleIZone = 0; // motor already does this
        public static final double AngleFF = 0.0;
        public static final double AngleKG = 0.29;

        public static final double AngleCANCoderMagnetOffset = 0.312744140625;

        public static final double AngleGearRatio = 46.67; // From K's spreadsheet
        public static final double ShooterAngleMaxDegreesPerSecond = (Falcon500MaxRPM / SecondsPerMinute * 360) / AngleGearRatio;
        public static final double AngleMMCruiseDegPerSec = 1.5; 
        public static final double AngleMMAcceleration = 3;
        public static final double AngleMMJerk = 20; //30; 

        /* ELEVATOR */

        public static final double ElevatorKP = 32;
        public static final double ElevatorKI = 0.001;
        public static final double ElevatorKD = 0.0;
        // public static final double ElevatorIZone = 0.1; // motor already does this
        public static final double ElevatorFF = 0.0;
        public static final double ElevatorKG = 2.7;
        public static final double ElevatorKS = 0; // use error sign instead of velocity sign? as part of initial config 

        public static final double ElevatorGearRatio = 15;
        public static final double ElevatorSprocketDiameter = 2.36;  // inches
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;
        public static final double RotationsPerElevatorInch = 1 / ElevatorSprocketCircumference * ElevatorGearRatio;
        // public static final double RotationsPerElevatorInch = 
        // ElevatorGearRatio / Units.metersToInches(ElevatorSprocketCircumference) / 2;

        public static final double ElevatorMaxInchesPerSec = 
                Falcon500MaxRPM / SecondsPerMinute / ElevatorGearRatio * ElevatorSprocketCircumference;
        public static final double ElevatorMMCruiseInchesPerSec = 10; 
        public static final double ElevatorMMInchesPerSecPerSec = 10; 
        public static final double ElevatorMMJerk = 800; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)
    
        // LMT added constants to enable changing shooter angle while driving to speaker
        public static final double CameraMountHeight = 24; // inches - possibly CHANGE
        public static final double CameraMountAngle = 3; // degrees - possibly CHANGE
        public static final double SpeakerAprilTagHeight = 52; /*inches - possibly CHANGE - is this the bottom of the
                                                            * AT? Might need to change to midpoint for the calc to
                                                            * work, originally did that from bottoms of speaker and
                                                            * AT */
        public static final double MidSpeakerHeight = 80.4; // inches to middle of speaker hole - possibly CHANGE
        public static final double ElevatorHeightUnextended = 26; // inches - possibly CHANGE - height of elevator at rest

    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class FloorIntakeConstants {
        /* PID CONTROLLER */
        public static final double KP = 0.2; 
        public static final double KI = 0; 
        public static final double KD = 0.01; 
        public static final double IZone = 0; 
        // public static final double FF = 1 / KrakenX60MaxRPM / 60; 
        public static final double FF = 0.13; // KV

        public static final int SupplyCurrentLimit = 20;
		
        public static final double MotorRPM = 2500;
        public static final double VelocityTolerance = 5;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class VisionConstants {
        public static final int CameraLightID = 0; // Need to change
        public static final String PoseCameraName = "Global_Shutter_Camera";
        public static final String TargetCameraName = "Arducam_OV9281_USB_Camera";

        public static final PoseStrategy PrimaryVisionStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
        public static final PoseStrategy FallbackVisionStrategy = PoseStrategy.LOWEST_AMBIGUITY;

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        // wpk need to update these to be more exact.
        public static final Transform3d PoseCameraToRobot =
                new Transform3d(
                    new Translation3d(0.0, Units.inchesToMeters(DriveConstants.TrackWidth/2), Units.inchesToMeters(23)), 
                    new Rotation3d(0, Units.degreesToRadians(5), 0));
        public static final Transform3d RobotToPoseCamera = PoseCameraToRobot.inverse();

        public static final Transform3d TargetCamToRobot =
                new Transform3d(
                    new Translation3d(0.0, -Units.inchesToMeters(DriveConstants.TrackWidth/2), Units.inchesToMeters(23)), 
                    new Rotation3d(0, Units.degreesToRadians(5), 0));
        public static final Transform3d RobotToTargetCam = TargetCamToRobot.inverse();

        // // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout FieldTagLayout =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        // constants for vision-calculated speaker shooting - LMT
        public static final int RedSpeakerCenterAprilTagID = 4;
        public static final int BlueSpeakerCenterAprilTagID = 7;
        public static final int NullAprilTagID = -1;
        public static final double InvalidAngle = -361; 
        public static final double NoTargetDistance = -1;

    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

     public static final class UnderGlowConstants {
        public static final SerialPort.Port Port = SerialPort.Port.kUSB1;
        public static final int BlueAlliance = 0; // 0b00000000
        public static final int RedAlliance = 1; // 0b00000001
        public static final int Enabled = 2; // 0b00000010
        public static final int NoteLoaded = 4; // 0b00000100
        public static final int Shooting = 8; // 0b00001000
        public static final int Auto = 16; // 0b00010000
        public static final int RobotFloorSource = 32; // 0b00100000
        public static final int Climbing = 64; // 0b01000000
        public static final int RobotSource = 128; // 0b10000000
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public final class LogitechDAConstants {
        public static final int LeftStickX = 0; 
        public static final int LeftStickY = 1;
        public static final int RightStickX = 2;
        public static final int RightStickY = 3;
        public static final int LeftTrigger = 7; 
        public static final int RightTrigger = 8; 
        public static final int ButtonA = 2; 
        public static final int ButtonB = 3; 
        public static final int ButtonX = 1; 
        public static final int ButtonY = 4; 
        public static final int LeftBumper = 5; 
        public static final int RightBumper = 6; 
        public static final int BackButton = 9; 
        public static final int StartButton = 10;
        public static final int LeftStick = 11;
        public static final int RightStick = 12; 
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class LogitechExtreme3DConstants {
        public static final int AxisX = 0; 
        public static final int AxisY = 1;
        public static final int AxisZRotate = 2; 
        public static final int Slider = 3; 
        public static final int Trigger = 1; 
        public static final int ButtonStick = 2; 
        public static final int Button3 = 3; 
        public static final int Button4 = 4; 
        public static final int Button5 = 5; 
        public static final int Button6 = 6;
        public static final int Button7 = 7; 
        public static final int Button8 = 8;
        public static final int Button9 = 9; 
        public static final int Button10 = 10;
        public static final int Button11 = 11;
        public static final int Button12 = 12;
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
        public static final int ButtonA = 1; 
        public static final int ButtonB = 2; 
        public static final int ButtonX = 3; 
        public static final int ButtonY = 4; 
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6; 
        // public static final int BackButton = 7;
        public static final int HamburgerButton = 8; 
        public static final int LeftStick = 9; 
        public static final int RightStick = 10; 
        public static final int WindowButton = 7;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}
