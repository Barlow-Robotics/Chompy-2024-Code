// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final double SecondsPerMinute = 60;

    public static final class CanIDs {
        
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
        public static final int LeftShooterMotorID = 41;
        public static final int RightShooterMotorID = 42;
        public static final int AngleMotorID = 43;

        /***************************** FLOOR INTAKE *****************************/
        // FloorMotorID = 5{locationOnBot}
        public static final int upperFloorMotorID = 51;
        public static final int lowerFloorMotorID = 52;
        public static final int floorBreakBeamID = 0; // CHANGE

        public static final int upperFloorEncoderID = 11;
        public static final int lowerFloorEncoderID = 21;

        public static final int upperFloorPIDControllerID = 11;
        public static final int lowerFloorPIDControllerID = 21;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {
        public static final double PhysicalMaxSpeedMetersPerSecond = 4.0; // CHANGE
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = DriveConstants.PhysicalMaxSpeedMetersPerSecond / 4; // CHANGE
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.PhysicalMaxAngularSpeedRadiansPerSecond
                / 10; // Default is 540 degress
        public static final double MaxAccelerationMetersPerSecondSquared = 3; // CHANGE
        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // default is 720 degrees
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class Swerve {
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
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterConstants {

    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class FloorIntakeConstants {
        // upper = 1_ | lower = 2_

        /* UPPER PID CONTROLLER */
        public static final double UpperKP = 0.5; // CHANGE
        public static final double UpperKI = 0; // CHANGE
        public static final double UpperKD = 0; // CHANGE
        public static final double UpperIZone = 0; // CHANGE
        public static final double UpperFF = 1; // CHANGE

        /* LOWER PID CONTROLLER */
        public static final double LowerKP = 0.5; // CHANGE
        public static final double LowerKI = 0; // CHANGE
        public static final double LowerKD = 0; // CHANGE
        public static final double LowerIZone = 0; // CHANGE
        public static final double LowerFF = 1; // CHANGE

        public static final double MotorSpeed = 0; // CHANGE
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ElevatorConstants {

    }
    public final class LogitechConstants {
        public static final int LDALeftStickX = 0; // LDA = Logitech Dual Action
        public static final int LDALeftStickY = 1;
        public static final int LDARightStickX = 2;
        public static final int LDARightStickY = 3;
        public static final int LDALeftTrigger = 7; // Speaker
        public static final int LDARightTrigger = 8; // Amp
        public static final int LDAButtonA = 2; 
        public static final int LDAButtonB = 3; // Trapdoor
        public static final int LDAButtonX = 1;
        public static final int LDAButtonY = 4;
        public static final int LDALeftBumper = 5; // Floor Intake
        public static final int LDARightBumper = 6; // Source Intake
        public static final int LDABackButton = 9;
        public static final int LDAStartButton = 10;
        public static final int LDALeftStick = 11;
        public static final int LDARightStick = 12;
        public static final double LDAForwardAxisAttenuation = -0.5;
        public static final double LDALateralAxisAttenuation = 0.5;
        public static final double LDAYawAxisAttenuation = 0.5;
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
        public static final int StartButton = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;
        public static final int WindowButton = 7;

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}
