// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElectronicsIDs;
//import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.LogitechExtreme3DConstants;
//import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class RobotContainer {

    /* SUBSYSTEMS */

    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final ShooterMount shooterMountSub = new ShooterMount();
    public final FloorIntake floorIntakeSub = new FloorIntake();
    public final Vision visionSub = new Vision(); 

    /* COMMANDS */
    private final SetShooterMountPosition setShooterPosSpeakerCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.Speaker);
    private final SetShooterMountPosition setShooterPosAmpCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.Amp);
    private final SetShooterMountPosition setShooterPosSourceIntakeCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.SourceIntake);
    public final SetShooterMountPosition setShooterPosFloorIntakeCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.FloorIntake);
    private final SetShooterMountPosition setShooterPosTrapCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.Trap);

    private final StartShooterIntake startShooterIntakeCmd = new StartShooterIntake(shooterSub, floorIntakeSub, shooterMountSub);
    private final StopShooterIntake stopShooterIntakeCmd = new StopShooterIntake(shooterSub, floorIntakeSub);
    
    // private final Climb climbCmd = new Climb(shooterPositionSub);
    // LT Climb
    private final SetShooterMountPosition prepareToClimbCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.PreClimb);
    private final SetShooterMountPosition climbCmd = new SetShooterMountPosition(shooterMountSub, ShooterMountState.Climb);
    

    /* CONTROLLERS */

    Joystick driverController; 
    Joystick operatorController;
    
    /* BUTTONS */
    private Trigger moveToSpeakerButton;
    private Trigger moveToAmpButton;
    private Trigger moveToSourceButton;
    private Trigger moveToFloorButton;
    private Trigger moveToTrapButton;

    private Trigger prepareToClimbButton;   // LT added.  CHANGE if not its own buttun
    public Trigger shootIntakeButton;

    private Trigger climbButton;

    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AutoBuilder.configureHolonomic(
                // driveSub::getPoseWithoutVision, // Robot pose supplier
                driveSub::getPoseWithVision, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveSub::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                driveSub
        );

        NamedCommands.registerCommand("Leave Area", Commands.print("***********************************Left Area"));
        NamedCommands.registerCommand("Shoot Amp", Commands.print("*******************************Shot in amp"));

        configureBindings();

        // if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Extreme 3D")) {
                driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        LogitechExtreme3DConstants.AxisX, LogitechExtreme3DConstants.AxisY, LogitechExtreme3DConstants.AxisZRotate, 
                        true));
        // } else if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Radiomaster TX12 Joystick")){
                // driveSub.setDefaultCommand(
                // new DriveRobot(
                //         driveSub, 
                //         driverController, 
                //         RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
                //         true));
        // } else if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Dual Action")){
        //         driveSub.setDefaultCommand(
        //         new DriveRobot(
        //                 driveSub, 
        //                 driverController, 
        //                 LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX, 
        //                 true));
        // } else {
        //     System.out.println("Unknown controller");
        // }

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {

        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);
        /******************** SET SHOOTER POSITION ********************/

        moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper); // top
        moveToAmpButton.onTrue(setShooterPosAmpCmd);

        moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick); // station
        moveToFloorButton.onTrue(setShooterPosFloorIntakeCmd);
        
        moveToTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonB); // no button on mantis controller
        moveToTrapButton.onTrue(setShooterPosTrapCmd);

        /******************** SHOOTER ********************/

        shootIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home 
        shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        /***************** FLOOR INTAKE *****************/
        
        // toggleFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); // floor 
        // toggleFloorIntakeButton.onTrue(startIntakeCmd).onFalse(stopIntakeCmd);

        /******************** CLIMB ********************/
        prepareToClimbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); // no button on mantis controller.  CHANGE button binding
        prepareToClimbButton.onTrue(prepareToClimbCmd);

        climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); // no button on mantis controller.  CHANGE button binding
        climbButton.onTrue(climbCmd);
        // climbButton.onTrue(climbCmd);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score Amp");
    }
}