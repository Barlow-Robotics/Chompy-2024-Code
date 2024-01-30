// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterPosition.ShooterPositionState;

public class RobotContainer {

    /* SUBSYSTEMS */

    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final ShooterPosition shooterPositionSub = new ShooterPosition();
    public final FloorIntake floorIntakeSub = new FloorIntake();

    /* COMMANDS */
    private final SetShooterPosition setShooterPosSpeakerCmd = new SetShooterPosition(shooterPositionSub, ShooterPositionState.Speaker);
    private final SetShooterPosition setShooterPosAmpCmd = new SetShooterPosition(shooterPositionSub, ShooterPositionState.Amp);
    private final SetShooterPosition setShooterPosSourceIntakeCmd = new SetShooterPosition(shooterPositionSub, ShooterPositionState.IntakeFromSource);
    private final SetShooterPosition setShooterPosFloorIntakeCmd = new SetShooterPosition(shooterPositionSub, ShooterPositionState.IntakeFromFloor);
    private final SetShooterPosition setShooterPosTrapCmd = new SetShooterPosition(shooterPositionSub, ShooterPositionState.Trap);

    private final StartShooting startShootingCmd = new StartShooting(shooterSub, shooterPositionSub);
    private final StopShooting stopShootingCmd = new StopShooting(shooterSub);
    
    private final StartIntake startIntakeCmd = new StartIntake(floorIntakeSub);
    private final StopIntake stopIntakeCmd = new StopIntake(floorIntakeSub);

    // private final Climb climbCmd = new Climb(shooterPositionSub);

    /* CONTROLLERS */
    Joystick driverController; 
    
    Joystick operatorController;
    
    /* BUTTONS */
    public Trigger shootButton;

    private Trigger toggleFloorIntakeButton;

    private Trigger climbButton;

    private Trigger moveToSpeakerButton;
    private Trigger moveToAmpButton;
    private Trigger moveToSourceButton;
    private Trigger moveToFloorButton;
    private Trigger moveToTrapButton;

    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AutoBuilder.configureHolonomic(
                driveSub::getPose, // Robot pose supplier
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

        if (DriverStation.getJoystickName(ElectronicIDs.DriverControllerPort).equals("Logitech Extreme 3D")) {
                driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX, 
                        true));
        } else {
                driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
                        true));
        }

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {

        driverController = new Joystick(ElectronicIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicIDs.OperatorControllerPort);
        
        /******************** SET SHOOTER POSITION ********************/

        moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper); // top
        moveToAmpButton.onTrue(setShooterPosAmpCmd);

        moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick); // right stick press
        moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        moveToFloorButton.onTrue(setShooterPosFloorIntakeCmd);
        
        moveToTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home
        moveToTrapButton.onTrue(setShooterPosTrapCmd);

        /******************** SHOOTER ********************/

        shootButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        shootButton.onTrue(startShootingCmd).onFalse(stopShootingCmd);

        /***************** FLOOR INTAKE *****************/
        toggleFloorIntakeButton = new JoystickButton(operatorController, LogitechDAConstants.LeftBumper); //floor 
        toggleFloorIntakeButton.onTrue(startIntakeCmd).onFalse(stopIntakeCmd);

        /******************** CLIMB ********************/
        
        climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonB);
        // climbButton.onTrue(climbCmd);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score Amp");
    }
}