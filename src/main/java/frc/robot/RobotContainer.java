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
import frc.robot.Constants.ElectronicsIDs;
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
    public final Vision visionSub = new Vision(); 

    /* COMMANDS */
    private final SetMouthPosition setShooterPosSpeakerCmd = new SetMouthPosition(shooterPositionSub, ShooterPositionState.Speaker, visionSub);
    private final SetMouthPosition setShooterPosAmpCmd = new SetMouthPosition(shooterPositionSub, ShooterPositionState.Amp, visionSub);
    private final SetMouthPosition setShooterPosSourceIntakeCmd = new SetMouthPosition(shooterPositionSub, ShooterPositionState.SourceIntake, visionSub);
    private final SetMouthPosition setShooterPosFloorIntakeCmd = new SetMouthPosition(shooterPositionSub, ShooterPositionState.FloorIntake, visionSub);
    private final SetMouthPosition setShooterPosTrapCmd = new SetMouthPosition(shooterPositionSub, ShooterPositionState.Trap, visionSub);

    private final StartShooterIntake startShootingCmd = new StartShooterIntake(shooterSub, floorIntakeSub, shooterPositionSub);
    private final StopShooterIntake stopShootingCmd = new StopShooterIntake(shooterSub, floorIntakeSub);
    
    private final StartIntake startIntakeCmd = new StartIntake(floorIntakeSub);
    private final StopIntake stopIntakeCmd = new StopIntake(floorIntakeSub);

    private final DriveRobotWithAlign driveRobotWithAlignCmd;

    // private final Climb climbCmd = new Climb(shooterPositionSub);

    /* CONTROLLERS */

    Joystick driverController; 
    Joystick operatorController;
    
    /* BUTTONS */
    private Trigger moveToSpeakerButton;
    private Trigger moveToAmpButton;
    private Trigger moveToSourceButton;
    private Trigger moveToFloorButton;
    private Trigger moveToTrapButton;

    public Trigger shootButton;
    public Trigger autoAlignButton;

    private Trigger toggleFloorIntakeButton;

    private Trigger climbButton;

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

        if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Extreme 3D")) {
                driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX, 
                        true));
                driveRobotWithAlignCmd = new DriveRobotWithAlign(
                    driveSub,
                    driverController,
                    LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX,
                    true,
                    visionSub,
                    autoAlignButton);
        } else {
                driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
                        true));
                driveRobotWithAlignCmd = new DriveRobotWithAlign(
                    driveSub,
                    driverController,
                    RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
                    true,
                    visionSub,
                    autoAlignButton);
        }

        // We can't do this during configureBindings because driveController doesn't get created until
        // the above if-statement.
        autoAlignButton.whileTrue(driveRobotWithAlignCmd);

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

        shootButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home 
        shootButton.onTrue(startShootingCmd).onFalse(stopShootingCmd);

        /***************** FLOOR INTAKE *****************/
        
        toggleFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); // floor 
        toggleFloorIntakeButton.onTrue(startIntakeCmd).onFalse(stopIntakeCmd);

        /******************** CLIMB ********************/
        
        // climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); // no button on mantis controller
        // climbButton.onTrue(climbCmd);

        /***************** AUTO ALIGN ******************/
        autoAlignButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score Amp");
    }
}