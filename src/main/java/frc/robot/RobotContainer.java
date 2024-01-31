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
import frc.robot.commands.DriveRobot;
import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.SetShooterPosition;
import frc.robot.commands.Climb;

import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* SUBSYSTEMS */
    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final ShooterPosition shooterPositionSub = new ShooterPosition();
    public final FloorIntake floorIntakeSub = new FloorIntake();

    /* COMMANDS */
    private final SetShooterVelocity startShooterSpeakerCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Speaker);
    private final SetShooterVelocity startShooterAmpCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Amp);
    private final SetShooterVelocity startShooterSourceIntakeCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.IntakeFromSource);
    // private final SetShooterVelocity startShooterFloorIntakeCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.IntakeFromFloor);
    private final SetShooterVelocity startShooterTrapCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Trap);
    private final SetShooterVelocity stopShooterCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Stopped);
    private final ToggleIntake toggleIntakeCmd = new ToggleIntake(floorIntakeSub);

    private final SetShooterPosition setShooterSpeakerAngleCmd = new SetShooterPosition(shooterPositionSub, shooterPositionSub.shooterAngleState.Speaker);
    private final SetShooterPosition setShooterAmpAngleCmd = new SetShooterPosition(shooterPositionSub, shooterPositionSub.shooterAngleState.Amp);
    private final SetShooterPosition setShooterSourceAngleCmd = new SetShooterPosition(shooterPositionSub, shooterPositionSub.shooterAngleState.IntakeFromSource);
    private final SetShooterPosition setShooterFloorAngleCmd = new SetShooterPosition(shooterPositionSub, shooterPositionSub.shooterAngleState.IntakeFromFloor);
    private final SetShooterPosition setShooterTrapAngleCmd = new SetShooterPosition(shooterPositionSub, shooterPositionSub.shooterAngleState.Trap);
    
    private final Climb climbCmd = new Climb(shooterPositionSub);
    /* CONTROLLERS */
    Joystick driverController; 
    
    Joystick operatorController;
    
    /* BUTTONS */
   
    private Trigger shootSpeakerButton;
    private Trigger shootAmpButton;
    private Trigger shooterSourceIntakeButton;
    private Trigger shooterFloorIntakeButton;
    private Trigger shootTrapButton;

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

        NamedCommands.registerCommand("Shoot Speaker", Commands.print("***********************************Shoot into Speaker"));
        NamedCommands.registerCommand("Shoot Amp", Commands.print("*******************************Shoot into Amp"));

        configureBindings();

        if(DriverStation.getJoystickName(Constants.DriverControllerPort).equals("Logitech Extreme 3D")) {
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

        driverController = new Joystick(Constants.DriverControllerPort);
        operatorController = new Joystick(Constants.OperatorControllerPort);
        // ***************** SHOOTING *****************
        shootSpeakerButton = new JoystickButton(operatorController, LogitechDAConstants.LeftTrigger); // middle 
        // shootSpeakerButton.onTrue(setShooterSpeakerAngleCmd);
        shootSpeakerButton.onTrue(startShooterSpeakerCmd).onFalse(stopShooterCmd);

        shootAmpButton = new JoystickButton(operatorController, LogitechDAConstants.RightTrigger); // top
        // shootAmpButton.onTrue(setShooterAmpAngleCmd);
        shootAmpButton.onTrue(startShooterAmpCmd).onFalse(stopShooterCmd);

        shooterSourceIntakeButton = new JoystickButton(operatorController, LogitechDAConstants.RightBumper); // right stick press
        // shooterSourceIntakeButton.onTrue(setShooterSourceAngleCmd);
        shooterSourceIntakeButton.onTrue(startShooterSourceIntakeCmd).onFalse(stopShooterCmd);

        // shooterFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        // shooterFloorIntakeButton.onTrue(setShooterFloorAngleCmd);
        // shooterFloorIntakeButton.onTrue(startShooterFloorIntakeCmd).onFalse(stopShooterCmd);
        
        shootTrapButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonB); // home
        // shootTrapButton.onTrue(setShooterTrapAngleCmd);
        shootTrapButton.onTrue(startShooterTrapCmd).onFalse(stopShooterCmd);

        // ***************** MISC *****************
        // toggleFloorIntakeButton = new JoystickButton(operatorController, LogitechDAConstants.LeftBumper); //floor
        toggleFloorIntakeButton = new JoystickButton(operatorController, LogitechDAConstants.LeftBumper); //floor 
        toggleFloorIntakeButton.onTrue(toggleIntakeCmd);
        
        climbButton = new JoystickButton(operatorController, LogitechDAConstants.BackButton);
        climbButton.onTrue(climbCmd);

        // ***************** ANGLEING *****************
        moveToSpeakerButton = new JoystickButton(operatorController, LogitechDAConstants.LeftStick);
        moveToSpeakerButton.onTrue(setShooterSpeakerAngleCmd);
        moveToAmpButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick);
        moveToAmpButton.onTrue(setShooterAmpAngleCmd);
        moveToSourceButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonA);
        moveToSourceButton.onTrue(setShooterSourceAngleCmd);
        moveToFloorButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonX);
        moveToFloorButton.onTrue(setShooterFloorAngleCmd);
        moveToTrapButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonY);
        moveToTrapButton.onTrue(setShooterTrapAngleCmd);


    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score in Amp Speaker Speaker V1 (SASS)");
    }
}