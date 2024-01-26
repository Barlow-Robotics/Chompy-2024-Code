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

import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* SUBSYSTEMS */
    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final ShooterAngle shooterAngleSub = new ShooterAngle();
    public final Elevator elevatorSub = new Elevator();
    public final FloorIntake floorIntakeSub = new FloorIntake();

    /* COMMANDS */
    private final SetShooterVelocity startShooterSpeakerCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Speaker);
    private final SetShooterVelocity startShooterAmpCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Amp);
    private final SetShooterVelocity startShooterSourceIntakeCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.IntakeFromSource);
    private final SetShooterVelocity startShooterFloorIntakeCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.IntakeFromFloor);
    private final SetShooterVelocity startShooterTrapCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Trap);
    private final SetShooterVelocity stopShooterCmd = new SetShooterVelocity(shooterSub, shooterSub.shooterVelState.Stopped);
    private final ToggleIntake toggleIntakeCmd = new ToggleIntake(floorIntakeSub);

    private final SetShooterPosition setShooterSpeakerAngleCmd = new SetShooterPosition(shooterAngleSub, elevatorSub, shooterAngleSub.shooterAngleState.Speaker);
    private final SetShooterPosition setShooterAmpAngleCmd = new SetShooterPosition(shooterAngleSub, elevatorSub, shooterAngleSub.shooterAngleState.Amp);
    private final SetShooterPosition setShooterSourceAngleCmd = new SetShooterPosition(shooterAngleSub, elevatorSub, shooterAngleSub.shooterAngleState.IntakeFromSource);
    private final SetShooterPosition setShooterFloorAngleCmd = new SetShooterPosition(shooterAngleSub, elevatorSub, shooterAngleSub.shooterAngleState.IntakeFromFloor);
    private final SetShooterPosition setShooterTrapAngleCmd = new SetShooterPosition(shooterAngleSub, elevatorSub, shooterAngleSub.shooterAngleState.Trap);
    /* CONTROLLERS */
    Joystick driverController; 
    private final int DriverControllerPort = 1;
    
    Joystick operatorController;
    private final int OperatorControllerPort = 2;
    
    /* BUTTONS */
   
    public Trigger shootSpeakerButton;
    public Trigger shootAmpButton;
    public Trigger shooterSourceIntakeButton;
    public Trigger shooterFloorIntakeButton;
    public Trigger shootTrapButton;

    public Trigger toggleFloorIntakeButton;

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

        if(DriverStation.getJoystickName(DriverControllerPort).equals("Logitech Extreme 3D")) {
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
                        RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, 2, 
                        true));
        }

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }


    private void configureBindings() {

        driverController = new Joystick(DriverControllerPort);
        operatorController = new Joystick(OperatorControllerPort);
        
        shootSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        // shootSpeakerButton.onTrue(setShooterSpeakerAngleCmd);
        shootSpeakerButton.onTrue(startShooterSpeakerCmd).onFalse(stopShooterCmd);

        shootAmpButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper); // top
        // shootAmpButton.onTrue(setShooterAmpAngleCmd);
        shootAmpButton.onTrue(startShooterAmpCmd).onFalse(stopShooterCmd);

        shooterSourceIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick); // right stick press
        // shooterSourceIntakeButton.onTrue(setShooterSourceAngleCmd);
        shooterSourceIntakeButton.onTrue(startShooterSourceIntakeCmd).onFalse(stopShooterCmd);

        shooterFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        // shooterFloorIntakeButton.onTrue(setShooterFloorAngleCmd);
        shooterFloorIntakeButton.onTrue(startShooterFloorIntakeCmd).onFalse(stopShooterCmd);
        
        shootTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home
        // shootTrapButton.onTrue(setShooterTrapAngleCmd);
        shootTrapButton.onTrue(startShooterTrapCmd).onFalse(stopShooterCmd);

        toggleFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); //floor 
        toggleFloorIntakeButton.onTrue(toggleIntakeCmd);     
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score Amp");
    }
}