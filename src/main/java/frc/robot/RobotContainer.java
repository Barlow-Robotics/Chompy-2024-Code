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
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SetShooterVelocity;
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
    private final ToggleIntake toggleIntakeCmd = new ToggleIntake(floorIntakeSub);
    
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

        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, 
                        driverController, 
                        // wpk - on the logitech controller, the rotation input is #2. 
                        // wpk - put this back for radio master, or we could figure out a way to automaticaly detect the control type and adjust)
                        // RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
                        RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, 2, 
                        true));

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }


    private void configureBindings() {

        driverController = new Joystick(DriverControllerPort);
        operatorController = new Joystick(OperatorControllerPort);
        
        shootSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        shootSpeakerButton.onTrue(startShooterSpeakerCmd);

        shootAmpButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper); // top
        shootAmpButton.onTrue(startShooterAmpCmd);

        shooterSourceIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick); // right stick press
        shooterSourceIntakeButton.onTrue(startShooterSourceIntakeCmd);

        shooterFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        shooterFloorIntakeButton.onTrue(startShooterFloorIntakeCmd);
        
        shootTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home
        shootTrapButton.onTrue(startShooterTrapCmd);

        toggleFloorIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX); //floor 
        toggleFloorIntakeButton.onTrue(toggleIntakeCmd);     
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Score Amp");
    }
}