// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    /*********************************************************************/
    /***************************** CONSTANTS *****************************/

    private static final double WheelRadius = Units.inchesToMeters(2.0);
    private static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
    private static final double GearRatio = 6.75;
    private static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute / GearRatio;

    private static final double MaxRPM = 5820;
    public static final double MaxVelocityPerSecond = MaxRPM * VelocityConversionFactor;

    /* DRIVE ENCODER */
    private static final double DriveKP = 0.04; // REQUIRES TUNING 
    private static final double DriveKI = 0.0015;
    private static final double DriveKD = 0;
    private static final double DriveIZone = 0.15;
    private static final double DriveFF = 1.0 / MaxVelocityPerSecond;

    /* TURN ENCODER */
    private static final int CANCoderResolution = 4096;
    private static final double PositionConversionFactor = WheelCircumference / GearRatio;
    private static final double TurnKP = 1; // Need to change
    private static final double TurnKI = 0;
    private static final double TurnKD = 0;

    private static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per revolution (rad/sec)
    private static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared

    /**********************************************************************/
    /**********************************************************************/

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    public final SparkPIDController drivePIDController;

    private final CANSparkMax turnMotor;
    private final CANcoder turnEncoder;
    public final ProfiledPIDController turnPIDController;
    private final SimpleMotorFeedforward TurnFF = new SimpleMotorFeedforward(0, 0.4); // Need to change these #'s

    private String swerveName;

    public SwerveModule(
            String name,
            int driveMotorID,
            int turningMotorID,
            int turnEncoderID,
            double magnetOffset,
            boolean reversed) {

        /* Set up drive motor and encoder */
        swerveName = name;
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(reversed);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(VelocityConversionFactor);

        double localPositionConversionFactor = PositionConversionFactor;
        if (RobotBase.isSimulation()) {
            localPositionConversionFactor *= 1000;
        }
        driveEncoder.setPositionConversionFactor(localPositionConversionFactor);

        /* Config drive motor PID */
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(DriveKP);
        drivePIDController.setI(DriveKI);
        drivePIDController.setD(DriveKD);
        drivePIDController.setIZone(DriveIZone);
        drivePIDController.setFF(DriveFF);
        drivePIDController.setOutputRange(-1, 1);

        /* Set up turn motor and encoder */
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(true);

        turnEncoder = new CANcoder(turnEncoderID, "rio");
        var canCoderConfiguration = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = magnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfiguration.MagnetSensor = magnetConfig;

        // need to be added
        // canCoderConfiguration.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition; // BW sets sensor to be absolute zero
        // canCoderConfiguration.sensorCoefficient = Math.PI / 2048.0;

        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        turnPIDController = new ProfiledPIDController(
                1,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration));
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees

        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble()));

        // SwerveModuleState state = desiredState; // wpk temp

        Logger.recordOutput(swerveName + " Drive velocity", driveMotor.getEncoder().getVelocity());

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        final double turnOutput = turnPIDController.calculate(
                turnEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI,
                state.angle.getRadians());

        final double turnFF = TurnFF.calculate(turnPIDController.getSetpoint().velocity);
        turnMotor.setVoltage(turnOutput + turnFF);

        if (RobotBase.isSimulation()) {
            double angle = state.angle.getRadians();
            CANcoderSimState encoderSim = turnEncoder.getSimState();

            int rawPosition = 0;
            if (angle < 0) {
                rawPosition = 4096 + (int) ((angle / Math.PI) * 2048.0);
            } else {
                rawPosition = (int) ((angle / Math.PI) * 2048.0);
            }
            encoderSim.setRawPosition(state.angle.getDegrees() / 180.0);
            Logger.recordOutput("CANCoder " + swerveName,
                    turnEncoder.getAbsolutePosition().getValueAsDouble());
            Logger.recordOutput("CANCoder Raw " + swerveName, rawPosition);
            Logger.recordOutput("Module Desired State Angle" + swerveName,
                    desiredState.angle.getRadians());
            Logger.recordOutput("Module State Angle" + swerveName,
                    desiredState.angle.getRadians());
        }
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNEO(1));
    }
}
