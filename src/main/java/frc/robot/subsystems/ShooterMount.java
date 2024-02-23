// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

import frc.robot.Constants;

public class ShooterMount extends SubsystemBase {

    TalonFX angleMotor;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, Constants.jKgMetersSquared);

    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, Constants.jKgMetersSquared);

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, Constants.jKgMetersSquared);

    DigitalInput bottomHallEffect;
    DIOSim bottomHallEffectSim;

    private final CANcoder angleCANCoder; // needs an encoder
    private final CANcoderSimState absoluteAngleEncoderSim; // CHANGE needed? never used

    public enum ShooterMountState {
        Speaker, Amp, SourceIntake, FloorIntake, Climb, MovingToPosition
    }

    private ShooterMountState shooterPosState = ShooterMountState.FloorIntake;

    private boolean simulationInitialized = false;

    public ShooterMount() {
        bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        angleMotor = new TalonFX(ElectronicsIDs.AngleMotorID);
        angleMotorSim = angleMotor.getSimState();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();
        leftElevatorMotor.setPosition(0);

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState();

        angleCANCoder = new CANcoder(ElectronicsIDs.AngleEncoderID, "rio");
        absoluteAngleEncoderSim = angleCANCoder.getSimState();

        applyAngleMotorConfigs(InvertedValue.Clockwise_Positive);
        applyAngleEncoderConfigs();
        applyElevatorMotorConfigs(leftElevatorMotor, "leftElevatorMotor", InvertedValue.CounterClockwise_Positive);
        // applyElevatorMotorConfigs(rightElevatorMotor, "rightElevatorMotor", InvertedValue.Clockwise_Positive);
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);

        stopElevator();
        stopAngle();
    }

    @Override
    public void periodic() {
        logData();

        if ((isAtBottom() && leftElevatorMotor.getVelocity().getValue() < 0) ||
                (getHeightInches() == ShooterMountConstants.MaxHeightInches && leftElevatorMotor.getVelocity().getValue() > 0)) {
            stopElevator();
        }

        if ((getAngleCANCoderDegrees() == ShooterMountConstants.MinAngleDegrees && angleMotor.getVelocity().getValue() < 0) ||
                (getHeightInches() == ShooterMountConstants.MaxAngleDegrees && angleMotor.getVelocity().getValue() > 0)) {
            stopAngle();
        }
    }

    /* ANGLE */

    public void setAngle(double desiredDegrees) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        angleMotor.setControl(request/* * .withFeedForward(ShooterPositionConstants.AngleFF) */);
    }

    public void setAngleWithVision(double desiredDegrees) {
        // Need to make this
    }

    public double getAngleCANCoderDegrees() {
        return Units.rotationsToDegrees(angleCANCoder.getAbsolutePosition().getValue());
    }

    private double getTalonEncoderDegrees() {
        return Units.rotationsToDegrees(angleMotor.getPosition().getValue());
    }

    /* ELEVATOR */

    public void setHeightInches(double desiredHeight) {
        MotionMagicVoltage request = new MotionMagicVoltage(
                desiredHeight * ShooterMountConstants.RotationsPerElevatorInch);
        leftElevatorMotor.setControl(request/* .withFeedForward(ShooterPositionConstants.ElevatorFF) */);
    }

    public double getHeightInches() {
        return leftElevatorMotor.getPosition().getValueAsDouble() / ShooterMountConstants.RotationsPerElevatorInch;
    }

    public void stopElevator() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    public void stopAngle() {
        angleMotor.set(0);
    }

    public void setBasePosition(double height) {
        leftElevatorMotor.setPosition(height);
    }

    public boolean isAtBottom() {
        return !bottomHallEffect.get();
    }

    /* SHOOTER MOUNT STATES */

    public void setShooterPosState(ShooterMountState newState) {
        shooterPosState = newState;
    }

    public ShooterMountState getShooterMountState() {
        return shooterPosState;
    }

    public String getShooterMountStateAsString() {
        return shooterPosState.toString();
    }

    /* TOLERANCES */

    private boolean isWithinAngleTolerance(double desiredAngle) {
        return (getAngleCANCoderDegrees() >= desiredAngle - ShooterMountConstants.AngleTolerance) &&
                (getAngleCANCoderDegrees() <= desiredAngle + ShooterMountConstants.AngleTolerance);
    }

    private boolean isWithinHeightTolerance(double desiredHeight) {
        return (getHeightInches() >= desiredHeight - ShooterMountConstants.HeightTolerance) &&
                (getHeightInches() <= desiredHeight + ShooterMountConstants.HeightTolerance);
    }

    public boolean isWithinPositionTolerance(double desiredAngle, double desiredHeight) {
        return isWithinAngleTolerance(desiredAngle) && isWithinHeightTolerance(desiredHeight);
    }

    private void logData() {
        Logger.recordOutput("ShooterMount/ShooterMountState", getShooterMountStateAsString());
        Logger.recordOutput("ShooterMount/CANCoderAngleDegrees", getAngleCANCoderDegrees());
        Logger.recordOutput("ShooterMount/CANCoderAngleRotations", angleCANCoder.getAbsolutePosition().getValue());
        Logger.recordOutput("ShooterMount/TalonAngle", getTalonEncoderDegrees());
        Logger.recordOutput("ShooterMount/ActualVoltageAngleMotor", angleMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/ActualVoltageElevatorLeftMotor", leftElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/ActualVoltageElevatorRightMotor", rightElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/ActualHeight", getHeightInches());
        Logger.recordOutput("ShooterMount/IsAtBottom", isAtBottom());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorLeft", leftElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorRight",
                rightElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/Angle", angleMotor.getSupplyCurrent().getValue());
        // log number of rotations and the angle being reported back by cancoder
        // Encoder offset where the thing is 0

        // LT & EH CHANGE - copied logging for new climb functions / consts -
        // ClimbHeight & MinHeight
        Logger.recordOutput("ShooterMount/Climb/IsAtClimbHeight",
                isWithinHeightTolerance(ShooterMountConstants.MaxHeightInches));
        Logger.recordOutput("ShooterMount/Climb/IsAtMinHeight",
                isWithinHeightTolerance(ShooterMountConstants.MinHeight));
    }

    /* CONFIG */

    private void applyAngleMotorConfigs(InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ShooterMountConstants.AngleKP;
        talonConfigs.Slot0.kI = ShooterMountConstants.AngleKI;
        talonConfigs.Slot0.kD = ShooterMountConstants.AngleKD;
        talonConfigs.Slot0.kV = ShooterMountConstants.AngleFF;
        talonConfigs.Slot0.kG = ShooterMountConstants.AngleKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterMountConstants.AngleMMCruiseVel;
        motionMagicConfigs.MotionMagicAcceleration = ShooterMountConstants.AngleMMAcceleration;
        motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.AngleMMJerk;

        talonConfigs.Feedback.FeedbackRemoteSensorID = angleCANCoder.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        applyMotorConfigs(angleMotor, "angleMotor", talonConfigs, inversion);
    }

    private void applyElevatorMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ShooterMountConstants.ElevatorKP;
        talonConfigs.Slot0.kI = ShooterMountConstants.ElevatorKI;
        talonConfigs.Slot0.kD = ShooterMountConstants.ElevatorKD;
        talonConfigs.Slot0.kV = ShooterMountConstants.ElevatorFF;
        talonConfigs.Slot0.kG = ShooterMountConstants.ElevatorKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        
        double rotationsPerSecond = 
            ShooterMountConstants.ElevatorMMCruiseInchesPerSecond * ShooterMountConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;
        
        double rotationsPerSecondPerSecond = 
            ShooterMountConstants.ElevatorMMInchesPerSecondPerSecond * ShooterMountConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecond;
        
        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecond * 3;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        /* SET & APPLY INVERSION CONFIGS */

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT CONFIGS */

        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterMountConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motor + " error code: " + status.toString());
        }
    }

    private void applyAngleEncoderConfigs() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        var canCoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ShooterMountConstants.AngleCANCoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfiguration.MagnetSensor = magnetConfig;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = angleCANCoder.getConfigurator().apply(canCoderConfiguration, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply CANCoder configs to angle encoder, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = angleCANCoder.getConfigurator().apply(magnetConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply magnet configs to angle encoder, error code: " + status.toString());
        }
    }

    private void setNeutralMode(NeutralModeValue angleMotorMode, NeutralModeValue elevatorMotorMode) {
        angleMotor.setNeutralMode(angleMotorMode);
        leftElevatorMotor.setNeutralMode(elevatorMotorMode);
        rightElevatorMotor.setNeutralMode(elevatorMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(angleMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001);
        bottomHallEffectSim = new DIOSim(bottomHallEffect);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        angleMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double angleVoltage = angleMotorSim.getMotorVoltage();
        angleMotorModel.setInputVoltage(angleVoltage);
        angleMotorModel.update(0.02);
        angleMotorSim.setRotorVelocity(angleMotorModel.getAngularVelocityRPM() / 60.0);
        angleMotorSim.setRawRotorPosition(angleMotorModel.getAngularPositionRotations());

        leftElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftElevatorMotorSim.getMotorVoltage();
        leftElevatorMotorModel.setInputVoltage(leftVoltage);
        leftElevatorMotorModel.update(0.02);
        leftElevatorMotorSim.setRotorVelocity(leftElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        leftElevatorMotorSim.setRawRotorPosition(leftElevatorMotorModel.getAngularPositionRotations());

        rightElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightElevatorMotorSim.getMotorVoltage();
        rightElevatorMotorModel.setInputVoltage(rightVoltage);
        rightElevatorMotorModel.update(0.02);
        rightElevatorMotorSim.setRotorVelocity(rightElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        rightElevatorMotorSim.setRawRotorPosition(rightElevatorMotorModel.getAngularPositionRotations());

        bottomHallEffectSim.setValue(isWithinHeightTolerance(0));
    }
}