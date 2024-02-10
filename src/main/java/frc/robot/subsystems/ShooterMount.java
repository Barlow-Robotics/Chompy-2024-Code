// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPositionConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

import frc.robot.Constants;

public class ShooterMount extends SubsystemBase {

    TalonFX angleMotor;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);

    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, 0.0005);

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, 0.0005);

    private final VelocityVoltage angleVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0,
            false, false, false);
    private final VelocityVoltage elevatorVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 1,
            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput bottomHallEffect;

    private final CANcoder absoluteAngleEncoder;   //needs an encoder 
    private final CANcoderSimState absoluteAngleEncoderSim;

    public enum ShooterPositionState {
        Speaker, Amp, SourceIntake, FloorIntake, Trap, MovingToPosition, Interrupted
    }

    private ShooterPositionState shooterPosState = ShooterPositionState.FloorIntake;

    private boolean simulationInitialized = false;

    public ShooterMount() {
        bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        angleMotor = new TalonFX(ElectronicsIDs.AngleMotorID);
        angleMotorSim = angleMotor.getSimState();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState();
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        absoluteAngleEncoder = new CANcoder(ElectronicsIDs.AngleEncoderID, "rio");
        absoluteAngleEncoderSim = absoluteAngleEncoder.getSimState();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        // set current limit for all 3 motors
        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        applyMotorConfigs(angleMotor, "angleMotor", configs, motorOutputConfigs, InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(leftElevatorMotor, "leftElevatorMotor", configs, motorOutputConfigs,
                InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(rightElevatorMotor, "rightElevatorMotor", configs, motorOutputConfigs,
                InvertedValue.CounterClockwise_Positive); // CHANGE

        applyAngleEncoderConfigs(configs);
    }

    @Override
    public void periodic() {
        logData();
    }

    /** @param desiredAngle Desired angle in degrees */
    public void setAngle(double desiredAngle) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredAngle));
        angleMotor.setControl(request/* * .withFeedForward(ShooterPositionConstants.AngleFF) */);
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(absoluteAngleEncoder.getPosition().getValue());
    }

    public double getTalonEncoderDegrees() {
        return Units.rotationsToDegrees(angleMotor.getPosition().getValue());
    }

    public void setInches(double desiredHeight) {
        MotionMagicVoltage request = new MotionMagicVoltage(
                desiredHeight * ShooterPositionConstants.RotationsPerElevatorInch);
        leftElevatorMotor.setControl(request/* .withFeedForward(ShooterPositionConstants.ElevatorFF) */);
    }

    public double getHeight() {
        return leftElevatorMotor.getPosition().getValueAsDouble() / ShooterPositionConstants.RotationsPerElevatorInch;
    }

    public void setShooterPosState(ShooterPositionState newState) {
        shooterPosState = newState;
    }

    public ShooterPositionState getShooterPosState() {
        return shooterPosState;
    }

    public String getShooterPosStateAsString() {
        return shooterPosState.toString();
    }

    private boolean isWithinAngleTolerance(double desiredAngle) {
        return (getAngleDegrees() >= Constants.LowerToleranceLimit * desiredAngle) &&
                (getAngleDegrees() <= Constants.UpperToleranceLimit * desiredAngle);
    }

    private boolean isWithinHeightTolerance(double desiredHeight) {
        return (getHeight() >= Constants.LowerToleranceLimit * desiredHeight) &&
                (getHeight() <= Constants.UpperToleranceLimit * desiredHeight);
    }

    public boolean isWithinPositionTolerance(double desiredAngle, double desiredHeight) {
        return isWithinAngleTolerance(desiredAngle) && isWithinHeightTolerance(desiredHeight);
    }

    public boolean isAtBottom() {
        return !bottomHallEffect.get(); // might need to get rid of the ! depending on how the hall effect works
    }

    public void stop() {
        leftElevatorMotor.setControl(brake);
        rightElevatorMotor.setControl(brake);
    }

    private void logData() {
        Logger.recordOutput("ShooterMount/State", getShooterPosStateAsString());
        Logger.recordOutput("ShooterMount/ActualAngle", getAngleDegrees());
        Logger.recordOutput("ShooterMount/SimulationAngle", getTalonEncoderDegrees());
        Logger.recordOutput("ShooterMount/ActualHeight", getHeight());
        Logger.recordOutput("ShooterMount/Speaker/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.SpeakerAngle));
        Logger.recordOutput("ShooterMount/Speaker/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.SpeakerHeight));
        Logger.recordOutput("ShooterMount/Amp/IsAtAmpAngle",
                isWithinAngleTolerance(ShooterPositionConstants.AmpAngle));
        Logger.recordOutput("ShooterMount/Amp/IsAtAmpHeight",
                isWithinHeightTolerance(ShooterPositionConstants.AmpHeight));
        Logger.recordOutput("ShooterMount/Source/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.SourceIntakeAngle));
        Logger.recordOutput("ShooterMount/Source/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.SourceIntakeHeight));
        Logger.recordOutput("ShooterMount/Floor/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.FloorIntakeAngle));
        Logger.recordOutput("ShooterMount/Floor/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.FloorIntakeHeight));
        Logger.recordOutput("ShooterMount/Trap/IsAtTrapAngle",
                isWithinAngleTolerance(ShooterPositionConstants.TrapAngle));
        Logger.recordOutput("ShooterMount/Trap/IsAtTrapHeight",
                isWithinHeightTolerance(ShooterPositionConstants.TrapHeight));
        Logger.recordOutput("ShooterMount/IsAtBottom", isAtBottom());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorLeft", leftElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorRight",
                rightElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/Angle", angleMotor.getSupplyCurrent().getValue());
        // log number of rotations and the angle being reported back by cancoder
        // Encoder offset where the thing is 0
    }

    /* CONFIG */

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {

        if (motorName.equals("angleMotor")) {
            configs.Slot0.kP = ShooterPositionConstants.AngleKP;
            configs.Slot0.kI = ShooterPositionConstants.AngleKI;
            configs.Slot0.kD = ShooterPositionConstants.AngleKD;
            configs.Slot0.kV = ShooterPositionConstants.AngleFF;

            var motionMagicConfigs = configs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ShooterPositionConstants.AngleMMCruiseVel;
            motionMagicConfigs.MotionMagicAcceleration = ShooterPositionConstants.AngleMMAcceleration;
            motionMagicConfigs.MotionMagicJerk = ShooterPositionConstants.AngleMMJerk;
        } else if (motorName.indexOf("ElevatorMotor") != -1) {
            configs.Slot0.kP = ShooterPositionConstants.ElevatorKP;
            configs.Slot0.kI = ShooterPositionConstants.ElevatorKI;
            configs.Slot0.kD = ShooterPositionConstants.ElevatorKD;
            configs.Slot0.kV = ShooterPositionConstants.ElevatorFF;

            var motionMagicConfigs = configs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ShooterPositionConstants.ElevatorMMCruiseVel;
            motionMagicConfigs.MotionMagicAcceleration = ShooterPositionConstants.ElevatorMMAcceleration;
            motionMagicConfigs.MotionMagicJerk = ShooterPositionConstants.ElevatorMMJerk;
        }

        // configs.Voltage.PeakForwardVoltage =
        // ShooterConstants.PeakShooterForwardVoltage; // Peak output of 8 volts

        motorOutputConfigs.Inverted = inversion;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // attempt setting configs up to 5 times
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motorName + " error code: " + status.toString());
        }
    }

    private void applyAngleEncoderConfigs(TalonFXConfiguration configs) {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        var canCoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = 0.0;
        } else {
            magnetConfig.MagnetOffset = 0.0; // CHANGE
        }
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfiguration.MagnetSensor = magnetConfig;

        configs.Feedback.FeedbackRemoteSensorID = absoluteAngleEncoder.getDeviceID();
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        absoluteAngleEncoder.getConfigurator().apply(canCoderConfiguration);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(angleMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001);
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
    }

}