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
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);

    private final VelocityVoltage angleVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0,
            false, false, false);
    private final VelocityVoltage elevatorVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 1,
            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput bottomHallEffect;

    private final CANcoder angleEncoder;
    private final CANcoderSimState angleEncoderSim;

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

        angleEncoder = new CANcoder(ElectronicsIDs.AngleEncoderID, "rio");
        angleEncoderSim = angleEncoder.getSimState();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        applyMotorConfigs(angleMotor, "angleMotor", configs, motorOutputConfigs, InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(leftElevatorMotor, "leftElevatorMotor", configs, motorOutputConfigs,
                InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(rightElevatorMotor, "rightElevatorMotor", configs, motorOutputConfigs,
                InvertedValue.CounterClockwise_Positive); // CHANGE

        applyMagnetConfigs(configs);
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
        return Units.rotationsToDegrees(angleEncoder.getPosition().getValue());
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

    private void logData() {
        Logger.recordOutput("ShooterPosition/State", getShooterPosStateAsString());
        Logger.recordOutput("ShooterPosition/ActualAngle", getAngleDegrees());
        Logger.recordOutput("ShooterPosition/ActualHeight", getHeight());
        Logger.recordOutput("ShooterPosition/Speaker/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.SpeakerAngle));
        Logger.recordOutput("ShooterPosition/Speaker/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.SpeakerHeight));
        Logger.recordOutput("ShooterPosition/Amp/IsAtAmpAngle",
                isWithinAngleTolerance(ShooterPositionConstants.AmpAngle));
        Logger.recordOutput("ShooterPosition/Amp/IsAtAmpHeight",
                isWithinHeightTolerance(ShooterPositionConstants.AmpHeight));
        Logger.recordOutput("ShooterPosition/Source/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.SourceIntakeAngle));
        Logger.recordOutput("ShooterPosition/Source/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.SourceIntakeHeight));
        Logger.recordOutput("ShooterPosition/Floor/IsAtAngle",
                isWithinAngleTolerance(ShooterPositionConstants.FloorIntakeAngle));
        Logger.recordOutput("ShooterPosition/Floor/IsAtHeight",
                isWithinHeightTolerance(ShooterPositionConstants.FloorIntakeHeight));
        Logger.recordOutput("ShooterPosition/Trap/IsAtTrapAngle",
                isWithinAngleTolerance(ShooterPositionConstants.TrapAngle));
        Logger.recordOutput("ShooterPosition/Trap/IsAtTrapHeight",
                isWithinHeightTolerance(ShooterPositionConstants.TrapHeight));
        Logger.recordOutput("ShooterPosition/IsAtBottom", isAtBottom());
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

        configs.Voltage.PeakForwardVoltage = ShooterConstants.PeakShooterForwardVoltage; // Peak output of 8 volts

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
    private void applyMagnetConfigs(TalonFXConfiguration configs) {
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

        configs.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        angleEncoder.getConfigurator().apply(canCoderConfiguration);
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