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
import com.ctre.phoenix6.controls.NeutralOut;
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
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
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

    private final NeutralOut brake = new NeutralOut();

    DigitalInput bottomHallEffect;
    DIOSim bottomHallEffectSim;

    private final CANcoder absoluteAngleEncoder;   //needs an encoder 
    private final CANcoderSimState absoluteAngleEncoderSim;  //CHANGE needed?  never used

    public enum ShooterMountState {
        Speaker, Amp, SourceIntake, FloorIntake, PreClimb, Climb, PreTrap, Trap, MovingToPosition
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
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        absoluteAngleEncoder = new CANcoder(ElectronicsIDs.AngleEncoderID, "rio");
        absoluteAngleEncoderSim = absoluteAngleEncoder.getSimState();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        // set current limit for all 3 motors
        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        applyMotorConfigs(angleMotor, configs, motorOutputConfigs, InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(leftElevatorMotor, configs, motorOutputConfigs,
                InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(rightElevatorMotor, configs, motorOutputConfigs,
                InvertedValue.CounterClockwise_Positive); // CHANGE

        applyAngleEncoderConfigs(configs);
        stop();
    }

    @Override
    public void periodic() {
        logData();

        if (isAtBottom() && leftElevatorMotor.getVelocity().getValue() < 0) {
            stop();
        } else if (getHeightInches() == ShooterMountConstants.MaxHeightInches && leftElevatorMotor.getVelocity().getValue() > 0) {
            stop();
        }
    }

    /** @param desiredAngle Desired angle in degrees */
    public void setAngle(double desiredAngle) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredAngle));
        angleMotor.setControl(request/* * .withFeedForward(ShooterPositionConstants.AngleFF) */);
    }

    public void setAngleWithVision(double desiredAngle) {
        // Need to make this
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(angleMotor.getPosition().getValue());
    }

    public double getTalonEncoderDegrees() {
        return Units.rotationsToDegrees(absoluteAngleEncoder.getPosition().getValue());
    }

    public void setHeightInches(double desiredHeight) {
        MotionMagicVoltage request = new MotionMagicVoltage(
                desiredHeight * ShooterMountConstants.RotationsPerElevatorInch);
        leftElevatorMotor.setControl(request/* .withFeedForward(ShooterPositionConstants.ElevatorFF) */);
    }

    public double getHeightInches() {
        return leftElevatorMotor.getPosition().getValueAsDouble() / ShooterMountConstants.RotationsPerElevatorInch;
    }

    public void setBasePosition(double height) {
        leftElevatorMotor.setPosition(height);
    }

    public void setShooterPosState(ShooterMountState newState) {
        shooterPosState = newState;
    }

    public ShooterMountState getShooterPosState() {
        return shooterPosState;
    }

    public String getShooterPosStateAsString() {
        return shooterPosState.toString();
    }

    private boolean isWithinAngleTolerance(double desiredAngle) {
        return (getAngleDegrees() >= desiredAngle - ShooterMountConstants.AngleTolerance) &&
                (getAngleDegrees() <= desiredAngle + ShooterMountConstants.AngleTolerance);
    }

    private boolean isWithinHeightTolerance(double desiredHeight) {
        return (getHeightInches() >= desiredHeight - ShooterMountConstants.HeightTolerance) &&
                (getHeightInches() <= desiredHeight + ShooterMountConstants.HeightTolerance);
    }

    public boolean isWithinPositionTolerance(double desiredAngle, double desiredHeight) {
        return isWithinAngleTolerance(desiredAngle) && isWithinHeightTolerance(desiredHeight);
    }

    public boolean isAtBottom() {
        return bottomHallEffect.get(); // In sim, hall effect is always true when we use ! and always false when we don't. 
                                       // Since we can't trigger hall effect in sim, remove !
    }

    private void logData() {
        Logger.recordOutput("ShooterMount/State", getShooterPosStateAsString());
        Logger.recordOutput("ShooterMount/ActualAngle", getAngleDegrees());
        Logger.recordOutput("ShooterMount/SimulationAngle", getTalonEncoderDegrees());
        Logger.recordOutput("ShooterMount/ActualHeight", getHeightInches());
        Logger.recordOutput("ShooterMount/Speaker/IsAtAngle",
                isWithinAngleTolerance(ShooterMountConstants.SpeakerAngle));
        Logger.recordOutput("ShooterMount/Speaker/IsAtHeight",
                isWithinHeightTolerance(ShooterMountConstants.SpeakerHeight));
        Logger.recordOutput("ShooterMount/Amp/IsAtAngle",
                isWithinAngleTolerance(ShooterMountConstants.AmpAngle));
        Logger.recordOutput("ShooterMount/Amp/IsAtHeight",
                isWithinHeightTolerance(ShooterMountConstants.AmpHeight));
        Logger.recordOutput("ShooterMount/Source/IsAtAngle",
                isWithinAngleTolerance(ShooterMountConstants.SourceIntakeAngle));
        Logger.recordOutput("ShooterMount/Source/IsAtHeight",
                isWithinHeightTolerance(ShooterMountConstants.SourceIntakeHeight));
        Logger.recordOutput("ShooterMount/Floor/IsAtAngle",
                isWithinAngleTolerance(ShooterMountConstants.FloorIntakeAngle));
        Logger.recordOutput("ShooterMount/Floor/IsAtHeight",
                isWithinHeightTolerance(ShooterMountConstants.FloorIntakeHeight));
        Logger.recordOutput("ShooterMount/Trap/IsAtAngle",
                isWithinAngleTolerance(ShooterMountConstants.TrapAngle));
        Logger.recordOutput("ShooterMount/Trap/IsAtHeight",
                isWithinHeightTolerance(ShooterMountConstants.TrapHeight));
        Logger.recordOutput("ShooterMount/IsAtBottom", isAtBottom());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorLeft", leftElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorRight",
                rightElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/Angle", angleMotor.getSupplyCurrent().getValue());
        // log number of rotations and the angle being reported back by cancoder
        // Encoder offset where the thing is 0
    
        // LT & EH CHANGE - copied logging for new climb functions / consts - ClimbHeight & MinHeight
        Logger.recordOutput("ShooterMount/Climb/IsAtClimbHeight",
                isWithinHeightTolerance(ShooterMountConstants.ClimbHeight));
        Logger.recordOutput("ShooterMount/Climb/IsAtMinHeight",
                isWithinHeightTolerance(ShooterMountConstants.MinHeight));
    }

    /* CONFIG */

    private void applyMotorConfigs(TalonFX motor, TalonFXConfiguration configs,
            MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {

        if (motor == angleMotor) {
            configs.Slot0.kP = ShooterMountConstants.AngleKP;
            configs.Slot0.kI = ShooterMountConstants.AngleKI;
            configs.Slot0.kD = ShooterMountConstants.AngleKD;
            configs.Slot0.kV = ShooterMountConstants.AngleFF;

            var motionMagicConfigs = configs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ShooterMountConstants.AngleMMCruiseVel;
            motionMagicConfigs.MotionMagicAcceleration = ShooterMountConstants.AngleMMAcceleration;
            motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.AngleMMJerk;
        } else if (motor == leftElevatorMotor || motor == rightElevatorMotor) {
            configs.Slot0.kP = ShooterMountConstants.ElevatorKP;
            configs.Slot0.kI = ShooterMountConstants.ElevatorKI;
            configs.Slot0.kD = ShooterMountConstants.ElevatorKD;
            configs.Slot0.kV = ShooterMountConstants.ElevatorFF;

            var motionMagicConfigs = configs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ShooterMountConstants.ElevatorMMCruiseVel;
            motionMagicConfigs.MotionMagicAcceleration = ShooterMountConstants.ElevatorMMAcceleration;
            motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
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
            System.out.println("Could not apply talon configs to " + motor + " error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
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

    public void stop() { 
        angleMotor.set(0);
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }
}