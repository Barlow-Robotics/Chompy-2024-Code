// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    private final TalonFX lowerShooterMotor;
    private final TalonFX upperShooterMotor;
    // private final CANSparkMax indexMotor;

    private final TalonFXSimState lowerShooterMotorSim;
    private final DCMotorSim lowerMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, ShooterConstants.jKgMetersSquared);

    private final TalonFXSimState upperShooterMotorSim;
    private final DCMotorSim upperMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, ShooterConstants.jKgMetersSquared);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final NeutralOut brake = new NeutralOut();

    public enum ShooterVelState {
        Stopped, Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterVelState shooterVelState = ShooterVelState.Stopped;

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    boolean simulationInitialized = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry shuffleBoardSpeed = tab.add("ShuffleBoard Speed", 1).getEntry();
    private GenericEntry shuffleBoardSpeedBool = tab.add("Use ShuffleBoard Speed", false).getEntry();

    public Shooter() {
        lowerShooterMotor = new TalonFX(ElectronicIDs.LowerShooterMotorID); // slot 0
        upperShooterMotor = new TalonFX(ElectronicIDs.UpperShooterMotorID); // slot 0

        lowerShooterMotorSim = lowerShooterMotor.getSimState();
        upperShooterMotorSim = upperShooterMotor.getSimState();

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = ShooterConstants.ShooterKP;
        // configs.Slot0.kI = ShooterConstants.ShooterKI;
        // configs.Slot0.kD = ShooterConstants.ShooterKD;
        configs.Slot0.kV = ShooterConstants.ShooterKV;
        configs.Voltage.PeakForwardVoltage = ShooterConstants.PeakShooterForwardVoltage; // Peak output of 8 volts
        configs.Voltage.PeakReverseVoltage = ShooterConstants.PeakShooterReverseVoltage;

        MotorOutputConfigs upperMotorOutputConfigs = new MotorOutputConfigs();
        upperMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        applyLowerMotorConfigs(configs);
        applyUpperMotorConfigs(upperShooterMotor, configs, upperMotorOutputConfigs);
        // StatusCode statusLeft = StatusCode.StatusCodeNotInitialized;
        // StatusCode statusRight = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; ++i) {
        // statusLeft = lowerShooterMotor.getConfigurator().apply(configs);
        // statusRight = upperShooterMotor.getConfigurator().apply(configs);
        // statusRight =
        // upperShooterMotor.getConfigurator().apply(upperMotorOutputConfigs);
        // if (statusLeft.isOK() && statusRight.isOK())
        // break;
        // }
        // if (!statusLeft.isOK()) {
        // System.out.println("Could not apply configs to left, error code: " +
        // statusLeft.toString());
        // } else if (!statusRight.isOK()) {
        // System.out.println("Could not apply configs to right, error code: " +
        // statusRight.toString());
        // }
        breakBeam = new DigitalInput(ElectronicIDs.BreakBeamID);
    }

    @Override
    public void periodic() {
        advantageKitLogging();
    }

    public void setVelocity(double rotsPerSecond) {
        if (shuffleBoardSpeedBool.getBoolean(false)) {
            if (shuffleBoardSpeed.getDouble(-1.0) <= Constants.Falcon500MaxRPM / 60) { // max rps: 105
                rotsPerSecond = shuffleBoardSpeed.getDouble(-1.0);
            }
        }

        lowerShooterMotor.setControl(voltageVelocity.withVelocity(rotsPerSecond));
        upperShooterMotor.setControl(voltageVelocity.withVelocity(rotsPerSecond));

        NetworkTableInstance.getDefault().getEntry("shooter/Desired Rotations Per Second").setDouble(rotsPerSecond);
    }

    public void stopShooting() {
        lowerShooterMotor.setControl(brake);
        upperShooterMotor.setControl(brake);
        shooterVelState = ShooterVelState.Stopped;
    }

    private double getRPS(TalonFX motor) {
        return motor.getVelocity().getValue();
    }

    private double getClosedLoopError(TalonFX motor) {
        return motor.getClosedLoopError().getValue();
    }

    public boolean isWithinTolerance(double desiredSpeed) {
        return (getRPS(lowerShooterMotor) >= Constants.LowerToleranceLimit * desiredSpeed) &&
                (getRPS(lowerShooterMotor) <= Constants.UpperToleranceLimit * desiredSpeed);
    }

    public boolean isNoteLoaded() {
        return breakBeam.get();
    }

    public void setShooterVelState(ShooterVelState newState) {
        shooterVelState = newState;
    }

    public ShooterVelState getShooterVelState() {
        return shooterVelState;
    }

    public String getShooterVelStateAsString() {
        return shooterVelState.toString();
    }

    /* LOGGING */

    private void advantageKitLogging() {
        Logger.recordOutput("Shooter/State", getShooterVelStateAsString());
        Logger.recordOutput("Shooter/ActualRPMLower", getRPS(lowerShooterMotor));
        Logger.recordOutput("Shooter/ActualRPMUpper", getRPS(upperShooterMotor));
        Logger.recordOutput("Shooter/ClosedLoopErrorLower", getClosedLoopError(lowerShooterMotor));
        Logger.recordOutput("Shooter/ClosedLoopErrorUpper", getClosedLoopError(upperShooterMotor));
        Logger.recordOutput("Shooter/ShootingAmp", isWithinTolerance(ShooterConstants.AmpVelocity));
        Logger.recordOutput("Shooter/ShootingSpeaker", isWithinTolerance(ShooterConstants.SpeakerVelocity));
        Logger.recordOutput("Shooter/ShootingTrap", isWithinTolerance(ShooterConstants.TrapVelocity));
        Logger.recordOutput("Shooter/IntakingSource", isWithinTolerance(ShooterConstants.SourceIntakeVelocity));
        Logger.recordOutput("Shooter/IntakingFloor", isWithinTolerance(ShooterConstants.FloorIntakeVelocity));
        Logger.recordOutput("Shooter/NoteLoaded", isNoteLoaded());
    }

    /* CONFIG */

    private void applyLowerMotorConfigs(TalonFXConfiguration configs) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // attempt setting configs up to 5 times
        for (int i = 0; i < 5; ++i) {
            status = lowerShooterMotor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        // if it didn't work after 5th time, print out an error
        if (!status.isOK()) {
            System.out.println("Could not apply configs to left, error code: " + status.toString());
        }
    }

    private void applyUpperMotorConfigs(TalonFX motor, TalonFXConfiguration talonConfigs,
            MotorOutputConfigs motorOutputConfigs) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // attempt setting configs up to 5 times
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(talonConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs to right, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs to right, error code: " + status.toString());
        }

    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(lowerShooterMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(upperShooterMotor, 0.001);

        lowerShooterMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        upperShooterMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        lowerShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double lowerVoltage = lowerShooterMotorSim.getMotorVoltage();
        lowerMotorModel.setInputVoltage(lowerVoltage);
        lowerMotorModel.update(0.02);
        lowerShooterMotorSim.setRotorVelocity(lowerMotorModel.getAngularVelocityRPM() / 60.0);
        lowerShooterMotorSim.setRawRotorPosition(lowerMotorModel.getAngularPositionRotations());

        upperShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double upperVoltage = upperShooterMotorSim.getMotorVoltage();
        upperMotorModel.setInputVoltage(upperVoltage);
        upperMotorModel.update(0.02);
        upperShooterMotorSim.setRotorVelocity(upperMotorModel.getAngularVelocityRPM() / 60.0);
        upperShooterMotorSim.setRawRotorPosition(upperMotorModel.getAngularPositionRotations());
    }

}
