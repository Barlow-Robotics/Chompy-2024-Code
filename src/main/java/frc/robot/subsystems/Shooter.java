// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
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

    private final TalonFXSimState lowerShooterMotorSim;
    private final DCMotorSim lowerMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1,
            0.0005);

    private final TalonFXSimState upperShooterMotorSim;
    private final DCMotorSim upperMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(
            0, 0, true, 0, 0,
            false, false, false);
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
    private double desiredRPSSmartDashboard;

    public Shooter() {
        lowerShooterMotor = new TalonFX(ElectronicIDs.LowerShooterMotorID); // slot 0
        upperShooterMotor = new TalonFX(ElectronicIDs.UpperShooterMotorID); // slot 0

        lowerShooterMotorSim = lowerShooterMotor.getSimState();
        upperShooterMotorSim = upperShooterMotor.getSimState();

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.5; // An error of 1 rotation per second results in 2V output
        // configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        // configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        configs.Voltage.PeakForwardVoltage = 8; // Peak output of 8 volts
        configs.Voltage.PeakReverseVoltage = -8;

        MotorOutputConfigs upperMotorOutputConfigs = new MotorOutputConfigs();
        upperMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode statusLeft = StatusCode.StatusCodeNotInitialized;
        StatusCode statusRight = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusLeft = lowerShooterMotor.getConfigurator().apply(configs);
            statusRight = upperShooterMotor.getConfigurator().apply(configs);
            statusRight = upperShooterMotor.getConfigurator().apply(upperMotorOutputConfigs);
            if (statusLeft.isOK() && statusRight.isOK())
                break;
        }
        if (!statusLeft.isOK()) {
            System.out.println("Could not apply configs to left, error code: " + statusLeft.toString());
        } else if (!statusRight.isOK()) {
            System.out.println("Could not apply configs to right, error code: " + statusRight.toString());
        }

        breakBeam = new DigitalInput(ElectronicIDs.BreakBeamID);

        networkTableInit();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Desired RPS", desiredRPSSmartDashboard);
    }

    public void setVelocity(double rotsPerSecond) {
        if (shuffleBoardSpeedBool.getBoolean(false)) {
            if (shuffleBoardSpeed.getDouble(-1.0) <= Constants.Falcon500MaxRPM/60) { // max rps: 105
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

    public double getLowerShooterVelocity() {
        return lowerShooterMotor.getVelocity().getValue();
    }

    public double getUpperShooterVelocity() {
        return upperShooterMotor.getVelocity().getValue();
    }

    private double getLowerShooterClosedLoopError() {
        return lowerShooterMotor.getClosedLoopError().getValue();
    }

    private double getUpperShooterClosedLoopError() {
        return upperShooterMotor.getClosedLoopError().getValue();
    }

    public boolean isSpeakerShooting() {
        return getLowerShooterVelocity() >= .95 * ShooterConstants.SpeakerVelocity;
    }

    public boolean isAmpShooting() {
        return (getLowerShooterVelocity() >= .95 * ShooterConstants.AmpVelocity) &&
                (getLowerShooterVelocity() <= 1.05 * ShooterConstants.AmpVelocity);
    }

    public boolean isSourceIntaking() {
        return (getLowerShooterVelocity() >= .95 * ShooterConstants.SourceIntakeVelocity)
                && (getLowerShooterVelocity() <= 1.05 * ShooterConstants.SourceIntakeVelocity);
    }

    public boolean isShooterFloorIntaking() {
        return (getLowerShooterVelocity() >= .95 * ShooterConstants.ShooterFloorIntakeVelocity)
                && (getLowerShooterVelocity() <= 1.05 * ShooterConstants.ShooterFloorIntakeVelocity);
    }

    public boolean isTrapShooting() {
        return (getLowerShooterVelocity() >= .95 * ShooterConstants.TrapVelocity)
                && (getLowerShooterVelocity() <= 1.05 * ShooterConstants.TrapVelocity);
    }

    public boolean isNoteLoaded() {
        // return breakBeam.get();
        return false;
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

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addStringProperty("State", this::getShooterVelStateAsString, null);
        builder.addDoubleProperty("Actual Lower Shooter Velocity", this::getLowerShooterVelocity, null);
        builder.addDoubleProperty("Actual Upper Shooter Velocity", this::getUpperShooterVelocity, null);
        builder.addDoubleProperty("Lower Shooter Closed Loop Error", this::getLowerShooterClosedLoopError, null);
        builder.addDoubleProperty("Upper Shooter Closed Loop Error", this::getUpperShooterClosedLoopError, null);
        builder.addBooleanProperty("Speaker Shooting", this::isSpeakerShooting, null);
        builder.addBooleanProperty("Amp Shooting", this::isAmpShooting, null);
        builder.addBooleanProperty("Source Intaking", this::isSourceIntaking, null);
        builder.addBooleanProperty("Floor Intaking", this::isShooterFloorIntaking, null);
        builder.addBooleanProperty("Trap Shooting", this::isTrapShooting, null);
        // builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
    }

    public void networkTableInit() {
        NetworkTableInstance.getDefault().getEntry("shooter/Desired Rotations Per Second").setDouble(0);
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
