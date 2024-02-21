// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    private final TalonFX leftFlywheelMotor;
    private final TalonFXSimState lowerFlywheelMotorSim;
    private final DCMotorSim lowerMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, Constants.jKgMetersSquared);
    
    private final TalonFX rightFlywheelMotor;
    private final TalonFXSimState upperFlywheelMotorSim;
    private final DCMotorSim upperMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, Constants.jKgMetersSquared);

    private final TalonFX indexMotor;
    private final TalonFXSimState indexMotorSim;
    private final DCMotorSim indexMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, Constants.jKgMetersSquared);

    private final VelocityVoltage voltageVelocityFlywheel = 
        new VelocityVoltage(0, 0, true, 0, 0, 
                            false, false, false);
    private final VelocityVoltage voltageVelocityIndex = 
        new VelocityVoltage(0, 0, true, 0, 1, 
                            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    private boolean simulationInitialized = false;

    // private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    // private GenericEntry shuffleBoardSpeed = tab.add("ShuffleBoard Speed", 1).getEntry();

    public Shooter() {
        leftFlywheelMotor = new TalonFX(ElectronicsIDs.LeftShooterMotorID); 
        rightFlywheelMotor = new TalonFX(ElectronicsIDs.RightShooterMotorID); 
        indexMotor = new TalonFX(ElectronicsIDs.IndexMotorID); 
        
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        setPIDConfigs(PIDConfigs);
        applyMotorConfig( // check inversion
            leftFlywheelMotor, "lowerFlywheelMotor", PIDConfigs, motorOutputConfigs, InvertedValue.Clockwise_Positive);
        applyMotorConfig( // check inversion
            rightFlywheelMotor, "upperFlywheelMotor", PIDConfigs, motorOutputConfigs, InvertedValue.Clockwise_Positive);
        applyMotorConfig( // check inversion
            indexMotor, "indexMotor", PIDConfigs, motorOutputConfigs, InvertedValue.Clockwise_Positive);

        lowerFlywheelMotorSim = leftFlywheelMotor.getSimState();
        upperFlywheelMotorSim = rightFlywheelMotor.getSimState();
        indexMotorSim = indexMotor.getSimState();

        breakBeam = new DigitalInput(ElectronicsIDs.BreakBeamID);
    }

    @Override
    public void periodic() {
        logData();
    }

    public void startFlywheels(double shooterRPM) {
        Logger.recordOutput("Shooter/DesiredFlywheelRPM", shooterRPM);
        Logger.recordOutput("Shooter/DesiredFlywheelRPMWithGearRatio", shooterRPM * ShooterConstants.FlywheelGearRatio);

        double shooterRPS = shooterRPM / 60;

        leftFlywheelMotor.setControl(voltageVelocityFlywheel.withVelocity(shooterRPS));
        rightFlywheelMotor.setControl(voltageVelocityFlywheel.withVelocity(shooterRPS));

        Logger.recordOutput("Shooter/WithinToleranceFlywheels", isWithinFlywheelVelocityTolerance(shooterRPS));
    }

    public void startIndex(double indexRPM) {
        Logger.recordOutput("Shooter/DesiredIndexRPM", indexRPM);

        double indexRPS = indexRPM / 60;

        indexMotor.setControl(voltageVelocityIndex.withVelocity(indexRPS));
    }

    public void stop() {
        leftFlywheelMotor.setControl(brake);
        rightFlywheelMotor.setControl(brake);
        indexMotor.setControl(brake);
    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValue()*Constants.SecondsPerMinute;
    }

    public boolean isWithinFlywheelVelocityTolerance(double desiredRPM) {
        return (getRPM(leftFlywheelMotor) >= desiredRPM - ShooterConstants.VelocityTolerance) &&
                (getRPM(leftFlywheelMotor) <= desiredRPM + ShooterConstants.VelocityTolerance) &&
                (getRPM(rightFlywheelMotor) >= desiredRPM - ShooterConstants.VelocityTolerance) && 
                (getRPM(rightFlywheelMotor) <= desiredRPM + ShooterConstants.VelocityTolerance);   
    }

    public boolean isNoteLoaded() {
        return !breakBeam.get();
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("Shooter/ActualRPMLower", getRPM(leftFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMUpper", getRPM(rightFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMIndex", getRPM(indexMotor));
        Logger.recordOutput("Shooter/ClosedLoopErrorLower", leftFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/ClosedLoopErrorUpper", rightFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/IsNoteLoaded", isNoteLoaded());
        Logger.recordOutput("Shooter/IsShooting/Amp", isWithinFlywheelVelocityTolerance(ShooterConstants.AmpRPM));
        Logger.recordOutput("Shooter/IsShooting/Speaker", isWithinFlywheelVelocityTolerance(ShooterConstants.SpeakerRPM));
        Logger.recordOutput("Shooter/IsShooting/Trap", isWithinFlywheelVelocityTolerance(ShooterConstants.TrapRPM));
        Logger.recordOutput("Shooter/IsIntaking", isWithinFlywheelVelocityTolerance(ShooterConstants.IntakeRPM));
        Logger.recordOutput("Shooter/SupplyCurrent/FlywheelLower", leftFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/SupplyCurrent/FlywheelUpper", rightFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/SupplyCurrent/Index", indexMotor.getSupplyCurrent().getValue());
    }

    /* CONFIG */

    private void setPIDConfigs(TalonFXConfiguration talonConfigs) {
        talonConfigs.Slot0.kP = ShooterConstants.FlywheelKP;
        talonConfigs.Slot0.kI = ShooterConstants.FlywheelKI;
        talonConfigs.Slot0.kD = ShooterConstants.FlywheelKD;
        talonConfigs.Slot0.kV = ShooterConstants.FlywheelFF;

        talonConfigs.Slot1.kP = ShooterConstants.IndexKP;
        talonConfigs.Slot1.kI = ShooterConstants.IndexKI;
        talonConfigs.Slot1.kD = ShooterConstants.IndexKD;
        talonConfigs.Slot1.kV = ShooterConstants.IndexFF;
    }

    private void applyMotorConfig(TalonFX motor, String motorName, 
        TalonFXConfiguration talonConfigs, MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(talonConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply PID configs to " + motorName + ", error code: " + status.toString());
        }

        /* SET & APPLY INVERSION */

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply inversion configs to " + motorName + ", error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT */

        CurrentLimitsConfigs currentLimitConfigs = talonConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply current limit configs to " + motorName + ", error code: " + status.toString());
        }
    }
    
    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(leftFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);

        lowerFlywheelMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        upperFlywheelMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        lowerFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double lowerVoltage = lowerFlywheelMotorSim.getMotorVoltage();
        lowerMotorModel.setInputVoltage(lowerVoltage);
        lowerMotorModel.update(0.02);
        lowerFlywheelMotorSim.setRotorVelocity(lowerMotorModel.getAngularVelocityRPM() / 60.0);
        lowerFlywheelMotorSim.setRawRotorPosition(lowerMotorModel.getAngularPositionRotations());

        upperFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double upperVoltage = upperFlywheelMotorSim.getMotorVoltage();
        upperMotorModel.setInputVoltage(upperVoltage);
        upperMotorModel.update(0.02);
        upperFlywheelMotorSim.setRotorVelocity(upperMotorModel.getAngularVelocityRPM() / 60.0);
        upperFlywheelMotorSim.setRawRotorPosition(upperMotorModel.getAngularPositionRotations());

        indexMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double indexVoltage = indexMotorSim.getMotorVoltage();
        indexMotorModel.setInputVoltage(indexVoltage);
        indexMotorModel.update(0.02);
        indexMotorSim.setRotorVelocity(indexMotorModel.getAngularVelocityRPM() / 60.0);
        indexMotorSim.setRawRotorPosition(indexMotorModel.getAngularPositionRotations());
    }
}