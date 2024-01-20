// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;



public class Shooter extends SubsystemBase {
  /*********************************************************************/
  /***************************** CONSTANTS *****************************/

  private static final int Velocity = 0;
  

  /*******************************************************************************/
  /*******************************************************************************/
  
  TalonFX leftShooterMotor;
  TalonFX rightShooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new TalonFX(CanIDs.LeftShooterMotorID);
    rightShooterMotor = new TalonFX(CanIDs.RightShooterMotorID);
  }

  
    public void setShooterMotor(TalonFX shooterMotor) {
        leftShooterMotor = shooterMotor;
    }
    public void startShooter(){
      leftShooterMotor.set
    }

  
    public void stopShooter(){

    }
    public void getVelocity(){

    }
    public boolean isShooting(){
      
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
