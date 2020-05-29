/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconFXConstants;
import frc.robot.Constants.TurretSubsystemConstants;

public class TurretRotatorSubsystem extends SubsystemBase {

  private static double rotatorMaxSpeed = TurretSubsystemConstants.TURRET_ROTATOR_MAX_SPEED;
  
  private final WPI_TalonFX turretRotatorMotor = new WPI_TalonFX(TurretSubsystemConstants.TURRET_ROTATOR_MOTOR_CAN_ID);
  
  /**
   * Creates a new TurretRotatorSubsystem.
   */
  public TurretRotatorSubsystem() {
    configRotatorMotor();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Rotator Speed", turretRotatorMotor.get());
    SmartDashboard.putNumber("Turret Rotator Encoder Counts", turretRotatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Rotator Angle Degrees", getAngleDegrees());
    SmartDashboard.putBoolean("Turret Forward Limit Hit", turretRotatorFwdLimitSwitchHit());
    SmartDashboard.putBoolean("Turret Reverse Limit Hit", turretRotatorReverseLimitSwitchHit());
  }

  public void setTurretRotatorMotorSpeed(double speed) {  //ball
    if (speed >= -rotatorMaxSpeed && speed <= rotatorMaxSpeed)
      turretRotatorMotor.set(speed);
    else if (speed < -rotatorMaxSpeed)
      turretRotatorMotor.set( -rotatorMaxSpeed);
    else if (speed > rotatorMaxSpeed )
      turretRotatorMotor.set( rotatorMaxSpeed);
  }

  public void stopTurretRotator(){
    turretRotatorMotor.set(0.0);
  }

  public void configRotatorMotor(){
    turretRotatorMotor.configFactoryDefault();

    enableTurretRotatorSoftLimits();

    turretRotatorMotor.configClearPositionOnLimitF(true, 0);

    //turretRotatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    /*
    Figure this out in OC Not working out RN. ALso should set the orientation clockwise and counterclowise
    turretRotatorMotor.configPeakOutputForward(-0.20, 500);
    turretRotatorMotor.configPeakOutputReverse(0.20, 500); //the 500 is an arbituary guess
    */
  }

  private void enableTurretRotatorSoftLimits(){

    TalonFXConfiguration turretRotatorEncoderConfigs = new TalonFXConfiguration();
    turretRotatorEncoderConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //encoderConfigs.forwardSoftLimitThreshold = 4;
    turretRotatorEncoderConfigs.reverseSoftLimitThreshold = -86000;
    turretRotatorEncoderConfigs.reverseSoftLimitEnable = true;
    turretRotatorEncoderConfigs.forwardSoftLimitThreshold = 19000;
    turretRotatorEncoderConfigs.forwardSoftLimitEnable = true;
    turretRotatorMotor.configAllSettings(turretRotatorEncoderConfigs);    
  } 

  private void disableTurretRotatorSoftLimits(){

    TalonFXConfiguration turretRotatorEncoderConfigs = new TalonFXConfiguration();
    turretRotatorEncoderConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //encoderConfigs.forwardSoftLimitThreshold = 4;
    turretRotatorEncoderConfigs.reverseSoftLimitEnable = false;
    turretRotatorEncoderConfigs.forwardSoftLimitEnable = false;
    turretRotatorMotor.configAllSettings(turretRotatorEncoderConfigs);
  }

  public void initRotatorHomingMode() {
    disableTurretRotatorSoftLimits();
    rotatorMaxSpeed = 0.10;
  }

  public void closeRotatorHomeingMode() {
    enableTurretRotatorSoftLimits();
    rotatorMaxSpeed = TurretSubsystemConstants.TURRET_ROTATOR_MAX_SPEED;

  }

  public boolean turretRotatorFwdLimitSwitchHit(){
    return turretRotatorMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
  }

  public boolean turretRotatorReverseLimitSwitchHit(){
    return turretRotatorMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }

  public double getAngleRadians(){
    return (double)(turretRotatorMotor.getSelectedSensorPosition()
    / (FalconFXConstants.ENCODER_UNITS_PER_REV)
    * (TurretSubsystemConstants.GEAR_RATIO*2*Math.PI));
  }

  public double getAngleDegrees(){
    return Math.toDegrees(getAngleRadians());
  }

}
