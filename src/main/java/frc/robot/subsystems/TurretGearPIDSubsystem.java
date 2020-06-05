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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.FalconFXConstants;
import frc.robot.Constants.TurretGearPIDSubsystemConstants;

public class TurretGearPIDSubsystem extends ProfiledPIDSubsystem {

  private static double rotatorMaxSpeed = TurretGearPIDSubsystemConstants.TURRET_ROTATOR_MAX_SPEED;
  
  private final WPI_TalonFX turretRotatorMotor = new WPI_TalonFX(TurretGearPIDSubsystemConstants.TURRET_ROTATOR_MOTOR_CAN_ID);
  /**
   * Creates a new ArmRotatorPIDSubsystem. From previous tests, you can usually get away with a P controller
   */
  public TurretGearPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new ProfiledPIDController(TurretGearPIDSubsystemConstants.kP, 
        TurretGearPIDSubsystemConstants.kI, TurretGearPIDSubsystemConstants.kD, 
        new TrapezoidProfile.Constraints(28, 12)));

    configRotatorMotor();
    getController().enableContinuousInput(-180, 180); // It is an angle controller.
    getController().setTolerance(TurretGearPIDSubsystemConstants.TOLERANCE);
    setGoal(TurretGearPIDSubsystemConstants.TURRET_HOME_ANGLE);
    enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    turretRotatorMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngleDegrees();
  }

  //@Deprecated No use
  public void setTurretRotatorMotorSpeed(double speed) {  //TODO rewrite in terms of voltage? Going to need to find experiemetnally
    if (speed >= -rotatorMaxSpeed && speed <= rotatorMaxSpeed)
      turretRotatorMotor.set(speed);
    else if (speed < -rotatorMaxSpeed)
      turretRotatorMotor.set( -rotatorMaxSpeed);
    else if (speed > rotatorMaxSpeed )
      turretRotatorMotor.set( rotatorMaxSpeed);
  }

  public boolean atSetpoint(){
    return getController().atSetpoint();
  }

  public double getSetpoint(){
    return getController().getSetpoint().position;
  }

  public void stopTurretRotator(){
    turretRotatorMotor.set(0.0);
  }

  public void moveForward(){
    disable();
    turretRotatorMotor.setVoltage(1.1);
    setGoal(getAngleDegrees());
    enable();

  }

  public void moveBackward(){
    disable();
    turretRotatorMotor.setVoltage(-1.1);
    setGoal(getAngleDegrees());
    enable();
  }

  public double getTurretRotatorMotorSpeed(){
    return turretRotatorMotor.get();
  }

  public int getTurretEncoderCounts(){
    return turretRotatorMotor.getSelectedSensorPosition();
  }

  public void configRotatorMotor(){
    turretRotatorMotor.configFactoryDefault();

    enableTurretRotatorSoftLimits();

    //turretRotatorMotor.configClearPositionOnLimitF(true, 0);

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
    turretRotatorEncoderConfigs.reverseSoftLimitThreshold = -65000;
    turretRotatorEncoderConfigs.reverseSoftLimitEnable = true;
     turretRotatorEncoderConfigs.forwardSoftLimitThreshold = 65000;
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
    rotatorMaxSpeed = TurretGearPIDSubsystemConstants.TURRET_ROTATOR_MAX_SPEED;

  }
/*
  public boolean turretRotatorFwdLimitSwitchHit(){ //Limit swtiches not needed
    return turretRotatorMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
  }

  public boolean turretRotatorReverseLimitSwitchHit(){ //Limit swtiches not needed
    return turretRotatorMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }
*/
  public double getAngleRadians(){
    return (double)(turretRotatorMotor.getSelectedSensorPosition()
    / (FalconFXConstants.ENCODER_UNITS_PER_REV)
    * (TurretGearPIDSubsystemConstants.GEAR_RATIO*2*Math.PI));
  }

  public double getAngleDegrees(){
    return Math.toDegrees(getAngleRadians());
  }

}
