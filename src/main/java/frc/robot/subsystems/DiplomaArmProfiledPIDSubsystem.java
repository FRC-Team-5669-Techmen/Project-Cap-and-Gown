/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DiplomaArmSubsystemConstants;
import frc.robot.Constants.FalconFXConstants;

public class DiplomaArmProfiledPIDSubsystem extends ProfiledPIDSubsystem {

  private static final WPI_TalonFX diplomaArmMotor = new WPI_TalonFX(DiplomaArmSubsystemConstants.DIPLOMA_ARM_CAN_ID);

  /**
   * Creates a new DiplomaArmProfiledPIDSubsystem.
   */
  public DiplomaArmProfiledPIDSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(DiplomaArmSubsystemConstants.kP, DiplomaArmSubsystemConstants.kI,
            DiplomaArmSubsystemConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(DiplomaArmSubsystemConstants.MAX_VELOCITY,
                DiplomaArmSubsystemConstants.MAX_ACCELERATION)));
    configDiplomaArmMotor();
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(3);
    // TODO find out offset
    // setGoal(DiplomaArmSubsystemConstants.ARM_ANGLE_OFFSET_AT_REST);//move to
    // starting angle
    setGoal(DiplomaArmSubsystemConstants.MIN_ANGLE_Q1_DEGREES);
    enable();//not sure why I have to do this.
    
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    diplomaArmMotor.setVoltage(output + feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0))/*feedForwardVolts(setpoint)*/);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngleFromHorizontalDegrees(); // +Angle offset if needed
  }

  public boolean atGoal() {
    return getController().atGoal();
  }

  public double getSetpointPosition() {
    return getController().getSetpoint().position;
  }

  public int getArmEncoderCounts() {
    return diplomaArmMotor.getSelectedSensorPosition();
  }

  /*
   * Going to use empircal method for the sake of time. But do try to figure this
   * out when you have the chance. private double
   * feedForwardVolts(TrapezoidProfile.State setpoint){ //not using help class
   * yet. too complicated return DiplomaArmSubsystemConstants.ARM_WEIGHT_N *
   * (DiplomaArmSubsystemConstants.COM_DISTANCE_m)
   * /(FalconFXConstants.MOTOR_STALL_TORQUE_Nm) *
   * (DiplomaArmSubsystemConstants.NUMBER_OF_MOTORS)
   * (DiplomaArmSubsystemConstants.GEAR_RATIO)*
   * Math.cos(Math.toRadians(setpoint.position)); }
   */

  public double feedForwardVolts(TrapezoidProfile.State setpoint) { // not using help class yet. too complicated
    return DiplomaArmSubsystemConstants.GRAVITY_FEED_FORWARD_VOLTAGE * Math.cos(Math.toRadians(setpoint.position));
  }

  private void configDiplomaArmMotor() {
    diplomaArmMotor.configFactoryDefault();
    TalonFXConfiguration encoderConfigs = new TalonFXConfiguration();
    encoderConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    diplomaArmMotor.configAllSettings(encoderConfigs);
    diplomaArmMotor.setNeutralMode(NeutralMode.Brake); //use to prevent falling to quickly in the event of disabling. Won't keep it level like feedforward, but helps.
    enableDiplomaArmSoftLimits();
  }

  private void enableDiplomaArmSoftLimits(){

    TalonFXConfiguration armEncoderConfigs = new TalonFXConfiguration();
    armEncoderConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //encoderConfigs.forwardSoftLimitThreshold = 4;
    armEncoderConfigs.reverseSoftLimitThreshold = 15000;
    armEncoderConfigs.reverseSoftLimitEnable = true;
    armEncoderConfigs.forwardSoftLimitThreshold = 63000;
    armEncoderConfigs.forwardSoftLimitEnable = true;
    diplomaArmMotor.configAllSettings(armEncoderConfigs);    
  } 

  public double getAngleFromHorizontalRadians() { // just returns how much of a revolution was actually made. Should
                                                  // range from 0 to 0.5 (1 rev=2pi)
    return (double) (diplomaArmMotor.getSelectedSensorPosition() / (FalconFXConstants.ENCODER_UNITS_PER_REV)
        * (DiplomaArmSubsystemConstants.GEAR_RATIO * 2 * Math.PI));
  }

  public double getAngleFromHorizontalDegrees() {
    return Math.toDegrees(getAngleFromHorizontalRadians());
  }

  public void moveToAngle(double angleDegrees) {
    setGoal(angleDegrees);
  }

  /**
   * Trigger with toggle. Voltage will not be set permanenty. Must repativiely call this method
   */
  public void moveForward() {
    if(getAngleFromHorizontalDegrees()<=DiplomaArmSubsystemConstants.MAX_ANGLE_Q2_DEGREES-1)
    {
      disable();//allow the user to move the arm by tuning off pid correction
      if (getAngleFromHorizontalDegrees()>74 && getAngleFromHorizontalDegrees()<90)
        diplomaArmMotor.setVoltage(1.7); //cosine too weak in this region
      else if (getAngleFromHorizontalDegrees()>DiplomaArmSubsystemConstants.MAX_ANGLE_Q1_DEGREES && getAngleFromHorizontalDegrees()<DiplomaArmSubsystemConstants.MIN_ANGLE_Q2_DEGREES)
        diplomaArmMotor.setVoltage(1.8*Math.abs(Math.cos(getAngleFromHorizontalRadians()))+feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));
      else if (getAngleFromHorizontalDegrees()>=DiplomaArmSubsystemConstants.MAX_ANGLE_Q2_DEGREES)
        diplomaArmMotor.setVoltage(0.60+feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));  //0.30
      else 
        diplomaArmMotor.setVoltage(1.5*feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));
      setGoal(getAngleFromHorizontalDegrees());
      enable();//turn the control back on
    }
    else 
      new PrintCommand("Angular Arm Limits Reached");
    
  }

  /**
   * Trigger with toggle. Voltage will not be set permanenty. Must repativiely call this method
   */
  public void moveBackward() {
    if(getAngleFromHorizontalDegrees()>=DiplomaArmSubsystemConstants.MIN_ANGLE_Q1_DEGREES+1)
    {
      disable();//allow the user to move the arm by tuning off pid correction
      if (getAngleFromHorizontalDegrees()<DiplomaArmSubsystemConstants.MIN_ANGLE_Q2_DEGREES && getAngleFromHorizontalDegrees()>DiplomaArmSubsystemConstants.MAX_ANGLE_Q1_DEGREES)
        diplomaArmMotor.setVoltage(-1.8*Math.abs(Math.cos(getAngleFromHorizontalRadians()))+feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));
      else if (getAngleFromHorizontalDegrees()<=DiplomaArmSubsystemConstants.MAX_ANGLE_Q1_DEGREES)
        diplomaArmMotor.setVoltage(-0.60+feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));  //0.30
      else 
        diplomaArmMotor.setVoltage(1.5*feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));
      setGoal(getAngleFromHorizontalDegrees());
      enable();//turn the control back on
    }
    else 
     new PrintCommand("Angular Arm Limits Reached");
   
  
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
   // diplomaArmMotor.setVoltage(feedForwardVolts(new TrapezoidProfile.State(getAngleFromHorizontalDegrees(), 0)));

  }

  
  

  
}
