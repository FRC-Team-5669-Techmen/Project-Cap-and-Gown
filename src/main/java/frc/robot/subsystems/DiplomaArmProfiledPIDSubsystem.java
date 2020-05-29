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

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
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
        new ProfiledPIDController(DiplomaArmSubsystemConstants.kP, DiplomaArmSubsystemConstants.kI, DiplomaArmSubsystemConstants.kD,
                                  // The motion profile constraints
                                  new TrapezoidProfile.Constraints(DiplomaArmSubsystemConstants.MAX_VELOCITY,
                                   DiplomaArmSubsystemConstants.MAX_ACCELERATION)));
    configDiplomaArmMotor();
    getController().enableContinuousInput(-180, 180);
    //TODO find out offset setGoal(DiplomaArmSubsystemConstants.ARM_ANGLE_OFFSET_AT_REST);//move to starting angle
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    diplomaArmMotor.setVoltage(output+feedForwardVolts(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngleFromHorizontalDegrees(); //+Angle offset if needed
  }

  private double feedForwardVolts(TrapezoidProfile.State setpoint){ //not using help class yet. too complicated
    return DiplomaArmSubsystemConstants.ARM_WEIGHT_N * (DiplomaArmSubsystemConstants.COM_DISTANCE_m)
    /(FalconFXConstants.MOTOR_STALL_TORQUE_Nm) * (DiplomaArmSubsystemConstants.NUMBER_OF_MOTORS) 
    * (DiplomaArmSubsystemConstants.GEAR_RATIO)* Math.cos(Math.toRadians(setpoint.position));
  }

  private void configDiplomaArmMotor(){
    diplomaArmMotor.configFactoryDefault();
    TalonFXConfiguration encoderConfigs = new TalonFXConfiguration();
    encoderConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    diplomaArmMotor.configAllSettings(encoderConfigs);
  }

  public double getAngleFromHorizontalRadians(){ //just returns how much of a revolution was actually made. Should range from 0 to 0.5 (1 rev=2pi)
    return (double)(diplomaArmMotor.getSelectedSensorPosition()
    / (FalconFXConstants.ENCODER_UNITS_PER_REV)
    * (DiplomaArmSubsystemConstants.GEAR_RATIO*2*Math.PI));
  }

  public double getAngleFromHorizontalDegrees(){
    return Math.toDegrees(getAngleFromHorizontalRadians());
  }

  public void moveToAngle (double angleDegrees){
    setGoal(angleDegrees);
  }

  
  

  
}
