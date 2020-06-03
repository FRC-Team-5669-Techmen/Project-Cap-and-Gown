/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DiplomaArmSubsystemConstants;
import frc.robot.Constants.MoveDiplomaArmConstants;
import frc.robot.Constants.RotateTurretGearConstants;
import frc.robot.Constants.TurretGearPIDSubsystemConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualMecanumDrive;
import frc.robot.commands.MoveDiplomaArm;
import frc.robot.commands.RotateTurret;
import frc.robot.subsystems.BlinkinLEDSubsystem;
import frc.robot.subsystems.DiplomaArmProfiledPIDSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.subsystems.TurretGearPIDSubsystem;
import frc.robot.subsystems.TurretRotatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MecanumDriveSubsystem mecanumDriveSubsystem = new MecanumDriveSubsystem();
  private final BlinkinLEDSubsystem blinkinLEDSubsystem = new BlinkinLEDSubsystem();
  private final DiplomaArmProfiledPIDSubsystem profiledPIDDiplomaArm = new DiplomaArmProfiledPIDSubsystem();
  private final TurretGearPIDSubsystem turretGearPIDSubsystem = new TurretGearPIDSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Joystick m_joystick = new Joystick(ControllerConstants.JOYSTICK_CONTROLLER_PORT);
  private final Joystick buttonBox = new Joystick(ControllerConstants.BUTTON_BOX_CONTROLLER_PORT);

  private InstantCommand displayStandby = new InstantCommand(blinkinLEDSubsystem::standby_mode, blinkinLEDSubsystem);

  private InstantCommand displayCSEEBlue = new InstantCommand(blinkinLEDSubsystem::csee_blue, blinkinLEDSubsystem);

  private InstantCommand displayMATYellow = new InstantCommand(blinkinLEDSubsystem::mat_yellow, blinkinLEDSubsystem);
  
  private InstantCommand displayMSETRed = new InstantCommand(blinkinLEDSubsystem::mset_red, blinkinLEDSubsystem);

  private InstantCommand displayIDEAGreen = new InstantCommand(blinkinLEDSubsystem::idea_green, blinkinLEDSubsystem);

  private InstantCommand displayACEOrange = new InstantCommand(blinkinLEDSubsystem::ace_orange, blinkinLEDSubsystem);

  private Command deliverDiplomaToStudent = new RotateTurret(RotateTurretGearConstants.STUDENT_POSITION_DEGREES, turretGearPIDSubsystem).
  alongWith(new MoveDiplomaArm(MoveDiplomaArmConstants.STUDENT_POSITION_DEGREES, profiledPIDDiplomaArm));//fix

  private Command returnArmToPresident = new RotateTurret(RotateTurretGearConstants.PRESDIENT_POSITION_DEGREES, turretGearPIDSubsystem).
  alongWith(new MoveDiplomaArm(MoveDiplomaArmConstants.PRESDIENT_POSITION_DEGREES, profiledPIDDiplomaArm));//fix



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    Shuffleboard.getTab("Colors").add("Standby", displayStandby);
    Shuffleboard.getTab("Colors").add("CSEE Blue", displayCSEEBlue);
    Shuffleboard.getTab("Colors").add("MAT Yellow", displayMATYellow);
    Shuffleboard.getTab("Colors").add("MSET Red", displayMSETRed);
    Shuffleboard.getTab("Colors").add("IDEA Green", displayIDEAGreen);
    Shuffleboard.getTab("Colors").add("ACE Orange", displayACEOrange);

    Shuffleboard.getTab("Gradbot").addNumber("Turret Roator Speed", turretGearPIDSubsystem::getTurretRotatorMotorSpeed);
    Shuffleboard.getTab("Gradbot").addNumber("Turret Rotator Encoder Counts", turretGearPIDSubsystem::getTurretEncoderCounts);
    Shuffleboard.getTab("Gradbot").addNumber("Turret Rotator Current Angle Degrees", turretGearPIDSubsystem::getAngleDegrees);
    Shuffleboard.getTab("Gradbot").addBoolean("Turret Gear At Setpoint", turretGearPIDSubsystem::atSetpoint);
    Shuffleboard.getTab("Gradbot").addNumber("Turret Target Angle", turretGearPIDSubsystem::getSetpoint);
    
    Shuffleboard.getTab("Gradbot").addBoolean("Arm At Setpoint Position", profiledPIDDiplomaArm::atGoal);
    Shuffleboard.getTab("Gradbot").addNumber("Arm Target Angle", profiledPIDDiplomaArm::getSetpointPosition);
    Shuffleboard.getTab("Gradbot").addNumber("Arm Encoder Counts", profiledPIDDiplomaArm::getArmEncoderCounts);
    Shuffleboard.getTab("Gradbot").addNumber("Arm Current Angle Degrees", profiledPIDDiplomaArm::getAngleFromHorizontalDegrees);



    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


        //y drives robot right
      //x drives is front
    
    new JoystickButton(m_joystick, 1).whenHeld(    //this time, we want the drivetraint to be stationary and only move when the tirggger is held
      new ManualMecanumDrive(() -> -m_joystick.getRawAxis(1)*0.65, 
    () -> m_joystick.getRawAxis(0)*0.65, 
    () -> m_joystick.getRawAxis(4)*0.65, mecanumDriveSubsystem));


  
    
    
/*
    new JoystickButton(buttonBox, 2).cancelWhenPressed(deliverDiplomaToStudent); //The button with the e
    new JoystickButton(buttonBox, 2).cancelWhenPressed(returnArmToPresident);
  */
    
    new JoystickButton(buttonBox, 6).whileActiveContinuous(new InstantCommand(profiledPIDDiplomaArm::moveForward, profiledPIDDiplomaArm)); //SW 1
    new JoystickButton(buttonBox, 7).whileActiveContinuous(new InstantCommand(profiledPIDDiplomaArm::moveBackward, profiledPIDDiplomaArm)); //SW 2
    new JoystickButton(buttonBox, 8).whileActiveContinuous(new InstantCommand(turretGearPIDSubsystem::moveForward, turretGearPIDSubsystem)); //SW 3
    new JoystickButton(buttonBox, 9).whileActiveContinuous(new InstantCommand(turretGearPIDSubsystem::moveBackward, turretGearPIDSubsystem)); //SW 4
    new JoystickButton(buttonBox, 10).whenHeld(deliverDiplomaToStudent); //SW 5
    new JoystickButton(buttonBox, 11).whenHeld(returnArmToPresident); //SW 6
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
