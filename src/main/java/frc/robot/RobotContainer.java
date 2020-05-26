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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualMecanumDrive;
import frc.robot.subsystems.BlinkinLEDSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MecanumDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Joystick m_joystick = new Joystick(ControllerConstants.JOYSTICK_CONTROLLER_PORT);

  private InstantCommand displayStandby = new InstantCommand(blinkinLEDSubsystem::standby_mode, blinkinLEDSubsystem);

  private InstantCommand displayCSEEBlue = new InstantCommand(blinkinLEDSubsystem::csee_blue, blinkinLEDSubsystem);

  private InstantCommand displayMATYellow = new InstantCommand(blinkinLEDSubsystem::mat_yellow, blinkinLEDSubsystem);
  
  private InstantCommand displayMSETRed = new InstantCommand(blinkinLEDSubsystem::mset_red, blinkinLEDSubsystem);

  private InstantCommand displayIDEAGreen = new InstantCommand(blinkinLEDSubsystem::idea_green, blinkinLEDSubsystem);

  private InstantCommand displayACEOrange = new InstantCommand(blinkinLEDSubsystem::ace_orange, blinkinLEDSubsystem);



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
    
    

    /*
    mecanumDriveSubsystem.setDefaultCommand(
      //y drives robot right
      //x drives is front
      new ManualMecanumDrive(() -> -m_joystick.getRawAxis(1)*0.65, 
      () -> m_joystick.getRawAxis(0)*0.65, 
      () -> m_joystick.getRawAxis(4)*0.65, mecanumDriveSubsystem)); 
    */
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

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
