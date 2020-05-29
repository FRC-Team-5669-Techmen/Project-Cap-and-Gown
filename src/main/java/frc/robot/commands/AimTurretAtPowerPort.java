/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AimtTurretAtPowerPortConstants;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.TurretRotatorSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimTurretAtPowerPort extends PIDCommand {

  private final TurretRotatorSubsystem m_turret;
  //also going to need to add the drivetrain for rotating that too
  /**
   * Creates a new LockOnToPowerPort.
   */
  public AimTurretAtPowerPort(double desiredAngleDegrees, TurretRotatorSubsystem turret){
    super(
        // The controller that the command will use
        //https://docs.wpilib.org/en/latest/docs/software/commandbased/pid-subsystems-commands.html#full-pidcommand-example
        new PIDController(AimtTurretAtPowerPortConstants.kP, 
                          AimtTurretAtPowerPortConstants.kI, 
                          AimtTurretAtPowerPortConstants.kD),  //Need to find these: 
        // This should return the measurement
        turret::getAngleDegrees, //note limelight limits for x -29.8 to 29.8 degrees
        // This should return the setpoint (can also be a constant)
        () -> desiredAngleDegrees,
        // This uses the output
        output -> {
          // Use the output here
            turret.setTurretRotatorMotorSpeed(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.m_turret = turret;
    addRequirements(m_turret);
     //https://docs.wpilib.org/en/latest/docs/software/commandbased/pid-subsystems-commands.html#full-pidcommand-example
    getController().enableContinuousInput(-180, 180); // It is an angle controller.
    getController().setTolerance(AimtTurretAtPowerPortConstants.TOLERANCE);

    //getController().setTolerance(positionTolerance);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint(); //if the target gets blocked by say, another robot, stop the command
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_turret.setTurretRotatorMotorSpeed(0.0);
  }
}
