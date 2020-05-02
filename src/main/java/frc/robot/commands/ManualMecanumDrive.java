/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDriveSubsystem;

public class ManualMecanumDrive extends CommandBase {
  private MecanumDriveSubsystem m_drive;

  private final DoubleSupplier x, y, z;
  
  public ManualMecanumDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, MecanumDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    this.x = x;
    this.y = y;
    this.z = z;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveCartesian(x.getAsDouble(), y.getAsDouble(), z.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveCartesian(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
