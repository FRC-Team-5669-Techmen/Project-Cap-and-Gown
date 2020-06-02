/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretGearPIDSubsystem;

public class RotateTurret extends CommandBase {
  private final TurretGearPIDSubsystem turretGearPIDSubsystem;
  private final double angleDegrees;
  /**
   * Creates a new RotateTurret.
   */
  public RotateTurret(double angleDegrees, TurretGearPIDSubsystem turretGearPIDSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretGearPIDSubsystem = turretGearPIDSubsystem;
    this.angleDegrees = angleDegrees; 
    addRequirements(turretGearPIDSubsystem);
    setName("Rotate Turret");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretGearPIDSubsystem.setSetpoint(angleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //handled by PID controller
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted)
      turretGearPIDSubsystem.setSetpoint(turretGearPIDSubsystem.getAngleDegrees());//stop at last position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretGearPIDSubsystem.atSetpoint();
  }
}
