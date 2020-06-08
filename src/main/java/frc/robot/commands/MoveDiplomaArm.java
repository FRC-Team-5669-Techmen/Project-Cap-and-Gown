/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.DiplomaArmSubsystemConstants;
import frc.robot.subsystems.DiplomaArmProfiledPIDSubsystem;

public class MoveDiplomaArm extends CommandBase {
  /**
   * Creates a new MoveArmToStudent. Handles interrupt in movement by stopping arm at last position 
   * at time the command is interrupted.
   */

  private final DiplomaArmProfiledPIDSubsystem diplomaArm;
  private final double angleDegrees;

  public MoveDiplomaArm(double angleDegrees, DiplomaArmProfiledPIDSubsystem diplomaArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.diplomaArm = diplomaArm;
    this.angleDegrees = angleDegrees;
    addRequirements(diplomaArm);
    setName("Move Diploma Arm to student");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if((double)angleDegrees>= DiplomaArmSubsystemConstants.MIN_ANGLE_Q1_DEGREES && angleDegrees <= DiplomaArmSubsystemConstants.MAX_ANGLE_Q2_DEGREES)
    {
      diplomaArm.setGoal(angleDegrees);
      diplomaArm.enable();
    }
    
   // else
       //new PrintCommand("Out of Movement Range");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //handled by PIDF subsystem controller
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    /*
    if(interrupted)
      diplomaArm.setGoal(diplomaArm.getAngleFromHorizontalDegrees()); //stop arm at last position
    */

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return diplomaArm.atGoal();
  }

  
}
