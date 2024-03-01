// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.liftIntakeEnc;
import frc.robot.commands.auton.runIntakeTimed;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class lowerIntakeRunIntake extends SequentialCommandGroup {
  IntakeLift m_IntakeLift;
  Intake m_Intake;
  /** Creates a new lowerIntakeRunIntake. */
  public lowerIntakeRunIntake(IntakeLift sentIntakeLift, Intake sentIntake) {
    m_IntakeLift = sentIntakeLift;
    m_Intake = sentIntake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new liftIntakeEnc(m_IntakeLift, IntakeConstants.liftDownLimit),
      new runIntakeTimed(m_Intake, 3)
    );
  }
}
