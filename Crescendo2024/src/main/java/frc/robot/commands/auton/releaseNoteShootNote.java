// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.runIntake;
import frc.robot.commands.shootNote;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Launcher.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class releaseNoteShootNote extends ParallelCommandGroup {
  Intake m_intake;
  Launcher m_launcher;
  /** Creates a new releaseNoteShootNote. */
  public releaseNoteShootNote(Intake sentIntake, Launcher sentLauncher) {
    m_intake = sentIntake;
    m_launcher = sentLauncher;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new runIntake(sentIntake, IntakeConstants.intakeSpeed, true),
      new shootNote(sentLauncher, LauncherConstants.launcherSpeed)
    );
  }
}
