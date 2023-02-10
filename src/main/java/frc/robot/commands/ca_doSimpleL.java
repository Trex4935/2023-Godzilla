// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryContainer;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ca_doSimpleL extends SequentialCommandGroup {
  /** Creates a new ca_doSimpleL. */
  public ca_doSimpleL(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double squareAngle = 95;
    addCommands(
        new ca_autoTurnKinematic(drivetrain, 0.0, squareAngle),
        new ca_autoTrajectory(drivetrain, TrajectoryContainer.trajectoryFront,TrajectoryContainer.trajFrontEnd),
        new ca_autoTurnKinematic(drivetrain, 0.0, squareAngle),
        new ca_autoTrajectory(drivetrain, TrajectoryContainer.trajectoryFront, TrajectoryContainer.trajFrontEnd)

    );
  }
}
