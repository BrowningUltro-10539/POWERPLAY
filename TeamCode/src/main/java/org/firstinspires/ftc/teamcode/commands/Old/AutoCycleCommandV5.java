package org.firstinspires.ftc.teamcode.commands.Old;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class AutoCycleCommandV5 extends SequentialCommandGroup {
    public AutoCycleCommandV5(Robot robot, TrajectorySequence toPole, TrajectorySequence toReturn, double returnSlideHeight){
        super(
            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
            new WaitCommand(350),
            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
            new WaitCommand(75),
            new ParallelCommandGroup(
                    new NewLiftPositionCommand(robot.lift, 10, 40, 50, 2),
                    new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPole),
                    new SequentialCommandGroup(
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER))
                    )

//                    new SequentialCommandGroup(
//                            new WaitCommand(1200),
//                            new LiftPositionCommand(robot.lift, 24, 2)
//                    )

            ),
//                new WaitCommand(350),
                new NewLiftPositionCommand(robot.lift, 23, 40, 50, 2),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new NewLiftPositionCommand(robot.lift, 21, 40, 50, 2)
                ),
            new WaitCommand(200),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitCommand(50),
                            new NewLiftPositionCommand(robot.lift, returnSlideHeight, 40, 50, 2)

                    ),

                    new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toReturn)
            )
        );
    }
}
