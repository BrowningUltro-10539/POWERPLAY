package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class AutoCycleCommandV5 extends SequentialCommandGroup {
    public AutoCycleCommandV5(Robot robot, TrajectorySequence toPole, TrajectorySequence toReturn, double returnSlideHeight){
        super(
            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
            new WaitCommand(400),
            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
            new WaitCommand(75),
            new ParallelCommandGroup(
                    new LiftPositionCommand(robot.lift, 7, 2),
                    new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPole),
                    new SequentialCommandGroup(
                            new WaitCommand(150),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER))
                    )

            ),
                new LiftPositionCommand(robot.lift, 25, 2),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new LiftPositionCommand(robot.lift, 20, 2)
                ),
            new WaitCommand(350),
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitCommand(50),
                            new LiftPositionCommand(robot.lift, returnSlideHeight, 2)

                    ),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                    new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toReturn)
            )
        );
    }
}
