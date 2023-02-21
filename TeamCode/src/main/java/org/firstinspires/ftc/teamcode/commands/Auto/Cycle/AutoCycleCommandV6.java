package org.firstinspires.ftc.teamcode.commands.Auto.Cycle;

import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;

public class AutoCycleCommandV6 extends SequentialCommandGroup {
    public AutoCycleCommandV6(Robot robot, TrajectorySequence toPole, TrajectorySequence toConeStack, double slideHeight){
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(50),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
                new WaitCommand(25),
                new ParallelCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPole),
                        new SequentialCommandGroup(
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
                                new NewLiftPositionCommand(robot.lift, 21.5, 200, 200, 2)
                        )
                ),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new WaitCommand(50),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new NewLiftPositionCommand(robot.lift, 20, 200, 200, 2)
                ),
                new WaitCommand(50),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                        new NewLiftPositionCommand(robot.lift, slideHeight, 200, 200, 2),
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStack)
                )

        );
    }
}
