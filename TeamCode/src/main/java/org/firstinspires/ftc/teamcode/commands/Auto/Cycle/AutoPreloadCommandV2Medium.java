package org.firstinspires.ftc.teamcode.commands.Auto.Cycle;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.NewLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.rr.util.AutoConstants;

public class AutoPreloadCommandV2Medium extends SequentialCommandGroup {
    public AutoPreloadCommandV2Medium(Robot robot, TrajectorySequence toPole, TrajectorySequence toConeStack){
        super(
           new ParallelCommandGroup(
                    new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toPole),
                    new SequentialCommandGroup(
                            new WaitCommand(150),
                            new ParallelCommandGroup(
                                    new NewLiftPositionCommand(robot.lift, 11, 40, 50, 2),
                                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT))
                            ))
                   ),
           new WaitCommand(50),
           new ParallelCommandGroup(
                   new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.POLE_DUNK)),
                   new NewLiftPositionCommand(robot.lift, 9.5, 200, 200, 2)
           ),
           new WaitCommand(50),
           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
           new WaitCommand(100),
           new ParallelCommandGroup(
                   new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toConeStack),
                   new SequentialCommandGroup(
                           new WaitCommand(400),
                           new NewLiftPositionCommand(robot.lift, AutoConstants.SLIDE_HEIGHTS[0], 70, 90, 2)
                   ),
                   new SequentialCommandGroup(
                           new WaitCommand(1600),
                           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE))

                   )
           )
        );
    }
}
