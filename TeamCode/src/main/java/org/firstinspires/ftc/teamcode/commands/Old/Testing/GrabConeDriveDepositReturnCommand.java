package org.firstinspires.ftc.teamcode.commands.Old.Testing;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class GrabConeDriveDepositReturnCommand extends SequentialCommandGroup {
    public GrabConeDriveDepositReturnCommand(Robot robot, Trajectory toPole, Trajectory toReturn, double returnSlideHeight){
        super(
          new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
          new WaitCommand(200),
          new SequentialCommandGroup(
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                  new WaitCommand(200)
          ),
          new WaitCommand(175),
          new ParallelCommandGroup(
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toPole)
          ),
                new ParallelCommandGroup(
                        new LiftPositionCommand(robot.lift, 16.6, 2),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER))
                ),
          new WaitCommand(100),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DUNK)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new WaitCommand(500),
          new ParallelCommandGroup(
                  new LiftPositionCommand(robot.lift, returnSlideHeight, 2),

                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                  new InstantCommand(() -> robot.driveSubsystem.turn(Math.toRadians(42)))
          ),

          new WaitCommand(200),
          new ParallelCommandGroup(
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toReturn),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
          )


        );
    }
}
