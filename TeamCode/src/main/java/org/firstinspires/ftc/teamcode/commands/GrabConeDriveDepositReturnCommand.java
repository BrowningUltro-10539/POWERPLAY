package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

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
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),

                  new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.AUTO_STACK_RIGHT_POLE))

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
                  new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED))
          ),

          new ParallelCommandGroup(
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toReturn),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
          )


        );
    }
}
