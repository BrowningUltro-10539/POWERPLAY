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
          new WaitCommand(100),
          new SequentialCommandGroup(
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                  new WaitCommand(75),
                  new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER))
          ),
          new WaitCommand(50),
          new ParallelCommandGroup(
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toPole),
                  new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)),
                  new LiftPositionCommand(robot.lift, 10, 2)
          ),
          new LiftPositionCommand(robot.lift, 23, 2),
          new WaitCommand(25),
          new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE)),
          new ParallelCommandGroup(
                  new TrajectoryFollowerCommand(robot.driveSubsystem, toReturn),
                  new LiftPositionCommand(robot.lift, returnSlideHeight, 2),
                  new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT))
          )

        );
    }
}
