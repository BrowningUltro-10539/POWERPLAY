package org.firstinspires.ftc.teamcode.commands.Auto.NewAutoCommands;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class GrabConeFromStackCommand extends SequentialCommandGroup {
    public GrabConeFromStackCommand(Robot robot, IntakeSubsystem.DualAngle dualAngle){
        super(
          new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setArmTargetAngle(dualAngle.getArmAngle())),
                        new InstantCommand(() -> robot.intake.setPivot(dualAngle.getPivotPosition())),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
          ),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(1000),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                        new WaitCommand(100),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.TRANSFER))
                ),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(400),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setPivot(0.12)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION))
                )
        );
    }
}
