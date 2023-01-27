package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class AutoDepositAndRetract extends SequentialCommandGroup {
    public AutoDepositAndRetract(Robot robot, double returnSlidePos){
        super(
                new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.RIGHT_POLE)),
                new LiftPositionCommand(robot.lift, 23, 2),
                new WaitCommand(400),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(200),
                new LiftPositionCommand(robot.lift, returnSlidePos, 2),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT))

                )
        );
    }
}
