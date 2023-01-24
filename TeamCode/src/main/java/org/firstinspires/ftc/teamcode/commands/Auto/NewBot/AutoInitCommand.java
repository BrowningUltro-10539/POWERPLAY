package org.firstinspires.ftc.teamcode.commands.Auto.NewBot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class AutoInitCommand extends SequentialCommandGroup {
    public AutoInitCommand(Robot robot){
        super(
           new LiftPositionCommand(robot.lift, 2, 2),
           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)),
           new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
        );
    }
}
