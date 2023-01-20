package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeAndDepositCommand extends SequentialCommandGroup {
    public IntakeAndDepositCommand(Robot robot){
        super(
               new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
               new WaitCommand(250),
               new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
               new WaitCommand(500),
               new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(1500),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE))
        );
    }
}
