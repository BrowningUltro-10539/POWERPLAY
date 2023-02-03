package org.firstinspires.ftc.teamcode.commands.Auto.Testing;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class AutoPickUpConeCommand extends SequentialCommandGroup {
    public AutoPickUpConeCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER)),
                new WaitCommand(50)
        );
    }
}
