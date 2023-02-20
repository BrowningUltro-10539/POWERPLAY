package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.IntakeCommands;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class ColorSensorTestCommand extends SequentialCommandGroup {
    public ColorSensorTestCommand(Robot robot){
        super(
              new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE))
        );
    }
}
