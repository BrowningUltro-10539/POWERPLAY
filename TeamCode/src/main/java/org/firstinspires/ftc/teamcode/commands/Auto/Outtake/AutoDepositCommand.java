package org.firstinspires.ftc.teamcode.commands.Auto.Outtake;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class AutoDepositCommand extends SequentialCommandGroup {
    public AutoDepositCommand(Robot robot){
        super(
            new LiftPositionCommand(robot.lift, 1600, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.TRANSITION))),
            new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
            new WaitCommand(750),
            new LiftPositionCommand(robot.lift, 0, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE)))
        );
    }
}
