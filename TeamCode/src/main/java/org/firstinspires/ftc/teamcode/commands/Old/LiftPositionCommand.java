package org.firstinspires.ftc.teamcode.commands.Old;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystem.LiftSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private final double error;

    private final LiftSubsystem lift;

    private ElapsedTime timer;

    public LiftPositionCommand(LiftSubsystem lift, double position, double error) {
        this.position = position;
        this.lift = lift;
        this.error = error;


    }

    @Override
    public void execute() { lift.setTargetLiftPosition(position); }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getLiftPos() - position) < error;
    }
}
