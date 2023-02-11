package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class NewLiftPositionCommand extends CommandBase {
    private final double position;
    private final double max_v;
    private final double max_a;
    private final double allowed_error;

    private final LiftSubsystem lift;

    private ElapsedTime timer;

    public NewLiftPositionCommand(LiftSubsystem lift, double position, double v, double a, double allowed_error) {
        this.position = position;
        this.lift = lift;
        this.max_v = v;
        this.max_a = a;
        this.allowed_error = allowed_error;
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
            lift.newProfile(position, max_v, max_a);
        }


    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getLiftPos() - position) < allowed_error;
    }
}
