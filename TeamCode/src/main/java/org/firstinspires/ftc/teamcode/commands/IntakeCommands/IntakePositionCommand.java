package org.firstinspires.ftc.teamcode.commands.IntakeCommands;//package org.firstinspires.ftc.teamcode.commands.IntakeCommands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class IntakePositionCommand extends CommandBase {
//    private final double position;
//    private final double timeout;
//    private final double max_v;
//    private final double max_a;
//    private final double allowed_error;
//
//    private final IntakeSubsystem intake;
//
//
//    private ElapsedTime timer;
//
//    public IntakePositionCommand(IntakeSubsystem intake, double position, double v, double a,
//                                 double allowed_error, double timeout) {
//        this.position = position;
//        this.timeout = timeout;
//        this.intake = intake;
//        this.max_v = v;
//        this.max_a = a;
//        this.allowed_error = allowed_error;
//
//    }
//
//    @Override
//    public void execute() {
//        if (timer == null) {
//            timer = new ElapsedTime();
//            intake.newProfile(position, max_v, max_a);
//        }
//
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        return Math.abs(intake.getPos() - position) < allowed_error || timer.milliseconds() > timeout;
//    }
//}
