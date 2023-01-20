package org.firstinspires.ftc.teamcode.commands.IntakeCommands;//package org.firstinspires.ftc.teamcode.commands.IntakeCommands;
//
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class ServoWithMotionProfileCommand extends SequentialCommandGroup {
//    public ServoWithMotionProfileCommand(Robot robot){
//        super(
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION)),
//                new IntakePositionCommand(robot.intake, 100, 3000, 2000, 5, 2000)
//        );
//    }
//}
