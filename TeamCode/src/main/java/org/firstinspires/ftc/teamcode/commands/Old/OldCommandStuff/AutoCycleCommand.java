//package org.firstinspires.ftc.teamcode.commands.Auto;
//
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake.AutoStackCommand;
//import org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Outtake.AutoDepositCommand;
//
//public class AutoCycleCommand extends SequentialCommandGroup {
//
//   private Robot robot;
//    public AutoCycleCommand(Robot robot, double armPos){
//        super(
//                new AutoDepositCommand(robot),
//             new AutoStackCommand(robot, armPos)
//
//        );
//
//        this.robot = robot;
//    }
//
//
//}
