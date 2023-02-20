//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Outtake;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//public class DepositAndRetractCommand extends SequentialCommandGroup {
//    public DepositAndRetractCommand(Robot robot){
//        super(
//          new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
//                new WaitCommand(1000),
//                new LiftPositionCommand(robot.lift, 0, 10).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE)))
//        );
//    }
//}
