//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.NewAutoCommands;
//
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.Old.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//public class LiftAndDropConeCommand extends SequentialCommandGroup {
//    public LiftAndDropConeCommand(Robot robot){
//        super(
//                new LiftPositionCommand(robot.lift, 23, 2).alongWith(new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.TRANSITION))),
//                new InstantCommand(() -> robot.lift.outtakeClaw.setPosition(0.5)),
//                new WaitCommand(600),
//                new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.OUTTAKE)),
//                new WaitCommand(600),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.FunnelState.INTAKE)),
//                        new LiftPositionCommand(robot.lift, 0, 2)
//                )
//
//        );
//    }
//}
