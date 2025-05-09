//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.NewAutoCommands;
//
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeSubsystem;
//
//public class GrabConeFromStackCommand extends SequentialCommandGroup {
//    public GrabConeFromStackCommand(Robot robot, IntakeSubsystem.DualAngle dualAngle){
//        super(
//          new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.setArmTargetAngle(dualAngle.getArmAngle())),
//                        new InstantCommand(() -> robot.intake.setRotate(dualAngle.getPivotPosition())),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))
//          ),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(1000),
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                        new WaitCommand(100),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.TRANSFER))
//                ),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(400),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.setRotate(0.12)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION))
//                )
//        );
//    }
//}
