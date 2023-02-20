//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.Intake;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class AutoStackCommand extends SequentialCommandGroup {
//    /* Assumes claw is open*/
//    public AutoStackCommand(Robot robot, double armPos){
//        super(
//
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.STRAIGHT)),
//                new InstantCommand(() -> robot.intake.setArm(armPos)),
//                new WaitCommand(400),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(300),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)),
//                new WaitCommand(275),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.ALIGNED)),
//                new WaitCommand(800),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(250),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.STRAIGHT)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.TRANSITION))
//                )
//        );
//    }
//}
