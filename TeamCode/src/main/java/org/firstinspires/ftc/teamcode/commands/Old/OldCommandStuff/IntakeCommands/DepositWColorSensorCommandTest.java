//package org.firstinspires.ftc.teamcode.commands.Old.OldCommandStuff.IntakeCommands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//
//public class DepositWColorSensorCommandTest extends SequentialCommandGroup {
//    public DepositWColorSensorCommandTest(Robot robot){
//        super(
//
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.TRANSFER)),
//                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT))
//                )  ,
//
//                new WaitCommand(2000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
//                new WaitCommand(1000),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.INTAKE))
//        );
//    }
//}
