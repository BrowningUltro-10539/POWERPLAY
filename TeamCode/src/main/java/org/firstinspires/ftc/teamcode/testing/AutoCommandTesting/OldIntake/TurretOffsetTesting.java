package org.firstinspires.ftc.teamcode.testing.AutoCommandTesting.OldIntake;//package org.firstinspires.ftc.teamcode.testing.AutoCommandTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//
//
//@TeleOp
//@Config
//public class TurretOffsetTesting extends CommandOpMode {
//    private Robot robot;
//    private ElapsedTime timer;
//    private double loopTime = 0;
//
//    /*
//    * Driver 1:
//    *   - Driving the robot
//    *   - Turret Control
//    *   - Funnel Control
//    * Driver 2:
//    *   - Control the arm
//    *   - Control the lift
//    *   - Control the transfer
//    *
//    *
//    * Commands needed:
//    *   - Turret change (change the FSM)
//    *   - Lift Change (PID tune the list, change the FSM)
//    *   - Arm Change - pickup and deposit, only if my toggle is on
//    *   - Auto retract - change the state of the slides to intake
//    *   - TOGGLE BETWEEN HIGH, MED, LOW for DRIVER 2 so this determines how the robot works with the intake alignment and how
//    *   - Test between
//    *
//    *
//    *
//    * */
//
//    // 25 90, -10 -90
//    public static double liftTurretAngle = 0;
//    public static double armTurretAngle = 0;
//
//    @Override
//    public void initialize(){
//        CommandScheduler.getInstance().reset();
//
//        robot = new Robot(hardwareMap, false);
//        robot.reset();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(0);
//        PhotonCore.enable();
//
//    }
//
//
//    @Override
//    public void run(){
//        if(timer == null){
//            timer = new ElapsedTime();
//            robot.reset();
//        }
//
//        robot.read();
//
//        if(gamepad1.a){
//            schedule(new InstantCommand(() -> robot.intake.setTurretTargetAngle(armTurretAngle)));
//        }
//
//        if(gamepad1.b){
//            schedule(new InstantCommand(() -> robot.lift.setTargetTurretAngle(liftTurretAngle)));
//        }
//
//        if(gamepad1.x){
//            schedule(new InstantCommand(() -> robot.intake.setTurretTargetAngle(0)));
//        }
//
//        if(gamepad1.y){
//            schedule(new InstantCommand(() -> robot.lift.setTargetTurretAngle(0)));
//        }
//
//        if(gamepad1.dpad_up){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.INTAKE)));
//        }
//
//        if(gamepad1.dpad_down){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ArmState.DEPOSIT)));
//        }
//
//        if(gamepad1.dpad_left){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
//        }
//
//        if(gamepad1.dpad_right){
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
//        }
//
//        if(gamepad1.left_bumper){
//            schedule(new InstantCommand(() -> robot.intake.setArm(0.6)));
//        }
//
//        schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.DEPOSIT)));
//
//
//        robot.intake.loop();
//        robot.lift.loop();
//
//        CommandScheduler.getInstance().run();
//
//        robot.write();
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("Lift Turret Angle: ", robot.lift.getTurretAngle());
//        telemetry.addData("Arm Turret Angle: ", robot.intake.getTurretAngle());
//        loopTime = loop;
//        telemetry.update();
//
//        PhotonCore.CONTROL_HUB.clearBulkCache();
//        PhotonCore.EXPANSION_HUB.clearBulkCache();
//    }
//
//
//}
