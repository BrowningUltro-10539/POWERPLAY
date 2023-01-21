package org.firstinspires.ftc.teamcode.testing.DriveCommandTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.NewAutoCommands.GrabConeFromStackCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.NewAutoCommands.LiftAndDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp
@Config
public class AutoTestingCommand extends CommandOpMode {
    private Robot robot;
    private ElapsedTime timer;
    private double loopTime = 0;

    private GamepadEx driver2Ex;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(0);
//        PhotonCore.enable();

        for(LynxModule module : robot.getControllers()){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver2Ex = new GamepadEx(gamepad2);

    }


    @Override
    public void run(){
        if(timer == null){
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

        /* Gamepad 1 Functions
        *  Driving - Left and Right Stick
        *  A and D-Pad Down = Turret.RIGHT AND LEFT POLE
        *  B and D-Pad Left = Turret.NINETY and NEG_NINETY
        *  Y and D-Pad Up = Adjust turret angle (right and left, respectively) -> Add to the target angle
        *  Left Bumper = Straighten the turret to be zero degrees
        *  Right Bumper = Drop and retract
        *  */

        /* Gamepad 2 Functions
        *
        *  A - Transfer cone
        *  Y - manual control for arm lifting up
        *  X - manual control for arm coming down
        *
        *
        *  DPad Up - High Point w/ Funnel in Transition Mode
        *  Dpad-Down - Put slides down if not down
        *  Dpad-Left - Put slides in some medium position
        *
        *  Left Bumper - override slide automation  - manual control for slides
        *  Left Stick - control the slides manually after a tri
        *
        *  Right Bumper - override turret automation - manual control for the intake turret
        *  Right Stick - manual control for the intake turret
        *
        *  Right and Left Triggers - Overrides claw and let's me open it up automatically

        * */
        // Add driving later

        if(gamepad1.a){
            schedule(new InstantCommand(() -> robot.intake.setArmTargetAngle(-75)));
        }


        if(gamepad1.b){
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.RotateState.INTAKE)).alongWith(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))));
        }

        if(gamepad1.y){
            schedule(new SequentialCommandGroup(
                    new LiftAndDropConeCommand(robot),
                    new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[0]),
                    new LiftAndDropConeCommand(robot),
                    new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[1]),
                    new LiftAndDropConeCommand(robot),
                    new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[2]),
                    new LiftAndDropConeCommand(robot),
                    new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[3]),
                    new LiftAndDropConeCommand(robot),
                    new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[4]),
                    new LiftAndDropConeCommand(robot)
            ));
        }

        if(gamepad1.x){
            schedule(new GrabConeFromStackCommand(robot, robot.intake.FIVE_STACK_POSITIONS[0]));
        }


        schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x));


        robot.intake.setLiftTurretState(robot.lift.turretState);
        robot.intake.setLiftTurretCurrentAngle(robot.lift.getTurretAngle());

        robot.intake.loop();
        robot.lift.loop();


        CommandScheduler.getInstance().run();

        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        for(LynxModule module : robot.getControllers()){
            module.clearBulkCache();
        }
    }

    @Override
    public void reset(){
        schedule(new InstantCommand(() -> robot.lift.update(LiftSubsystem.TurretState.STRAIGHT)));
        CommandScheduler.getInstance().reset();
    }


}
