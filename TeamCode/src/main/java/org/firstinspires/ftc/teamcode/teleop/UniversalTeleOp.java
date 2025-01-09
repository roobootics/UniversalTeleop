package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.ryanNemesis;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.tetsyWetsy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.RobotCentricMecanumAction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.BotMotor;

@TeleOp
public abstract class UniversalTeleOp extends LinearOpMode {
    boolean isPressed = false;
    @Override
    public void runOpMode(){

        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);

        waitForStart();
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                //ryanNemesis.triggeredMoveToTargetAction(()->(gamepad1.dpad_up), ryanNemesis.target+15),
                //ryanNemesis.triggeredMoveToTargetAction(()->(gamepad1.dpad_down), ryanNemesis.target-15),
                tetsyWetsy.triggeredDynamicAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_up),1),
                //new RobotCentricMecanumAction(new BotMotor[]{}, ()->(gamepad1.left_stick_x), ()->(gamepad1.left_stick_y), ()->(gamepad1.right_stick_x), ()->(gamepad1.left_trigger==1))
                new UpdateTelemetryAction()
        );
    }
}
