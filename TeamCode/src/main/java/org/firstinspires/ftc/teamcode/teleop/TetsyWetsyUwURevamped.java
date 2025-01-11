package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.ryanNemesis;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.tetsyWetsy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.RobotCentricMecanumAction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.BotMotor;

@TeleOp
public abstract class TetsyWetsyUwURevamped extends LinearOpMode {
    boolean isPressed = false;
    @Override
    public void runOpMode(){

        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);

        waitForStart();
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                tetsyWetsy.triggeredDynamicAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_down),1),
                new UpdateTelemetryAction()
        );
    }
}
