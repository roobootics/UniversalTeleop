package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.Condition;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.DoubleFunction;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.ShortFunction;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.StrafeToLinearHeading;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.WaitSeconds;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.TurnTo;
import org.firstinspires.ftc.teamcode.teleop.LambdaInterfaces.RoadrunnerFunction;
import org.firstinspires.ftc.teamcode.teleop.TeleOpComponents.BotMotor;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;

public abstract class TeleOpActions{
    public static TelemetryPacket packet = new TelemetryPacket();
    public static boolean isRRActive=false;
    public interface TeleOpAction extends Action {
        boolean repeatFromStart(@NonNull TelemetryPacket packet);
        void stop();
    }
    public static class ConditionalAction implements TeleOpAction{
        LinkedHashMap<Condition,TeleOpAction> actions = new LinkedHashMap<>();
        TeleOpAction currentAction = null;
        public ConditionalAction(Condition[] conditions, TeleOpAction[] actions){
            for (int i=0;i<=conditions.length;i++){
                this.actions.put(conditions[i],actions[i]);
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean diffAction = false;
            for (Condition condition : actions.keySet()){
                if (condition.call()){
                    if (currentAction != null && actions.get(condition)!=currentAction){
                        currentAction.stop();
                        diffAction=true;
                    }
                    currentAction=actions.get(condition);
                    break;
                }
            }
            if (currentAction != null) {
                if (diffAction){
                    if(!currentAction.repeatFromStart(packet)){
                        currentAction=null;
                        return false;
                    }
                    else{
                        return true;
                    }
                }
                else {
                    if (!currentAction.run(packet)) {
                        currentAction = null;
                        return false;
                    } else {
                        return true;
                    }
                }
            }
            return false;
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            boolean reset = false;
            for (Condition condition : actions.keySet()){
                if (condition.call()){
                    if (currentAction != null && actions.get(condition)!=currentAction){
                        currentAction.stop();
                    }
                    currentAction=actions.get(condition);
                    reset=true;
                    break;
                }
            }
            if (currentAction != null) {
                if (reset) {
                    if (!currentAction.repeatFromStart(packet)) {
                        currentAction = null;
                        return false;
                    } else {
                        return true;
                    }
                }
                else{
                    if (!currentAction.run(packet)) {
                        currentAction = null;
                        return false;
                    } else {
                        return true;
                    }
                }
            }
            return false;
        }
        @Override
        public void stop() {
            if (currentAction != null){
                currentAction.stop();
            }
        }
    }

    public static class UninterruptibleConditionalAction implements TeleOpAction{
        LinkedHashMap<Condition,TeleOpAction> actions;
        TeleOpAction currentAction = null;
        public UninterruptibleConditionalAction(Condition[] conditions, TeleOpAction[] actions){
            for (int i=0;i<=conditions.length;i++){
                this.actions.put(conditions[i],actions[i]);
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (currentAction == null) {
                for (Condition condition : actions.keySet()) {
                    if (condition.call()) {
                        currentAction = actions.get(condition);
                        break;
                    }
                }
            }
            if (currentAction != null) {
                if(!currentAction.run(packet)){
                    currentAction = null;
                    return false;
                }
                else{
                    return true;
                }
            }
            return false;
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            return run(packet);
        }
        @Override
        public void stop() {
            if (currentAction != null){
                currentAction.stop();
            }
        }
    }

    public static class sleepWhileTrue implements TeleOpAction{
        public Condition condition;
        public double timeout;
        private final ElapsedTime timeOutTimer = new ElapsedTime();
        public boolean isStart = true;
        public sleepWhileTrue(Condition condition, double timeout){
            this.condition=condition;
            this.timeout=timeout;
        }
        public sleepWhileTrue(Condition condition){
            this(condition, Double.POSITIVE_INFINITY);
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            isStart=true;
            return run(packet);
        }

        @Override
        public void stop() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isStart){
                timeOutTimer.reset();
            }
            return condition.call() && timeOutTimer.time()<timeout;
        }
    }

    public static class TeleOpSequentialAction implements TeleOpAction{
        public List<TeleOpAction> remainingActions;
        public List<TeleOpAction> actions;
        public TeleOpSequentialAction(TeleOpAction...actions){
            this.actions = Arrays.asList(actions);
            this.remainingActions=new ArrayList<>(this.actions);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!remainingActions.isEmpty()){
                if (!remainingActions.get(0).run(packet)){
                    remainingActions.remove(0);
                }
                return true;
            }
            else{
                return false;
            }
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            if (!this.remainingActions.isEmpty()){
                this.remainingActions.get(0).stop();
            }
            this.remainingActions=new ArrayList<>(this.actions);
            return run(packet);
        }
        @Override
        public void stop() {
            if (!this.remainingActions.isEmpty()){
                this.remainingActions.get(0).stop();
            }
        }
    }
    public static class TeleOpParallelAction implements TeleOpAction{
        public List<TeleOpAction> remainingActions;
        public List<TeleOpAction> actions;
        public TeleOpParallelAction(TeleOpAction...actions){
            this.actions = Arrays.asList(actions);
            this.remainingActions=new ArrayList<>(this.actions);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!remainingActions.isEmpty()){
                remainingActions.removeIf(action -> !action.run(packet));
                return true;
            }
            else{
                return false;
            }
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            for (TeleOpAction action : remainingActions){
                action.stop();
            }
            this.remainingActions=new ArrayList<>(this.actions);
            return run(packet);
        }
        @Override
        public void stop() {
            for (TeleOpAction action : remainingActions){
                action.stop();
            }
        }
    }

    public static class PressTrigger extends ConditionalAction{
        public boolean isPressed=false;
        public Condition modifyCondition(Condition condition){
            return () -> {
                if (condition.call()){
                    if (!isPressed) {
                        isPressed = true;
                        return true;
                    }
                    else return false;
                }
                else{
                    isPressed=false;
                    return false;
                }
            };
        }
        public PressTrigger(Condition[] conditions, TeleOpAction[] actions) {
            super(conditions, actions);
            this.actions.clear();
            for (int i=0;i<= conditions.length;i++){
                this.actions.put(modifyCondition(conditions[i]),actions[i]);
            }
        }
    }
    public static class UninterruptiblePressTrigger extends UninterruptibleConditionalAction{
        public boolean isPressed=false;

        public Condition modifyCondition(Condition condition){
            return () -> {
                if (condition.call()){
                    if (!isPressed) {
                        isPressed = true;
                        return true;
                    }
                    else return false;
                }
                else{
                    isPressed=false;
                    return false;
                }
            };
        }
        public UninterruptiblePressTrigger(Condition[] conditions, TeleOpAction[] actions) {
            super(conditions, actions);
            this.actions.clear();
            for (int i=0;i<= conditions.length;i++){
                this.actions.put(modifyCondition(conditions[i]),actions[i]);
            }
        }
    }

    public static class FieldCentricMecanumAction implements TeleOpAction{
        private final DoubleFunction xFun;
        private final DoubleFunction yFun;
        private final DoubleFunction rxFun;
        private final Condition slowDownFun;
        private final BotMotor[] motors;
        private final IMU[] imu;
        public FieldCentricMecanumAction(BotMotor[] motors, IMU[] imu, DoubleFunction xFun, DoubleFunction yFun, DoubleFunction rxFun, Condition slowDownFun){
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors=motors;
            this.imu=imu;
        }
        public FieldCentricMecanumAction(BotMotor[] motors, IMU[] imu, DoubleFunction xFun, DoubleFunction yFun, DoubleFunction rxFun){
            this(motors,imu,xFun,yFun,rxFun,null);
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            return run(packet);
        }
        @Override
        public void stop() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double y = -yFun.call();
            double x = xFun.call();
            double rx = rxFun.call();

            double botHeading = imu[0].getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowDownFun!=null && slowDownFun.call()) { // Checks for left trigger input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return false;
        }
    }
    public static class RobotCentricMecanumAction implements TeleOpAction{
        private final DoubleFunction xFun;
        private final DoubleFunction yFun;
        private final DoubleFunction rxFun;
        private final Condition slowDownFun;
        private final BotMotor[] motors;
        public RobotCentricMecanumAction(BotMotor[] motors, DoubleFunction xFun, DoubleFunction yFun, DoubleFunction rxFun, Condition slowDownFun){
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors=motors;
        }
        public RobotCentricMecanumAction(BotMotor[] motors, DoubleFunction xFun, DoubleFunction yFun, DoubleFunction rxFun){
            this(motors,xFun,yFun,rxFun,null);
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            return run(packet);
        }
        @Override
        public void stop() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double y = -yFun.call();
            double x = xFun.call();
            double rx = rxFun.call();

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowDownFun!=null && slowDownFun.call()) { // Checks for left trigger input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return false;
        }
    }
    public static class UpdateTelemetryAction implements TeleOpAction{
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            return run(packet);
        }

        @Override
        public void stop() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            TeleOpComponents.telemetry.addData("servo pos",TeleOpComponents.tetsyWetsy.getPosition());
            TeleOpComponents.telemetry.update();
            return true;
        }
    }
    public static class ShortAction implements TeleOpAction{
        ShortFunction action;
        public ShortAction(ShortFunction action){this.action=action;}

        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            return run(packet);
        }

        @Override
        public void stop() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            action.call();
            return false;
        }
    }
    public static class TeleOpSleepAction implements TeleOpAction{
        boolean isStart=true;
        double time;
        ElapsedTime timer = new ElapsedTime();
        public TeleOpSleepAction(double time){
            this.time=time;
        }
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            isStart=true;
            return run(packet);
        }
        @Override
        public void stop() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isStart){
                isStart=false;
                timer.reset();
            }
            return timer.time()<time;
        }
    }
    public static class TeleOpTrajectoryAction implements TeleOpAction{
        TrajectoryActionBuilder trajBuilder;
        Action traj;
        LinkedHashMap<RoadrunnerFunction,Object[]> funcs = new LinkedHashMap<>();
        boolean isStart=true;
        @Override
        public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
            isStart=true;
            return run(packet);
        }
        @Override
        public void stop() {
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightFront.setPower(0);
            drive.rightBack.setPower(0);
            isRRActive=false;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart){
                isStart=false;
                drive.updatePoseEstimate();
                trajBuilder = drive.actionBuilder(drive.pose);
                for (RoadrunnerFunction action : funcs.keySet()){
                    if (action instanceof StrafeToLinearHeading){
                        StrafeToLinearHeading castedAction = (StrafeToLinearHeading) action;
                        trajBuilder = castedAction.call((Vector2d) Objects.requireNonNull(funcs.get(action))[0],(double) Objects.requireNonNull(funcs.get(action))[1]);
                    }
                    else if (action instanceof WaitSeconds){
                        WaitSeconds castedAction = (WaitSeconds) action;
                        trajBuilder = castedAction.call((double) Objects.requireNonNull(funcs.get(action))[0]);
                    }
                }
                traj = trajBuilder.build();
            }
            isRRActive=true;
            return traj.run(packet);
        }
        public TeleOpTrajectoryAction strafeToLinearHeading(Vector2d vector, double heading){
            StrafeToLinearHeading func = trajBuilder::strafeToLinearHeading;
            funcs.put(func,new Object[]{vector,heading});
            return this;
        }
        public TeleOpTrajectoryAction waitSeconds(double time){
            WaitSeconds func = trajBuilder::waitSeconds;
            funcs.put(func,new Object[]{time});
            return this;
        }
        public TeleOpTrajectoryAction turnTo(double heading){
            TurnTo func = trajBuilder::turnTo;
            funcs.put(func,new Object[]{heading});
            return this;
        }
    }

    public static void runLoop(Condition opModeIsActive, Condition isStopRequested, TeleOpAction...actions){
        while (opModeIsActive.call()) {
            for (TeleOpAction action : actions) {
                action.repeatFromStart(packet);
            }
            for (BotMotor motor : TeleOpComponents.motors){
                if (Objects.equals(motor.MOVEMENT_MODE, "MOTION_PROFILE") && !motor.isStallResetting){
                    motor.runMotionProfileOnce();
                }
            }
            if (isStopRequested.call()) return;
        }
    }
}