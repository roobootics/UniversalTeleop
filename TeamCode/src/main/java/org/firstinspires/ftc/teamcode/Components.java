package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;


public class Components {
    HardwareMap hardwareMap;
    public BotMotor extendo = new BotMotor(hardwareMap.dcMotor.get("extendo").getController(),hardwareMap.dcMotor.get("extendo").getPortNumber(),hardwareMap.dcMotor.get("extendo").getMotorType());
    public static ArrayList<BotMotor> motors = new ArrayList<>();
    public class BotMotor extends DcMotorImplEx {

        public BotMotor(DcMotorController controller, int portNumber, @NonNull MotorConfigurationType motorType) {
            super(controller, portNumber, DcMotor.Direction.FORWARD,motorType);
            hardwareMap.put("extendo",this);
            motors.add(this);
        }
    }
    public class BotServo extends ServoImplEx {

        public BotServo(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
            super(controller, portNumber, servoType);
        }
    }
}
