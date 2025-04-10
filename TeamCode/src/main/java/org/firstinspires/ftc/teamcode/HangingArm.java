package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangingArm {
    private final Servo arm;

    public HangingArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "Hanging Arm");
    }

    public class up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 185 / 300);
            return false;
        }
    }
    public Action up() {return new up();}

    public class bar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 200 / 300);
            return false;
        }
    }
    public Action bar() {return new bar();}

    public class down implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 225 / 300);
            return false;
        }
    }
    public Action down() {return new down();}
}
