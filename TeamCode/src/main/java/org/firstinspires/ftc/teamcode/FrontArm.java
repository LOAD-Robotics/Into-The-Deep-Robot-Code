package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo arm;

    public FrontArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "Front Arm");
    }

    // Move the arm all the way up
    public class up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition(21.5 / 180);
            return false;
        }
    }
    public Action up() {return new up();}

    // Move the arm all the way down
    public class floor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 60 / 180);
            return false;
        }
    }
    public Action floor() {return new floor();}

    // Move the arm to the best pos to get specimens off the wall
    public class wall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition(37.0 / 180);
            return false;
        }
    }
    public Action wall() {return new wall();}
}
