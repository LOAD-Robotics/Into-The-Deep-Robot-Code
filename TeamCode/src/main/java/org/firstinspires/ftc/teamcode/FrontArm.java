package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private Servo arm;

    public FrontArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "Front Arm");
    }

    // Move the arm all the way up
    public class up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 16.5 / 180);
            return false;
        }
    }
    public Action up() {return new up();}

    // Move the arm all the way down
    public class floor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 54.25 / 180);
            return false;
        }
    }
    public Action floor() {return new floor();}

    // Move the arm to the best pos to get specimens off the wall
    public class wall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition((double) 32 / 180);
            return false;
        }
    }
    public Action wall() {return new wall();}
}
