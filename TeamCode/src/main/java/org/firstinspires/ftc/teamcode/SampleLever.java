package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SampleLever {
    private Servo lever;

    public SampleLever(HardwareMap hardwareMap) {
        lever = hardwareMap.get(Servo.class, "Sample Aligner");
    }

    // Implementation of the action to close the gripper
    public class up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lever.setPosition((double) 110 / 180);
            return false;
        }
    }
    public Action up() {return new up();}

    // Implementation of the action to open the gripper
    public class down implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lever.setPosition((double) 50 / 180);
            return false;
        }
    }
    public Action down() {return new down();}
}
