package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmGripper {
    private final Servo claw;

    public ArmGripper(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "Front Arm Gripper");
    }

    // Implementation of the action to close the gripper
    public class close implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition((double) 110 / 180);
            return false;
        }
    }
    public Action close() {return new close();}

    // Implementation of the action to open the gripper
    public class open implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition((double) 70 / 180);
            return false;
        }
    }
    public Action open() {return new open();}
}
