package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometryPuller {
    private final Servo puller;

    public OdometryPuller(HardwareMap hardwareMap) {
        puller = hardwareMap.get(Servo.class, "Odometry Puller");
    }

    // Implementation of the action to close the gripper
    public class up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            puller.setPosition(0.52);
            return false;
        }
    }
    public Action up() {return new up();}

    // Implementation of the action to open the gripper
    public class down implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            puller.setPosition(0.462);
            return false;
        }
    }
    public Action down() {return new down();}
}
