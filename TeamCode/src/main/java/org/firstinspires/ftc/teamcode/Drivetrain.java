package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drivetrain {
    private final DcMotor FL;
    private final DcMotor FR;
    private final DcMotor BL;
    private final DcMotor BR;
    private DcMotor.RunMode mode;

    public Drivetrain(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        mode = FL.getMode();

    }

    // Implementation of the action to close the gripper
    public class alignForward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            float speed = 0.5F;
            FL.setPower(speed);
            FR.setPower(speed);
            BL.setPower(speed);
            BR.setPower(speed);
            FL.setTargetPosition(FL.getCurrentPosition()+800);
            FR.setTargetPosition(FR.getCurrentPosition()+800);
            BL.setTargetPosition(BL.getCurrentPosition()+800);
            BR.setTargetPosition(BR.getCurrentPosition()+800);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action alignForward() {return new alignForward();}

    // Implementation of the action to close the gripper
    public class alignBackward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);
            FL.setTargetPosition(FL.getCurrentPosition()-320);
            FR.setTargetPosition(FR.getCurrentPosition()-320);
            BL.setTargetPosition(BL.getCurrentPosition()-320);
            BR.setTargetPosition(BR.getCurrentPosition()-320);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action alignBackward() {return new alignBackward();}

    public class stop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            FL.setMode(mode);
            FR.setMode(mode);
            BL.setMode(mode);
            BR.setMode(mode);
            return false;
        }
    }
    public Action stop() {return new stop();}
}
