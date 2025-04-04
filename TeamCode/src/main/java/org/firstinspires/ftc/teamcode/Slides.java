package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Slides {
    private DcMotor motor1;
    private DcMotor motor2;

    public Slides (HardwareMap hardwareMap){
        motor1 = hardwareMap.get(DcMotor.class, "Lslide");
        motor2 = hardwareMap.get(DcMotor.class, "Rslide");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class resetZero implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return false;
        }
    }
    public Action resetZero(){return new resetZero();}

    public class zero implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(0);
            motor2.setTargetPosition(0);
            motor1.setPower(0.8);
            motor2.setPower(0.8);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (((DcMotorEx) motor1).getCurrent(CurrentUnit.AMPS) > 7 || ((DcMotorEx) motor2).getCurrent(CurrentUnit.AMPS) > 7) {
                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            return false;
        }
    }
    public Action zero(){return new zero();}

    public class highBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(-5000);
            motor2.setTargetPosition(-5000);
            motor1.setPower(1);
            motor2.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action highBasket(){return new highBasket();}

    public class lowBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(0);
            motor2.setTargetPosition(0);
            motor1.setPower(1);
            motor2.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action lowBasket(){return new lowBasket();}

    public class highChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(-1622);
            motor2.setTargetPosition(-1622);
            motor1.setPower(1);
            motor2.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action highChamber(){return new highChamber();}

    public class highChamberClip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(-795);
            motor2.setTargetPosition(-795);
            motor1.setPower(1);
            motor2.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action highChamberClip(){return new highChamberClip();}

    public class lowChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            motor1.setTargetPosition(0);
            motor2.setTargetPosition(0);
            motor1.setPower(1);
            motor2.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }
    public Action lowChamber(){return new lowChamber();}
    
}
