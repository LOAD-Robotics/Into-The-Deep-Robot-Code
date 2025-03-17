/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
// For Gobilda odometry pods
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

// Testing to try to import the LOAD_Tools.java library
import org.firstinspires.ftc.teamcode.LOAD_Tools;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// Code ported from Blocks to Java so far:
// - Drivetrain - Robot Centric
// - Linear Slides
// - Front Arm
// - Grippers
// - Slappy Arm
// - Winch

@TeleOp(name="TeleOp_Main", group="Iterative_TeleOp")

public class TeleOp_Main extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare hardware variables
        // Declare drive motors
        private DcMotor driveFL;
        private DcMotor driveFR;
        private DcMotor driveBL;
        private DcMotor driveBR;

        // Declare slide motors
        private DcMotor slideL;
        private DcMotor slideR;

        // Declare winch motors
        private DcMotor winch1;
        private DcMotor winch2;

        // Declare servos
        private Servo FrontArm;
        private Servo FrontArmGripper;
        private Servo SlideGripper;
        private Servo HangingArm;
        private Servo SampleAligner;

        // Declare limit switches
        private DigitalChannel RLimitSwitch;
        private DigitalChannel LLimitSwitch;

        // Declare Gobilda Odometry Pod
        GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    // Declare code data variables
        // Variables to store values for the drivetrain
            double Y  = 0;
            double X  = 0;
            double rX = 0;
            double d = 0;
            int speedPercent = 65;
            int driveForwardActive = 0;
            double driveForwardBeginTime = 0;
        // Variable to store the movement power (speed) of the linear slides
            double SlidePow = 0;
        // Variable to store the movement speed of the front arm
            double ArmPos = 0;
        // Variables for storing the position of the grippers, as well as a millis()-based delay
            int FrontArmGripperPos = 0;
            int SlideGripperPos = 0;
            long OldTime = 0;
            int TopHangingArmPos = 0;
        // Variables for storing values for the hanging winches
            int WinchSpeed = 300;
            int winch1Pos;
            int winch2Pos;
            int MaxWinchPos;
        // Variables for storing zero offsets for the various servos
            int zeroOffset_Hanging = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // ---------------------------------------------------------------------------------------
        // Initialize drive motors
            driveFL  = hardwareMap.get(DcMotor.class, "FL");
            driveFR  = hardwareMap.get(DcMotor.class, "FR");
            driveBL  = hardwareMap.get(DcMotor.class, "BL");
            driveBR  = hardwareMap.get(DcMotor.class, "BR");
        // Initialize slide motors
            slideL   = hardwareMap.get(DcMotor.class, "Lslide");
            slideR   = hardwareMap.get(DcMotor.class, "Rslide");
        // Initialize winch motors
            winch1   = hardwareMap.get(DcMotor.class, "Winch1");
            winch2   = hardwareMap.get(DcMotor.class, "Winch2");
        // Initialize servos
            FrontArm = hardwareMap.get(Servo.class, "Front Arm");
            FrontArmGripper = hardwareMap.get(Servo.class, "Front Arm Gripper");
            SlideGripper = hardwareMap.get(Servo.class, "Slide Gripper");
            HangingArm = hardwareMap.get(Servo.class, "Hanging Arm");
            SampleAligner = hardwareMap.get(Servo.class, "Sample Aligner");
        // Initialize Limit Switches
            RLimitSwitch = hardwareMap.get(DigitalChannel.class, "Right Limit Switch");
            LLimitSwitch = hardwareMap.get(DigitalChannel.class, "Left Limit Switch");
        // Initialize Gobilda Odometry Pods
            odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            driveFL.setDirection(DcMotor.Direction.REVERSE);
            driveFR.setDirection(DcMotor.Direction.FORWARD);
            driveBL.setDirection(DcMotor.Direction.REVERSE);
            driveBR.setDirection(DcMotor.Direction.FORWARD);

        // Define the behavior of the slide motors
            slideL.setDirection(DcMotor.Direction.FORWARD);
            slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideR.setDirection(DcMotor.Direction.REVERSE);
            slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define the behavior of the winch motors
            winch1.setDirection(DcMotor.Direction.FORWARD);
            winch2.setDirection(DcMotor.Direction.FORWARD);

        // Define the behavior of the limit switches
            RLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
            LLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Configure the Gobilda Odometry Pods
            /*
            Set the odometry pod positions relative to the point that the odometry computer tracks around.
            The X pod offset refers to how far sideways from the tracking point the
            X (forward) odometry pod is. Left of the center is a positive number,
            right of center is a negative number. the Y pod offset refers to how far forwards from
            the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
            backwards is a negative number.
             */
            odo.setOffsets(-152.4, 187.325);
            /*
            Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
            the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
            If you're using another kind of odometry pod, fill in the function
            with the number of ticks per mm of your odometry pod.
             */
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            /*
            Set the direction that each of the two odometry pods count. The X (forward) pod should
            increase when you move the robot forward. And the Y (strafe) pod should increase when
            you move the robot to the left.
             */
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            /*
            Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
            The IMU will automatically calibrate when first powered on, but recalibrating before running
            the robot is a good idea to ensure that the calibration is "good".
            resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
            This is recommended before you run your autonomous, as a bad initial calibration can cause
            an incorrect starting value for x, y, and heading.
             */
            //odo.recalibrateIMU();
            odo.resetPosAndIMU();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        Request an update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
        */
        odo.update();

        UpdateDrivetrain();
        UpdateSlides();
        UpdateArmServo();
        UpdateGrippers();
        UpdateHangingArm();
        //UpdateWinches();
        UpdateSampleAligner();
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * This function handles the math for the mecanum drivetrain
     */
    private void UpdateSampleAligner() {
        double FrontArmPos = (FrontArm.getPosition() * 180);
        if (FrontArmPos < 11){
            SampleAligner.setPosition((double) 120 / 180);
        }else{
            SampleAligner.setPosition((double) 50 / 180);
        }

    }

    /**
     * This function handles the math for the mecanum drivetrain
     */
    private void UpdateDrivetrain() {

        telemetry.addData("-------------------------------------------", "-");
        if (gamepad1.right_trigger > 0.1) {
            speedPercent = 100;
            telemetry.addData("Driving Speed Percentage", "100%");
        } else if (gamepad1.left_trigger > 0.1) {
            speedPercent = 33;
            telemetry.addData("Driving Speed Percentage", "33%");
        } else {
            speedPercent = 66;
            telemetry.addData("Driving Speed Percentage", "66%");
        }

        Y = -gamepad1.left_stick_y;
        X = (gamepad1.left_stick_x * 1.1);
        rX = gamepad1.right_stick_x;
        d = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(rX))), 1));

        driveFL.setPower(((Y + X + rX) / d) * ((double) speedPercent / 100));
        driveBL.setPower(((Y - X + rX) / d) * ((double) speedPercent / 100));
        driveFR.setPower(((Y - X - rX) / d) * ((double) speedPercent / 100));
        driveBR.setPower(((Y + X - rX) / d) * ((double) speedPercent / 100));

        if (driveForwardActive == 1){
            driveFL.setPower(0.8);
            driveBL.setPower(0.8);
            driveFR.setPower(0.8);
            driveBR.setPower(0.8);
            if (System.currentTimeMillis() > driveForwardBeginTime){
                driveForwardActive = 2;
            }
        }else{
            driveFL.setPower(((Y + X + rX) / d) * ((double) speedPercent / 100));
            driveBL.setPower(((Y - X + rX) / d) * ((double) speedPercent / 100));
            driveFR.setPower(((Y - X - rX) / d) * ((double) speedPercent / 100));
            driveBR.setPower(((Y + X - rX) / d) * ((double) speedPercent / 100));
        }

        if (gamepad1.dpad_up && (driveForwardActive == 0)) {
            driveForwardActive = 1;
            driveForwardBeginTime = System.currentTimeMillis() + 50;
        }
        if (!gamepad1.dpad_up && (driveForwardActive == 2)){
            driveForwardActive = 0;
        }
    }

    /**
     * This function controls the movement of the linear slides
     */
    private void UpdateSlides() {
        SlidePow = Math.abs(gamepad2.right_stick_y * 1);
        if (gamepad2.right_stick_button) {
            slideL.setTargetPosition(-1545);
            slideR.setTargetPosition(-1545);
            slideL.setPower(1);
            slideR.setPower(1);
            slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            if (gamepad2.right_stick_y > 0.05) {
                slideL.setTargetPosition(0);
                slideR.setTargetPosition(0);
                slideL.setPower(SlidePow);
                slideR.setPower(SlidePow);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.right_stick_y < -0.05) {
                slideL.setTargetPosition(-5000);
                slideR.setTargetPosition(-5000);
                slideL.setPower(SlidePow);
                slideR.setPower(SlidePow);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slideR.setTargetPosition(slideR.getCurrentPosition());
                slideL.setTargetPosition(slideL.getCurrentPosition());
                slideR.setPower(1);
                slideL.setPower(1);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (slideR.getCurrentPosition() > -25 && !(gamepad2.right_stick_y < -0.05)) {
                ((DcMotorEx) slideL).setMotorDisable();
                ((DcMotorEx) slideR).setMotorDisable();
            } else {
                ((DcMotorEx) slideL).setMotorEnable();
                ((DcMotorEx) slideR).setMotorEnable();
            }
        }
        telemetry.addData("-------------------------------------------", "-");
        telemetry.addData("Slide Power", SlidePow);
        telemetry.addData("Left Slide Power", slideL.getPower());
        telemetry.addData("Left Slide Position", slideL.getCurrentPosition());
        telemetry.addData("Left Slide Target", slideL.getTargetPosition());
        telemetry.addData("Left Slide Current", ((DcMotorEx) slideL).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Slide Power", slideR.getPower());
        telemetry.addData("Right Slide Position", slideR.getCurrentPosition());
        telemetry.addData("Right Slide Target", slideR.getTargetPosition());
        telemetry.addData("Right Slide Current", ((DcMotorEx) slideR).getCurrent(CurrentUnit.AMPS));
    }

    /**
     * This function handles the movement for the front arm
     */
    private void UpdateArmServo() {
        if (Math.abs(gamepad2.left_stick_y) >= 0.05) {
            // When the stick exits the dead zone, change the arm pos based on the angle of the stick
            ArmPos -= (gamepad2.left_stick_y * 5);
        }
        // When the left stick button is pressed, go to the correct angle to get a specimen off the wall
        if (gamepad2.left_stick_button){
            ArmPos = 23;
        }
        // Constrain the arm position to prevent it from breaking
        ArmPos = Math.min(Math.max(ArmPos, 10), 49);
        // Set the position of the servo
        FrontArm.setPosition(ArmPos / 180);
        // Telemetry
        telemetry.addData("-------------------------------------------", "-");
        telemetry.addData("Front Arm Speed", ArmPos);
        telemetry.addData("Front Arm Position", FrontArm.getPosition() * 180);
    }

    /**
     * This function handles the opening and closing of both the front arm and slide grippers
     */
    private void UpdateGrippers() {
        if (gamepad2.b) {
            // When B is pressed, close the bottom gripper and open the top gripper
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            OldTime = System.currentTimeMillis() + 250;
            FrontArmGripperPos = 180;
            SlideGripperPos = 50;
        } else {
            // When B is not pressed, open the bottom gripper and close the top gripper
            SlideGripperPos = 130;
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            if (OldTime <= System.currentTimeMillis()) {
                FrontArmGripperPos = 120;
            }
        }
        if (!((slideL.getCurrentPosition() + slideR.getCurrentPosition()) / 2 >= -250)) {
            FrontArmGripperPos = 120;
        }
        // Set the positions of the servos
        FrontArmGripper.setPosition((double) FrontArmGripperPos / 180);
        SlideGripper.setPosition((double) SlideGripperPos / 180);
        // Telemetry
        telemetry.addData("-------------------------------------------", "-");
        telemetry.addData("Front Arm Gripper Position", FrontArmGripperPos);
        telemetry.addData("Slide Gripper Position", SlideGripperPos);
    }

    /**
     * This function handles the movement of the slappy arms for hanging
     */
    private void UpdateHangingArm() {
        if (gamepad2.left_bumper) {
            // When the left bumper is pressed, move the hanging arm in one direction
            TopHangingArmPos += 6;
        } else if (gamepad2.right_bumper) {
            // When the right bumper is pressed, move the hanging arm in the other direction
            TopHangingArmPos -= 6;
        }
        // Constrain the arm position to prevent it from breaking
        TopHangingArmPos = Math.min(Math.max(TopHangingArmPos, 102), 215);
        // Set the position of the servo
        HangingArm.setPosition((double) (Math.abs(TopHangingArmPos - 300) + zeroOffset_Hanging) / 300);
        // Telemetry
        telemetry.addData("-------------------------------------------", "-");
        telemetry.addData("Hanging Arm Position", TopHangingArmPos);
    }

    /**
     * This function handles control of the hanging winches
     */
    private void UpdateWinches() {

        WinchSpeed = 300;
        if (gamepad2.a) {
            winch1Pos += WinchSpeed;
        }
        if (!(RLimitSwitch.getState() || LLimitSwitch.getState())) {
            // Wind Winch 1 In
            if (gamepad2.dpad_down) {
                winch2Pos += WinchSpeed;
            }
        } else {
            MaxWinchPos = winch2.getCurrentPosition() + 2;
            if (gamepad2.dpad_down && MaxWinchPos < winch2.getCurrentPosition()) {
                winch2Pos += WinchSpeed;
            }
        }
        winch1.setTargetPosition(winch1Pos);
        winch1.setPower(0.5);
        winch1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winch2.setTargetPosition(winch2Pos);
        winch2.setPower(0.7);
        winch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("-------------------------------------------", "-");
        telemetry.addData("Right Limit Switch State", RLimitSwitch.getState());
        telemetry.addData("Left Limit Switch State", LLimitSwitch.getState());
        telemetry.addData("Intendedwinch1Pos", winch1.getTargetPosition());
        telemetry.addData("Actualwinch1Pos", winch1.getCurrentPosition());
        telemetry.addData("Intendedwinch2Pos", winch2.getTargetPosition());
        telemetry.addData("Actualwinch2Pos", winch2.getCurrentPosition());
    }

}
