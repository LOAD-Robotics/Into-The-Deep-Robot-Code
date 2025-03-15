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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
// import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
// For Gobilda odometry pods
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

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

@Autonomous(name = "Auto_Specimens", group = "Iterative_Auto", preselectTeleOp = "TeleOp_Main")
public class Auto_Specimens extends OpMode
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

        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
        */
        Pose2D odometryPos = odo.getPosition();
        float[] pos = new float[3];
        pos[0] = (float) odometryPos.getX(DistanceUnit.MM);
        pos[1] = (float) odometryPos.getY(DistanceUnit.MM);
        pos[2] = (float) odometryPos.getHeading(AngleUnit.DEGREES);
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", odometryPos.getX(DistanceUnit.MM), odometryPos.getY(DistanceUnit.MM), odometryPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        Pose2D odometryVel = odo.getVelocity();
        float[] vel = new float[3];
        pos[0] = (float) odometryVel.getX(DistanceUnit.MM);
        pos[1] = (float) odometryVel.getY(DistanceUnit.MM);
        pos[2] = (float) odometryVel.getHeading(AngleUnit.DEGREES);
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odometryVel.getX(DistanceUnit.MM), odometryVel.getY(DistanceUnit.MM), odometryVel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
