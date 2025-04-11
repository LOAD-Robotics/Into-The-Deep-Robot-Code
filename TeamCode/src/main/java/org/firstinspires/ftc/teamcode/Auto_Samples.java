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

// NON RR
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MecanumDrive;

// RR CRAP
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// THIS ADDS IT TO DRIVER HUB MENU \/
@Autonomous(name="Auto_Samples", group="Linear OpMode", preselectTeleOp = "TeleOp_Main")
// ADD @Disabled to disable

public class Auto_Samples extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-32,-61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Create the hardware instances
        Slides slides = new Slides(hardwareMap);
        TopGripper slideGripper = new TopGripper(hardwareMap);
        ArmGripper armGripper = new ArmGripper(hardwareMap);
        FrontArm arm = new FrontArm(hardwareMap);
        SampleLever lever = new SampleLever(hardwareMap);
        HangingArm hangingArm = new HangingArm(hardwareMap);
        OdometryPuller odometryPuller = new OdometryPuller(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Insert code to be run once when START is pressed
        runtime.reset();


        Actions.runBlocking(
                new SequentialAction(
                        odometryPuller.down(),
                        hangingArm.down(),
                        arm.up(),
                        armGripper.close(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highBasket(),
                        drive.actionBuilder(initialPose).waitSeconds(1.3).build(),
                        drive.actionBuilder(initialPose)
                                .strafeToLinearHeading(new Vector2d(-57, -57), Math.toRadians(225))
                                .build(),
                        slideGripper.open(),
                        // Sample 1 Scored
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(-59,-41), Math.toRadians(83.2))
                                .build(),
                        new ParallelAction(
                                slides.zero(),
                                lever.down(),
                                arm.floor()
                        ),
                        drive.actionBuilder(initialPose).waitSeconds(1.4).build(),
                        armGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        arm.up(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(1.9).build(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highBasket(),
                        drive.actionBuilder(initialPose).waitSeconds(1.75).build(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(-57, -57), Math.toRadians(250))
                                .build(),
                        drive.actionBuilder(initialPose).waitSeconds(0.4).build(),
                        slideGripper.open(),
                        // Sample 2 Scored
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(-48.5, -41.5), Math.toRadians(80))
                                .build(),
                        new ParallelAction(
                                slides.zero(),
                                lever.down(),
                                arm.floor()
                        ),
                        drive.actionBuilder(initialPose).waitSeconds(1.4).build(),
                        armGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        arm.up(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(1.9).build(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highBasket(),
                        drive.actionBuilder(initialPose).waitSeconds(1.75).build(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(-57, -57), Math.toRadians(245))
                                .build(),
                        drive.actionBuilder(initialPose).waitSeconds(0.4).build(),
                        slideGripper.open(),
                        // Sample 3 Scored
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        // Begin Lvl 1 Ascent
                        new ParallelAction(
                                new SequentialAction(
                                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                                        slides.zero(),
                                        hangingArm.up(),
                                        slideGripper.close()
                                ),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToSplineHeading(new Vector2d(-40,-20), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(-23,-5, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(30))
                                        .build()
                        ),
                        hangingArm.bar()
                )
        );

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.update();
        }
    }
}

