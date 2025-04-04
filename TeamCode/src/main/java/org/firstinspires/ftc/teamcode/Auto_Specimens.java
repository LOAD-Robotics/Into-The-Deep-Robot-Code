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

import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Vector;


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
@Autonomous(name="Auto_Specimens", group="Linear OpMode", preselectTeleOp = "TeleOp_Main")
// ADD @Disabled to disable

public class Auto_Specimens extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(9,-61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Create the hardware instances
        Slides slides = new Slides(hardwareMap);
        TopGripper slideGripper = new TopGripper(hardwareMap);
        ArmGripper armGripper = new ArmGripper(hardwareMap);
        FrontArm arm = new FrontArm(hardwareMap);
        SampleLever lever = new SampleLever(hardwareMap);
        HangingArm hangingArm = new HangingArm(hardwareMap);




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Insert code to be run once when START is pressed
        runtime.reset();

        double spec1 = 46;
        double spec2 = 54;

        Actions.runBlocking(
                new SequentialAction(
                        // Insert auto code here
                        arm.up(),
                        armGripper.close(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                        slides.resetZero(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highChamber(),
                        drive.actionBuilder(initialPose)
                                .strafeToLinearHeading(new Vector2d(0,-33), Math.toRadians(90))
                                .build(),
                        slides.highChamberClip(),
                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                        slideGripper.open(),
                        // Specimen 1 scored
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        slides.zero(),
                        // Begin sample pushing
                        drive.actionBuilder(new Pose2d(0, -33, Math.toRadians(90)))
                                // Begin setup for pushing
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(37, -35, Math.toRadians(90)), Math.toRadians(90))
                                // Begin Sample 1 push
                                .splineToLinearHeading(new Pose2d(36, -13, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(39.5, -10, Math.toRadians(90)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(spec1+3, -13, Math.toRadians(90)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(spec1, -52, Math.toRadians(90)), Math.toRadians(-90))
                                // Sample 1 is pushed, begin Sample 2 push
                                .build(),
                        // Begin Specimen 2 scoring
                        arm.wall(),
                        lever.down(),
                        slides.zero(),
                        drive.actionBuilder(new Pose2d(spec1, -52, Math.toRadians(90)))
                                .strafeToSplineHeading(new Vector2d(spec1, -35), Math.toRadians(0))
                                .strafeToSplineHeading(new Vector2d(spec1, -55.2), Math.toRadians(-100))
                                .build(),
                        drive.actionBuilder(initialPose).waitSeconds(0.3).build(),
                        armGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        arm.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.1).build(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.7).build(),
                        slides.resetZero(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highChamber(),
                        drive.actionBuilder(new Pose2d(spec1, -55.2, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(2,-33.25, Math.toRadians(90)), Math.toRadians(90))
                                .build(),
                        slides.highChamberClip(),
                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                        slideGripper.open(),
                        // Specimen 2 Scored, begin Specimen 3 scoring
                        arm.wall(),
                        lever.down(),
                        slides.zero(),
                        drive.actionBuilder(new Pose2d(2, -33.25, Math.toRadians(90)))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(-90))
                                .strafeToSplineHeading(new Vector2d(spec2, -45), Math.toRadians(-90))
                                .strafeToSplineHeading(new Vector2d(spec2, -55.2), Math.toRadians(-90))
                                .build(),
                        drive.actionBuilder(initialPose).waitSeconds(0.3).build(),
                        armGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        arm.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.1).build(),
                        lever.up(),
                        drive.actionBuilder(initialPose).waitSeconds(0.7).build(),
                        slides.resetZero(),
                        slideGripper.close(),
                        drive.actionBuilder(initialPose).waitSeconds(0.2).build(),
                        armGripper.open(),
                        slides.highChamber(),
                        drive.actionBuilder(new Pose2d(spec2, -55.2, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(4,-33.25, Math.toRadians(90)), Math.toRadians(90))
                                .build(),
                        slides.highChamberClip(),
                        drive.actionBuilder(initialPose).waitSeconds(0.5).build(),
                        slideGripper.open()
                )
        );

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.update();
        }
    }
}

