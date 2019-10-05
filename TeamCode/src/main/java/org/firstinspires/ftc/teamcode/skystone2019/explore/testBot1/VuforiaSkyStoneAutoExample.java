/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.skystone2019.explore.testBot1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.localvuforia.VuforiaKey;

import java.util.ArrayList;
import java.util.List;

import ftc.evlib.opmodes.AbstractAutoOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.google.common.collect.ImmutableList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;


/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@TeleOp(name="SKYSTONE Vuforia Auto Nav", group ="Concept")
public class VuforiaSkyStoneAutoExample extends AbstractAutoOp<TB2019RobotCfg> {

    private boolean vuforiaInitialized = false;
    private TeamColor teamColor;
    ResultReceiver<VuforiaManager> vfManagerResultRecvr = new BasicResultReceiver<>();

    @Override
    protected TB2019RobotCfg createRobotCfg() {
        return new TB2019RobotCfg(hardwareMap);
    }

    @Override
    public void setup() {
        super.setup();
        initVuforiaOnNewThread(vfManagerResultRecvr);
    }

    private void initVuforiaOnNewThread(ResultReceiver<VuforiaManager> resultReceiver) {
        Runnable runnable = new Runnable() {

            @Override
            public void run() {
                VuforiaManager vfMgr = VuforiaManager.init(cp, hardwareMap)
            }
        };
    }

    @Override
    protected Logger createLogger() {


        return new Logger("", "auto.csv",
                robotCfg.getLoggerColumns()
        );
    }


    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }


    @Override
    protected void act() {
        telemetry.addData("gyro", robotCfg.getGyro().getHeading());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }


    @Override
    protected void end() {
//        mineralLogOutputter.close();
//        mineralLogOutputter = null;
    }

    @Override
    public StateMachine buildStates() {
        double panSpeed = 0.15;
        double panSpeedLeft = .1;


//        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(RoverRuckusOptionsOp.FILENAME));
//
//
//        teamColor = TeamColor.RED;
//        isStartingDepot=optionsFile.get(RoverRuckusOptionsOp.isStartingDepot,RoverRuckusOptionsOp.isStartingDepotDefault);
//        moveToOpponentCrater=optionsFile.get(RoverRuckusOptionsOp.moveToOpponentCrater,RoverRuckusOptionsOp.moveToOpponentCraterDefault);
//        double waitForAlliance = optionsFile.get(RoverRuckusOptionsOp.wait,RoverRuckusOptionsOp.waitDefault);
//        boolean defendCrater = !optionsFile.get(RoverRuckusOptionsOp.Opts.DO_CLAIM_CRATER_SIDE.s,RoverRuckusOptionsOp.claimFromCraterSideDefault);
//        boolean doPartnerSample = optionsFile.get(RoverRuckusOptionsOp.Opts.DO_PARTNER_SAMPLE.s,RoverRuckusOptionsOp.doPartnerSampleDefault);
//        boolean doDescend = optionsFile.get(RoverRuckusOptionsOp.Opts.DESCEND.s,RoverRuckusOptionsOp.descendDefault); //this needs to be in the options op


//        final ResultReceiver <Mineral> mineralResultReceiver = new BasicResultReceiver<>();
//        final ResultReceiver<Boolean> actResultReceiver=new BasicResultReceiver<>();


        // START CONDITION
        StateName firstState = S.BEGIN;
        Angle tolerance = Angle.fromDegrees(3);
        EVStateMachineBuilder b = robotCfg.createEVStateMachineBuilder(firstState, teamColor, tolerance);//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        b.addWait(S.BEGIN, S.DRIVE1, 2000L);
        b.addDrive(S.DRIVE1, S.STOP, Distance.fromFeet(2.0), .5, 0, 0);
        b.addStop(S.STOP);

        return b.build();
    }


    @Override
    public void runOpMode() {
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

    }


    private enum S implements StateName {
        BEGIN, DRIVE1, STOP;
    }

    State createEndState(final VuforiaManager vm) {
        return new State() {
            @Override
            public StateName act() {
                // Disable Tracking when we are done;
                vm.getTargetsSkyStone().deactivate();
                return null;
            }
        };
    }
}