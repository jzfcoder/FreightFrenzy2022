/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.TestDemos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeamMarkerDetector;
import org.openftc.easyopencv.OpenCvCamera;

@Disabled
@TeleOp(name="OpenCVTest", group="Comp")
public class EasyOpenCVTest extends LinearOpMode
{
    OpenCvCamera webcam;

    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {

        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        telemetry.addData(">","Press start to detect Team marker");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;

            teamMarkerPosition = teamMarkerDetector.getTeamMarkerPosition();

            switch(teamMarkerPosition) {
                case LEFT:
                    telemetry.addData("TeamMarkerPosition", "LEFT");
                    break;
                case MIDDLE:
                    telemetry.addData("TeamMarkerPosition", "MIDDLE");
                    break;
                case RIGHT:
                    telemetry.addData("TeamMarkerPosition", "RIGHT");
                    break;
                default:
                    case NOT_DETECTED:
                        telemetry.addData("TeamMarkerPosition", "NOT_DETECTED");
                        break;
            }
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        teamMarkerDetector.clean();
    }

}