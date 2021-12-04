package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;
public class barcodePipeline extends OpenCvPipeline {
    int pos = 0;
    @Override
    public Mat processFrame(Mat input){
        pos = 10;

        return input;
    }
    public int getOrder(){
        return pos;
    }
}
