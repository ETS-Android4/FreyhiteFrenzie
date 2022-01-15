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

import java.util.ArrayList;
import java.util.List;

public class barcodePipeline extends OpenCvPipeline {
    int pos = 0;
    Rect crosshair = new Rect(new Point(150, 150), new Point(200,200));
    int c1 = 0;
    int c2 = 0;
    int c3 = 0;

    @Override
    public Mat processFrame(Mat input){
        pos = 10;
        Size imageSize = input.size();
        Mat output = input.clone();
        Imgproc.rectangle(output, crosshair, new Scalar(4, 233, 78), 3, 8);
        Rect crosshair1 = new Rect(new Point(0, imageSize.height/2), new Point(imageSize.width, imageSize.height - 50));
        Imgproc.rectangle(output, crosshair1, new Scalar(4, 233, 78), 3, 8);

        List<MatOfPoint> matList = new ArrayList<>();

        //Imgproc.findContours(output, matList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(input, matList, 1, new Scalar(54, 192, 243), 3, 8);

        for(int i = (int)(imageSize.height/2); i < imageSize.height-50 ; i++){
            for(int j = 0; j < imageSize.width; j++)
            {
                double[] pixelColor = input.get(i, j);
                if (pixelColor[0] > 160 && pixelColor[1] < 100 && pixelColor[2] < 50){
                    if (j <= imageSize.width/3){
                        c1++;
                    }
                    else if (j <= ((imageSize.width/3) * 2)){
                        c2++;
                    }
                    else{
                        c3++;
                    }

                    if(pixelColor[0] > 100)
                    {
                        double[] newColor = {0, 0, 0, pixelColor[3]};
                        output.put(i, j, newColor);
                    }
                }
            }
        }

        if(c1 >= c2 && c1 >= c3)
        {
            robot.barcode = 1;
        }
        else if(c2 >= c1 && c2 >= c3)
        {
            robot.barcode = 2;
        }
        else if(c3 >= c1 && c3 >= c2)
        {
            robot.barcode = 3;
        }

        return output;
    }
    public int getOrder(){
        return robot.barcode;
    }
}
