package org.firstinspires.ftc.teamcode.RoverRuckus;

import ftc.evlib.util.ImmutableList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ftc.electronvolts.util.ResultReceiver;

public class GoldDetector {

    public enum Detection{
        GOLD,
        SILVER,
        NOTHING
    }

    private static final float MIN_CONFIDENCE = .35f;
    private final static int maxWidth = 225;
    private final static int minWidth = 80;
    private final static int maxHeight = 320;
    private final static int minHeight = 90;
    private final static int x1=400; //320;
    private final static int x2=840; // 960;
    private final static int y1=-40;
    private final static int y2= 380; // 360;

    private final static Rectangle centerFilter = new Rectangle(x1, x2, y1, y2);

    public static Mineral findPosition(List<Recognition> fullList, Telemetry telemetry, ResultReceiver<List<Mineral>> potentialItemRR) {
        List<Mineral> list = filter(fullList, telemetry, potentialItemRR);

        if(list.size()>0){
            Mineral m1=list.get(0);
            if(m1.isGold()){
                m1.setType(Detection.GOLD);
            }
            else if(!m1.isGold()){
                m1.setType(Detection.SILVER);
            }
            return m1;
        }
        Mineral m1=new Mineral();

        return m1;
    }




    private static List<Mineral> filter(List<Recognition> fullList, Telemetry telemetry, ResultReceiver<List<Mineral>> potentialItemRR) {
        List<Mineral> filteredMinerals = new ArrayList<>();
        List<Mineral> allMinerals = new ArrayList<>();
        int c=0;
        for (Recognition r : fullList) {
            int x = 1280 - (int) r.getBottom();
            int y = 720 - (int) r.getRight();
            boolean insideFilterBox = centerFilter.isInside(x, y) ;
            int h = (int) r.getWidth(); // camera is sideways
            int w = (int) r.getHeight();
            //In our coordinate system, width  is height and height is width
            double centerOfFilterX=(x2-x1)/2.;
            double centerOfBlockX=(x+.5*(w));
            double centerOfFilterY=(y2-y1)/2.;
            double centerOfBlockY=(y+.5*(h));

            double sideX=centerOfFilterX-centerOfBlockX;
            double sideY=centerOfFilterY-centerOfBlockY;
            double radius= Math.sqrt(sideX*sideX+sideY*sideY);
            boolean isGold = r.getLabel().equals(ObjectDetector.LABEL_GOLD_MINERAL);

            boolean heightOk = (h < maxHeight) && (h > minHeight);
            boolean widthOK = (w < maxWidth) && (w > minWidth);

            Mineral m = new Mineral(x, y, w, h, radius, isGold, r.getConfidence(),c++,insideFilterBox,heightOk,widthOK);
            if (telemetry != null) {
                m.showInTelem(telemetry);
            }

            allMinerals.add(m);
            boolean isIn = false;
//            if (insideFilterBox) { // && heightOk && widthOK &&(r.getConfidence()>MIN_CONFIDENCE)) {
            if((centerOfBlockX > 320) && (centerOfBlockX < 960)) {
                filteredMinerals.add(m);
                isIn = true;
            }
//            log(m, isIn);

        }
        Collections.sort(filteredMinerals);
        Collections.sort(allMinerals);
        potentialItemRR.setValue(allMinerals);
        return new ArrayList<>(filteredMinerals); // ImmutableList.copyOf(filteredMinerals);
    }

    // Temp method to write mineral data to a file as it is discoverede
//    private static synchronized void log(Mineral m, boolean isIn) {
//        PrintStream pw = RoverRuckusAuto3.getMineralLogWriter();
//        long millis = System.currentTimeMillis();
//        pw.printf("%d,%s,%d\n",millis,m.toStringExtended(),isIn?1:0);
//    }
}

