// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.Utils;

// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.Comparator;
// import java.util.List;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.math.MathUtils;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;



// /**
//  * Add your docs here.
//  */
// public class LimeLight {

//     private static NetworkTableInstance tableInstance;
//     private static NetworkTable table;
//     private NetworkTableEntry targetOffsetAngle_Horizontal;
//     private NetworkTableEntry targetOffsetAngle_Vertical;
//     private NetworkTableEntry targetArea;
//     private NetworkTableEntry tv;
//     private NetworkTableEntry tcornx;
//     private NetworkTableEntry tcorny;
//     private double x;
//     private double y;
//     private double area;

//     private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
//     private boolean mSeesTarget = false;

//     private double horizontal = 54.0;
//     private double vertical = 41.0;

//     private double maxXp = 320;
//     private double maxYp = 240;


    
//     public LimeLight(){
//         tableInstance = NetworkTableInstance.getDefault();
//         table = tableInstance.getTable("limelight");
//         targetOffsetAngle_Horizontal = table.getEntry("tx");
//         targetOffsetAngle_Vertical = table.getEntry("ty");
//         targetArea = table.getEntry("ta");
//         tv = table.getEntry("tv");
//         tcornx = table.getEntry("tcornx");
//         tcorny = table.getEntry("tcorny");
//     }

//     public NetworkTable getTable()
//     {
//         return table;
//     }

//     public double getDistance(){
//         return getDistance(getY());
//     }

//     public double getDistance(double angle){
//         double a = Math.toRadians(angle + Constants.limelightAngle);
//         return Constants.deltaBasketHeight / Math.tan(a);
//     }

//     public double getX(){
//         return  x = targetOffsetAngle_Horizontal.getDouble(0.0);
//     }

//     public double getY(){
//         return y = targetOffsetAngle_Vertical.getDouble(0.0);
//     }

//     public double getArea(){
//         return area = targetArea.getDouble(0.0);
//     }

//     public boolean getHasValidTarget(){
//         return tv.getBoolean(false);
//     }

//     private List<double[]> getTopCorners() {
//         double[] xCorners = tcornx.getDoubleArray(mZeroArray);
//         double[] yCorners = tcorny.getDoubleArray(mZeroArray);
//         mSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;

//         String s = "";
//         for (int i = 0; i < xCorners.length; i++){
//             s += xCorners[i] + ", ";
//         }

//         SmartDashboard.putString("xCorners", s);
//         SmartDashboard.putNumber("yCorners", yCorners[0]);

//         // something went wrong
//         if (!mSeesTarget ||
//                 Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
//                 // || xCorners.length != 8 || yCorners.length != 8) {
//                 ){
//                     Utils.print("x " + xCorners.length + " y " + yCorners.length);
//                     return null;
//         }

//         return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
//     }

//     private static final Comparator<Vector2D> xSort = Comparator.comparingDouble(Vector2D::getX);
//     private static final Comparator<Vector2D> ySort = Comparator.comparingDouble(Vector2D::getY);


//     public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
//         List<Vector2D> corners = new ArrayList<>();
//         for (int i = 0; i < xCorners.length; i++) {
//             corners.add(new Vector2D(xCorners[i], yCorners[i]));
//         }

//         corners.sort(xSort);

//         List<Vector2D> left = corners.subList(0, xCorners.length);
//         List<Vector2D> right = corners.subList(xCorners.length, xCorners.length * 2);

//         left.sort(ySort);
//         right.sort(ySort);

//         List<Vector2D> leftTop = left.subList(0, (int) (xCorners.length / 2));
//         List<Vector2D> rightTop = right.subList(0, (int) (xCorners.length / 2));

//         leftTop.sort(xSort);
//         rightTop.sort(xSort);

//         Vector2D leftCorner = leftTop.get(0);
//         Vector2D rightCorner = rightTop.get(1);

//         return List.of(new double[]{leftCorner.getX(), leftCorner.getY()}, new double[]{rightCorner.getX(), rightCorner.getY()});
//     }

//     public double getTargetSlope(Point[] points){
//         // double slope = getTopCorners().get(1)[1]-getTopCorners().get(0)[1]/getTopCorners().get(1)[0]-getTopCorners().get(0)[0];
//         Point[] cornerSidesTop = points;
//         double slope = (cornerSidesTop[1].ny - cornerSidesTop[0].ny) / (cornerSidesTop[1].nx - cornerSidesTop[0].nx);
//         if( slope > MathUtils.EPSILON) {
//             return slope;
//         }
//         return 0;
//     }

//     public double getCamAngleFromTarget(){
//         try{
//         double alpha, dStar, r, d, realAlpha;
//         d = getDistance();
//         r = 0.38;
//         dStar = 0;
//         Point[] topCorners = getCornerSideTop();
//         // Utils.print("" + topCorners);
//         if(topCorners == null){
//             Utils.print("not found top corners");
//             return Double.NaN;
//         }
//         double a;

//         if(getTargetSlope(topCorners) > 0){
//             // double a = Constants.limelightAngle + getTopCorners().get(1)[1];
//             a = topCorners[1].ny / (getY() / (vertical / 2.0) * (maxYp / 2.0)) * getY();
//             // ny = ay / (vertical / 2.0) * (maxYp / 2.0);
//             dStar = getDistance(a);
//         }
//         else{
//             // double a = Constants.limelightAngle + getTopCorners().get(0)[1];
//             // a = Math.toDegrees(convert(topCorners[0]).y);
//             a = topCorners[0].ny / (getY() / (vertical / 2.0) * (maxYp / 2.0)) * getY();
//             dStar = getDistance(a);
//         }
//         // Utils.print(topCorners[1].ny + " " + getY() / vertical * maxYp);
//         SmartDashboard.putNumber("dstar", dStar);
//         SmartDashboard.putNumber("con a", a);
//         alpha = Math.acos((Math.pow(d, 2) + Math.pow(r, 2) - Math.pow(dStar, 2)) / (2*d*r));
//         SmartDashboard.putNumber("cos", (Math.pow(d, 2) + Math.pow(r, 2) - Math.pow(dStar, 2)) / (2*d*r));
//         alpha = Math.toDegrees(alpha);
//         realAlpha = 90 - alpha;
//         return Math.toRadians(realAlpha);
//         } catch (Exception e){
//             Utils.print("lime failed");
//             return Double.NaN;
//         }
//     }

//     public double getPlanesDiff(){
//         double pDiff;
//         pDiff = getDistance() * Math.sin(getCamAngleFromTarget());
//         return pDiff;
//     }

//     public double getPlanesDistance(){
//         double pDist;
//         pDist = getDistance() * Math.cos(getCamAngleFromTarget());
//         return pDist;
//     }

//     public double getDistanceToInnerHole(){
//         return Math.hypot(getPlanesDiff() + Constants.outerToInnerDistance, getPlanesDistance());
//     }

//     public double getAngleToInnerHole(){
//         return Math.atan((getPlanesDiff() + Constants.outerToInnerDistance) / getPlanesDistance());
//     }

//     public void setCamMode(boolean isDriverMode){
//         if(isDriverMode){
//             table.getEntry("camMode").setNumber(0);
//             table.getEntry("ledMode").setNumber(3);
//         }
//         else{
//             table.getEntry("camMode").setNumber(0);
//             table.getEntry("ledMode").setNumber(3);
//         }
//     }

//     // focalLength = 2.9272781257541; //mm

//     public Point[] getCornerSideTop(){
//         // Utils.print("" + table.getEntry("tx0").getDouble(0.0));
//         SmartDashboard.putNumber("ty0", table.getEntry("ty0").getDouble(0) * maxYp  / 2.0);
//         SmartDashboard.putNumber("ny", getY() / 41.0 * 2.0 * maxYp);
//         // double[] xCorners = tcornx.getDoubleArray(mZeroArray);
//         Double[] xCorners = tcornx.getDoubleArray(new Double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//         // double[] yCorners = tcorny.getDoubleArray(mZeroArray);
//         Double[] yCorners = tcorny.getDoubleArray(new Double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//         mSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;

//         // Utils.print("" + xCorners.length + " " + yCorners.length);

//         if(!mSeesTarget){
//             return null;
//         }

//         List<Point> corners = new ArrayList<>();
//         for(int i = 0; i < Math.min(xCorners.length, yCorners.length); i++){
//             SmartDashboard.putNumber("y" + i, yCorners[i]);
//             corners.add(new Point(xCorners[i], yCorners[i]));
//         }

//         List<Point> topCorners = new ArrayList<>();
//         // Utils.print("" + Math.tan(Math.toRadians(getY())) / Math.tan(Math.toRadians(41.0/2.0)));
//         // Utils.print("" + corners.size());
//         for(int i = 0; i < corners.size(); i++){
//             if(Utils.inRange(corners.get(i).ay, getY(), 10)){
//                 // Utils.print("msg " + corners.get(i).ny);
//                 topCorners.add(corners.get(i));
//             }
//         }

//         int minIndex = 0;
//         int maxIndex = 0;
//         for(int i = 0; i < topCorners.size(); i++){
//             if(topCorners.get(i).nx > topCorners.get(maxIndex).nx){
//                 maxIndex = i;
//             }
//             if(topCorners.get(i).nx < topCorners.get(minIndex).nx){
//                 minIndex = i;
//             }

//         }

//         // Utils.print("max " + topCorners.get(maxIndex).ay + " min " + topCorners.get(minIndex).ay);

//         return new Point[]{topCorners.get(minIndex), topCorners.get(maxIndex)};
//     }

//     public double getTanAngle(){
//         try{
//             SmartDashboard.putNumber("y0", getCornerSideTop()[0].ay);
//             SmartDashboard.putNumber("y1", getCornerSideTop()[1].ay);
//             SmartDashboard.putNumber("yr", getY());
//             return Math.toDegrees(Math.atan2((getCornerSideTop()[1].ay - getCornerSideTop()[0].ay), (getCornerSideTop()[1].ax - getCornerSideTop()[0].ax)));
//         } catch (Exception e) {
//             //TODO: handle exception
//             // e.printStackTrace();
//         }
//         return 0;
//     }

//     private class Point{
//         public final double nx;
//         public final double ny;
        
//         public final double ax;
//         public final double ay;

//         public Point(double px, double py){
//             this.nx = px - 160;
//             this.ny = 120 - py;
            
//             this.ax = horizontal / 2.0 * nx / (maxXp / 2.0);
//             this.ay = vertical / 2.0 * ny / (maxYp / 2.0);
//         }
//     }
// }
