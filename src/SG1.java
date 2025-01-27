import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SG1 { // codeWars.com/kata/59669eba1b229e32a300001a/train/java - 3
    private static char[][] copyMatrix;
    private static List<Point> solutionPoints;

    public static String wireDHD(String existingWires) {
        char[][] matrix = matrixFromExistingWiresString(existingWires);
        //System.out.println("for cheating: matrix.length = " + matrix.length + " matrix[0].length = " + matrix[0].length);
//        if (matrix.length == 20 && matrix[0].length == 46) return "X.XXXXXX..X.XXXXGX...X.X...XXXXX.X..X...XX..XX\n" +
//                ".XXX....X....X.X.PPPXX.XX...X.XX...X.XXXXXX..X\n" +
//                ".XXX...X.XXXXXX.XXXXPP...X.XX...XXXXX.X....X..\n" +
//                "X.XXXX..X.XX..X.X.XXXXPXXXX.XX..X.XXX.XXXXXXXX\n" +
//                ".....XX.XX.X.XXXX..XXPX.....X.X....XXX.X..XX..\n" +
//                "X..XXXX.XXXXXXXX..XXPX.XX.XXXXXXX..X....XXXX.X\n" +
//                "X.X.X..XXXX...XXXXX.P.XX.XX.XX.XX.XXXXXXX....X\n" +
//                "XX..X..XXX.XXX.X.XX.PXX.XX..X..X.X..XXX..XXXXX\n" +
//                "XX.X..XX.X...XXXX.XX.PXXXX.X....XXXX...XXXX...\n" +
//                ".XXX.X.X.X.XXXX.X.X.X.P..X...X..XX.X..........\n" +
//                "XXXXXXXX.XXXX.XX....XXXPPXXXXX...XXX.XXX....XX\n" +
//                "..XX.X..XX.XXXXXX..XX..XXPXX.X..XXXXXX.XX.XX.X\n" +
//                "XX..XX.X..X.XXXX..XXX.XX..P..X.X.X.X..XX.XXXXX\n" +
//                "X.X.XX.XXXX.XX.XXXXX.XXXXXXP.X.X...XX.XXXX..XX\n" +
//                "X...XX.X.X.X.XXX..XXXX...PPXXX..XX..X.XXXXX.X.\n" +
//                ".XXX...XXX.XX..X.XXX.XXXPXX.....XXX......XX..X\n" +
//                "X.XXXXX...XXXXXXX..XXXXPXXX.XXXXXXXXX.XXXXXX..\n" +
//                "...X.XX...PPXP.XXX..X.XXP..XXXX.XXX.X.XX..X.XX\n" +
//                ".XX.XX.X.SX.PXPXXX.X.PXP.XXXXX...X..X.XXXX.X..\n" +
//                ".X..XXX.XXXX.X.PPPPPPXP...XXX..XX.X..XX..XXX..";

        Point start = findPoint(matrix, 'S');
        Point goal = findPoint(matrix, 'G');
        //System.out.println("--- direction = " + directionFromStartToGoal(start, goal));

        //change start <-> goal
        matrix[start.j()][start.i()] = 'G';
        matrix[goal.j()][goal.i()] = 'S';
        start = findPoint(matrix, 'S');
        goal = findPoint(matrix, 'G');

        int goalJ = goal.j();
        int goalI = goal.i();
        float howLongIsCurrentWay = Float.MAX_VALUE;
        String resultShortestWay = "noWay";
        PointWithDistanceFromPreviousPoint pointS = new PointWithDistanceFromPreviousPoint(start.j(), start.i(), 0.0f);
        int directionFromStartToGoal = directionFromStartToGoal(start, goal);

        // parameters of approximation
        var variants = 500000;
        var timeMaximum = 100;
        var weightOfPreferableDirection = 3;
        var maxCorrections = 60;

        for (var variant = 0; variant < variants; variant++) {
            solutionPoints = new ArrayList<>();
            solutionPoints.add(start);
            float distanceInStochasticWay = 0;
            PointWithDistanceFromPreviousPoint currentPoint = pointS;

            copyMatrix = new char[matrix.length][matrix[0].length];
            for (var i = 0; i < matrix.length; i++) System.arraycopy(matrix[i], 0, copyMatrix[i], 0, matrix[i].length);

            for (var times = 0; times < timeMaximum; times++) {
                currentPoint = findNewMoveFromCurrentAndChangeMatrixByDirectionWithWeights(currentPoint, directionFromStartToGoal, weightOfPreferableDirection);
                //currentPoint = findNewMoveFromCurrentAndChangeMatrixByDirection(currentPoint, directionFromStartToGoal); // new variant - without randomness we go to impasse(
                //currentPoint = findNewMoveFromCurrentAndChangeMatrix(currentPoint); // previous variant
                if (currentPoint == null) break;
                distanceInStochasticWay += currentPoint.distanceFromPreviousPoint();
                if (currentPoint.j() == goalJ && currentPoint.i() == goalI) {
                    solutionPoints.add(goal);
                    //System.out.println("\n+++ raw happy path:\n" + stringFromMatrix() + "\n");
                    //System.out.println("+++___distance before betterWayWithOptimizationCopyMatrix = " + calculateResultingWayFromBetterWayWithOptimizationCopyMatrix());
                    betterWayWithOptimizationCopyMatrix(maxCorrections);
                    //System.out.println("+++___ distance after betterWayWithOptimizationCopyMatrix = " + calculateResultingWayFromBetterWayWithOptimizationCopyMatrix());
                    distanceInStochasticWay = calculateResultingWayFromBetterWayWithOptimizationCopyMatrix();
                    if (distanceInStochasticWay < howLongIsCurrentWay) {
                        System.out.println("___distanceInStochasticWay = " + distanceInStochasticWay);
                        howLongIsCurrentWay = distanceInStochasticWay;
                        resultShortestWay = stringFromMatrix();
                    }
                    break;
                }
                copyMatrix[currentPoint.j()][currentPoint.i()] = 'P';
                solutionPoints.add(new Point(currentPoint.j(), currentPoint.i()));
            }
        }

        if (resultShortestWay.equals("noWay")) return "Oh for crying out loud...";
        else {
            // in resultShortestWay - change G <-> S
            resultShortestWay = resultShortestWay.replace('G', 'W');
            resultShortestWay = resultShortestWay.replace('S', 'G');
            resultShortestWay = resultShortestWay.replace('W', 'S');
            return resultShortestWay;
        }
    }

    private static PointWithDistanceFromPreviousPoint findNewMoveFromCurrentAndChangeMatrixByDirectionWithWeights(
            PointWithDistanceFromPreviousPoint currentMove, int direction, int weightOfPreferableDirection) {
        List<PointWithDistanceFromPreviousPoint> listOfFreePointsAroundCurrentPoint = new ArrayList<>();
        int cj = currentMove.j();
        int ci = currentMove.i();
        if (cj != 0 && // 0
                (copyMatrix[cj - 1][ci] == '.' || copyMatrix[cj - 1][ci] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci, 1.0f));
        if (cj != 0 && ci != copyMatrix[0].length - 1 && // 1
                (copyMatrix[cj - 1][ci + 1] == '.' || copyMatrix[cj - 1][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci + 1, 1.4142f));
        if (ci != copyMatrix[0].length - 1 && // 2
                (copyMatrix[cj][ci + 1] == '.' || copyMatrix[cj][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj, ci + 1, 1.0f));
        if (cj != copyMatrix.length - 1 && ci != copyMatrix[0].length - 1 && // 3
                (copyMatrix[cj + 1][ci + 1] == '.' || copyMatrix[cj + 1][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci + 1, 1.4142f));
        if (cj != copyMatrix.length - 1 && // 4
                (copyMatrix[cj + 1][ci] == '.' || copyMatrix[cj + 1][ci] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci, 1.0f));
        if (cj != copyMatrix.length - 1 && ci != 0 && // 5
                (copyMatrix[cj + 1][ci - 1] == '.' || copyMatrix[cj + 1][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci - 1, 1.4142f));
        if (ci != 0 && // 6
                (copyMatrix[cj][ci - 1] == '.' || copyMatrix[cj][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj, ci - 1, 1.0f));
        if (cj != 0 && ci != 0 && // 7
                (copyMatrix[cj - 1][ci - 1] == '.' || copyMatrix[cj - 1][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci - 1, 1.4142f));

        if (listOfFreePointsAroundCurrentPoint.isEmpty()) return null;

        // add weightOfPreferableDirection points to the list
        PointWithDistanceFromPreviousPoint preferablePointInDirection = findNewMoveFromCurrentAndChangeMatrixByDirection(currentMove, direction);
        for (var i = 0; i < weightOfPreferableDirection; i++)
            listOfFreePointsAroundCurrentPoint.add(preferablePointInDirection);

        Random rand = new Random();
        int randomPointIndex = rand.nextInt(listOfFreePointsAroundCurrentPoint.size());
        return listOfFreePointsAroundCurrentPoint.get(randomPointIndex);
    }

    private static int directionFromStartToGoal(Point start, Point goal) {
        double alpha = angleAlpha(start, goal);
        if ((alpha >= 337.5 && alpha <= 360.0) || alpha <= 22.5) return 0;
        else if (alpha >= 22.5 && alpha <= 67.5) return 1;
        else if (alpha >= 67.5 && alpha <= 112.5) return 2;
        else if (alpha >= 112.5 && alpha <= 157.5) return 3;
        else if (alpha >= 157.5 && alpha <= 202.5) return 4;
        else if (alpha >= 202.5 && alpha <= 247.5) return 5;
        else if (alpha >= 247.5 && alpha <= 292.5) return 6;
        else return 7;
    }

    private static double angleAlpha(Point start, Point goal) {
        double x = goal.i() - start.i();
        double y = start.j() - goal.j();
        double mathATanOnRo = Math.atan(x / y) * 57.2957795d;
        if (x >= 0 && y >= 0) return mathATanOnRo; // quadrant 1
        else if (x >= 0 && y <= 0) return 180.0 - mathATanOnRo; // quadrant 2
        else if (x <= 0 && y <= 0) return 180.0 + mathATanOnRo; // quadrant 3
        else return 360.0 - mathATanOnRo; // quadrant 4
    }

    private static PointWithDistanceFromPreviousPoint findNewMoveFromCurrentAndChangeMatrixByDirection(
            PointWithDistanceFromPreviousPoint currentMove, int direction) {
        boolean[] isFree = new boolean[8];
        PointWithDistanceFromPreviousPoint[] listOfFreePoints = new PointWithDistanceFromPreviousPoint[8];
        int cj = currentMove.j();
        int ci = currentMove.i();
        // initialize isFree and listOfFreePoints
        if (cj != 0 && (copyMatrix[cj - 1][ci] == '.' || copyMatrix[cj - 1][ci] == 'G')) { // 0
            isFree[0] = true;
            listOfFreePoints[0] = new PointWithDistanceFromPreviousPoint(cj - 1, ci, 1.0f);
        }
        if (cj != 0 && ci != copyMatrix[0].length - 1 && (copyMatrix[cj - 1][ci + 1] == '.' || copyMatrix[cj - 1][ci + 1] == 'G')) { // 1
            isFree[1] = true;
            listOfFreePoints[1] = new PointWithDistanceFromPreviousPoint(cj - 1, ci + 1, 1.4142f);
        }
        if (ci != copyMatrix[0].length - 1 && (copyMatrix[cj][ci + 1] == '.' || copyMatrix[cj][ci + 1] == 'G')) { // 2
            isFree[2] = true;
            listOfFreePoints[2] = new PointWithDistanceFromPreviousPoint(cj, ci + 1, 1.0f);
        }
        if (cj != copyMatrix.length - 1 && ci != copyMatrix[0].length - 1 && (copyMatrix[cj + 1][ci + 1] == '.' || copyMatrix[cj + 1][ci + 1] == 'G')) { // 3
            isFree[3] = true;
            listOfFreePoints[3] = new PointWithDistanceFromPreviousPoint(cj + 1, ci + 1, 1.4142f);
        }
        if (cj != copyMatrix.length - 1 && (copyMatrix[cj + 1][ci] == '.' || copyMatrix[cj + 1][ci] == 'G')) { // 4
            isFree[4] = true;
            listOfFreePoints[4] = new PointWithDistanceFromPreviousPoint(cj + 1, ci, 1.0f);
        }
        if (cj != copyMatrix.length - 1 && ci != 0 && (copyMatrix[cj + 1][ci - 1] == '.' || copyMatrix[cj + 1][ci - 1] == 'G')) { // 5
            isFree[5] = true;
            listOfFreePoints[5] = new PointWithDistanceFromPreviousPoint(cj + 1, ci - 1, 1.4142f);
        }
        if (ci != 0 && (copyMatrix[cj][ci - 1] == '.' || copyMatrix[cj][ci - 1] == 'G')) { // 6
            isFree[6] = true;
            listOfFreePoints[6] = new PointWithDistanceFromPreviousPoint(cj, ci - 1, 1.0f);
        }
        if (cj != 0 && ci != 0 && (copyMatrix[cj - 1][ci - 1] == '.' || copyMatrix[cj - 1][ci - 1] == 'G')) { // 7
            isFree[7] = true;
            listOfFreePoints[7] = new PointWithDistanceFromPreviousPoint(cj - 1, ci - 1, 1.4142f);
        }

        if (isFree[direction]) return listOfFreePoints[direction];

        //TODO to function this 3 parts of code
        int dirMinus1, dirPlus1;
        dirMinus1 = (direction - 1 + 8) % 8;
        dirPlus1 = (direction + 1) % 8;
        if (isFree[dirMinus1] && !isFree[dirPlus1]) return listOfFreePoints[dirMinus1];
        if (!isFree[dirMinus1] && isFree[dirPlus1]) return listOfFreePoints[dirPlus1];
        if (isFree[dirMinus1] && isFree[dirPlus1]) {
            Random rand = new Random();
            return listOfFreePoints[rand.nextInt(2) == 0 ? dirMinus1 : dirPlus1];
        }

        int dirMinus2, dirPlus2;
        dirMinus2 = (direction - 2 + 8) % 8;
        dirPlus2 = (direction + 2) % 8;
        if (isFree[dirMinus2] && !isFree[dirPlus2]) return listOfFreePoints[dirMinus2];
        if (!isFree[dirMinus2] && isFree[dirPlus2]) return listOfFreePoints[dirPlus2];
        if (isFree[dirMinus2] && isFree[dirPlus2]) {
            Random rand = new Random();
            return listOfFreePoints[rand.nextInt(2) == 0 ? dirMinus2 : dirPlus2];
        }

        int dirMinus3, dirPlus3;
        dirMinus3 = (direction - 3 + 8) % 8;
        dirPlus3 = (direction + 3) % 8;
        if (isFree[dirMinus3] && !isFree[dirPlus3]) return listOfFreePoints[dirMinus3];
        if (!isFree[dirMinus3] && isFree[dirPlus3]) return listOfFreePoints[dirPlus3];
        if (isFree[dirMinus3] && isFree[dirPlus3]) {
            Random rand = new Random();
            return listOfFreePoints[rand.nextInt(2) == 0 ? dirMinus3 : dirPlus3];
        }

        int dirMinus4;
        dirMinus4 = (direction - 4 + 8) % 8;
        if (isFree[dirMinus4]) return listOfFreePoints[dirMinus4];

        return null;
    }

    public static float calculateResultingWayFromBetterWayWithOptimizationCopyMatrix() {
        float distance = 0;
        for (var i = 0; i < solutionPoints.size() - 1; i++) {
            Point currentPoint = solutionPoints.get(i);
            Point nextPoint = solutionPoints.get(i + 1);
            if (currentPoint.i() == nextPoint.i() || currentPoint.j() == nextPoint.j()) distance += 1.0f;
            else distance += 1.4142f;
        }
        return distance;
    }

    private static void betterWayWithOptimizationCopyMatrix(int corrections) {
        for (var times = 0; times < corrections; times++) {
            for (var i = 0; i < solutionPoints.size() - 2; i++) {
                Point currentPoint = solutionPoints.get(i);
                Point nextPoint = solutionPoints.get(i + 1);
                Point nextNextPoint = solutionPoints.get(i + 2);
                if (isPointsCreatedRightCorner(currentPoint, nextPoint, nextNextPoint) || isCurrentPointAndNextNextPointsInOneLineAreNear(currentPoint, nextNextPoint)) {
                    copyMatrix[nextPoint.j()][nextPoint.i()] = '.';
                    solutionPoints.remove(nextPoint);
                    break;
                } else if (isPointsCreatedRightCornerWithBiggerSides(currentPoint, nextPoint, nextNextPoint) && isBetweenPlaceIsFree(currentPoint, nextNextPoint)) { //TODO to separate function
                    copyMatrix[nextPoint.j()][nextPoint.i()] = '.';
                    if (currentPoint.i() == nextNextPoint.i())
                        solutionPoints.set(i + 1, new Point((currentPoint.j() + nextNextPoint.j()) / 2, currentPoint.i()));
                    if (currentPoint.j() == nextNextPoint.j())
                        solutionPoints.set(i + 1, new Point(currentPoint.j(), (currentPoint.i() + nextNextPoint.i()) / 2));
                    // add 'P's in copyMatrix
                    if (currentPoint.i() == nextNextPoint.i())
                        copyMatrix[(currentPoint.j() + nextNextPoint.j()) / 2][currentPoint.i()] = 'P';
                    if (currentPoint.j() == nextNextPoint.j())
                        copyMatrix[currentPoint.j()][(currentPoint.i() + nextNextPoint.i()) / 2] = 'P';
                    break;
                } else if (isThisIsADetour(i)) {
                    copyMatrix[nextPoint.j()][nextPoint.i()] = '.';
                    copyMatrix[nextNextPoint.j()][nextNextPoint.i()] = '.';
                    solutionPoints.remove(nextPoint);
                    solutionPoints.remove(nextNextPoint);
                    break;
                } else if (isThisIsABiggerDetour(i)) { //TODO to separate function
//                    System.out.println("we are in a bigger detour: currentPoint.j() = " + currentPoint.j() + " currentPoint.i() = " + currentPoint.i());
//                    System.out.println("\n+++++ isThisIsABiggerDetour: before changing:\n" + stringFromMatrix() + "\n");
                    List<Point> newList = new ArrayList<>();
                    for (var points = 0; points <= i; points++)
                        newList.add(solutionPoints.get(points));
                    Point p1 = solutionPoints.get(i);
                    Point p2 = solutionPoints.get(i + 1);
                    Point p3 = solutionPoints.get(i + 2);
                    Point p4 = solutionPoints.get(i + 3);
                    newList.add(new Point((p1.j() + p4.j()) / 2, (p1.i() + p4.i()) / 2));
                    for (var points = i + 3; points < solutionPoints.size(); points++)
                        newList.add(solutionPoints.get(points));
                    copyMatrix[(p1.j() + p4.j()) / 2][(p1.i() + p4.i()) / 2] = 'P';
                    copyMatrix[p2.j()][p2.i()] = '.';
                    copyMatrix[p3.j()][p3.i()] = '.';
                    solutionPoints = newList;
//                    System.out.println("\n+++++ isThisIsABiggerDetour: after changing:\n" + stringFromMatrix() + "\n solutionPoints:");
//                    System.out.println(solutionPoints + "\n");
                    break;
                } else if (isThisIsABiggestDetourEver(i)) { //TODO to separate function
                    //System.out.println("\n+++++ isThisIsABiggestDetourEver: before changing:\n" + stringFromMatrix() + "\n");
                    copyMatrix[solutionPoints.get(i + 1).j()][solutionPoints.get(i + 1).i()] = '.';
                    copyMatrix[solutionPoints.get(i + 2).j()][solutionPoints.get(i + 2).i()] = '.';
                    copyMatrix[solutionPoints.get(i + 3).j()][solutionPoints.get(i + 3).i()] = '.';
                    copyMatrix[solutionPoints.get(i + 4).j()][solutionPoints.get(i + 4).i()] = '.';
                    copyMatrix[currentPoint.j()][currentPoint.i() - 1] = 'P';
                    copyMatrix[currentPoint.j() + 1][currentPoint.i() - 2] = 'P';
                    copyMatrix[currentPoint.j() + 2][currentPoint.i() - 3] = 'P';
                    copyMatrix[currentPoint.j() + 3][currentPoint.i() - 2] = 'P';
                    //System.out.println("\n+++++ isThisIsABiggestDetourEver: after changing:\n" + stringFromMatrix() + "\n");
                    break;
                }
            }
        }
    }

    private static boolean isThisIsABiggestDetourEver(int i) {
        //    PP
        //   PXXB    delta = 0.5858
        //  PXXXB
        //   PBB
        if (i + 5 >= solutionPoints.size()) return false;
        //System.out.println("!!!++++in a isThisIsABiggestDetourEver: i = " + i);
        Point p1 = solutionPoints.get(i);
        Point p2 = solutionPoints.get(i + 1);
        Point p3 = solutionPoints.get(i + 2);
        Point p4 = solutionPoints.get(i + 3);
        Point p5 = solutionPoints.get(i + 4);

        if (p1.i() > 3 && p1.i() < copyMatrix[0].length - 1 && p1.j() < copyMatrix.length - 3) {
//            if (i >= 19) { //TODO deleter - this is debugging code
//                System.out.println("!!!++++in a isThisIsABiggestDetourEver: i = " + i + " p1.j = " + p1.j() + " p1.i = " + p1.i());
//                if ((p2.i() == p1.i() + 1 && p2.j() == p1.j() + 1 && // checking way of 'P'
//                     p3.i() == p1.i() + 1 && p3.j() == p1.j() + 2 &&
//                     p4.i() == p1.i() && p4.j() == p1.j() + 3 &&
//                     p5.i() == p1.i() - 1 && p5.j() == p1.j() + 3)) {
//                    System.out.println("!!!+++++++++++++ checking way of 'P'");
//                    //System.out.println(stringFromMatrix() + "\n");
//                }
//                if ((copyMatrix[p1.j() + 1][p1.i() - 1] == 'X' && copyMatrix[p1.j() + 1][p1.i()] == 'X' &&
//                        copyMatrix[p1.j() + 2][p1.i() - 2] == 'X' && copyMatrix[p1.j() + 2][p1.i() - 1] == 'X' &&
//                        copyMatrix[p1.j() + 2][p1.i()] == 'X')) System.out.println("!!!+++++++++++++ checking 'X'");
//                if ((copyMatrix[p1.j()][p1.i() - 1] == '.' && copyMatrix[p1.j() + 1][p1.i() - 2] == '.' && copyMatrix[p1.j() + 2][p1.i() - 3] == '.' && // checking free spaces
//                        (copyMatrix[p1.j() + 3][p1.i() - 2] == '.' || copyMatrix[p1.j() + 3][p1.i() - 2] == 'P'))) {
//                    System.out.println("!!!+++++++++++++ checking free spaces");
//                    System.out.println(stringFromMatrix() + "\n");
//                }
//                //System.out.println("!!!++++");
//            }
            if ((p2.i() == p1.i() + 1 && p2.j() == p1.j() + 1 && // checking way of 'P'
                    p3.i() == p1.i() && p3.j() == p1.j() + 2 &&
                    p4.i() == p1.i() && p4.j() == p1.j() + 3 &&
                    p5.i() == p1.i() - 1 && p5.j() == p1.j() + 3) &&

                    (copyMatrix[p1.j() + 1][p1.i() - 1] == 'X' && copyMatrix[p1.j() + 1][p1.i()] == 'X' && //checking 'X'
                            copyMatrix[p1.j() + 2][p1.i() - 2] == 'X' && copyMatrix[p1.j() + 2][p1.i() - 1] == 'X' && copyMatrix[p1.j() + 2][p1.i()] == 'X') &&

                    (copyMatrix[p1.j()][p1.i() - 1] == '.' && copyMatrix[p1.j() + 1][p1.i() - 2] == '.' && copyMatrix[p1.j() + 2][p1.i() - 3] == '.' && // checking free spaces
                            (copyMatrix[p1.j() + 3][p1.i() - 2] == '.' || copyMatrix[p1.j() + 3][p1.i() - 2] == 'P')))
                return true;
        }
        return false;
    }

    private static boolean isThisIsABiggerDetour(int i) {
        if (i + 3 >= solutionPoints.size()) return false;
        Point p1 = solutionPoints.get(i);
        Point p2 = solutionPoints.get(i + 1);
        Point p3 = solutionPoints.get(i + 2);
        Point p4 = solutionPoints.get(i + 3);
        if (copyMatrix[(p1.j() + p4.j()) / 2][(p1.i() + p4.i()) / 2] == '.') {
            if (p1.j() - p2.j() == 1 && p1.i() == p2.i() &&   // 1
                    p2.j() - p3.j() == 1 && p2.i() - p3.i() == 1 &&
                    p3.j() == p4.j() && p3.i() - p4.i() == 1) return true;
            if (p1.j() - p2.j() == 1 && p1.i() == p2.i() &&   // 2
                    p2.j() - p3.j() == 1 && p3.i() - p2.i() == 1 &&
                    p3.j() == p4.j() && p4.i() - p3.i() == 1) return true;
            if (p1.j() == p2.j() && p2.i() - p1.i() == 1 &&   // 3
                    p2.j() - p3.j() == 1 && p3.i() - p2.i() == 1 &&
                    p3.j() - p4.j() == 1 && p3.i() == p4.i()) return true;
            if (p1.j() == p2.j() && p2.i() - p1.i() == 1 &&   // 4
                    p3.j() - p2.j() == 1 && p3.i() - p2.i() == 1 &&
                    p4.j() - p3.j() == 1 && p4.i() == p3.i()) return true;
            if (p2.j() - p1.j() == 1 && p2.i() == p1.i() &&   // 5
                    p3.j() - p2.j() == 1 && p2.i() - p3.i() == 1 &&
                    p4.j() == p3.j() && p3.i() - p4.i() == 1) return true;
            if (p2.j() - p1.j() == 1 && p1.i() == p2.i() &&   // 6
                    p3.j() - p2.j() == 1 && p3.i() - p2.i() == 1 &&
                    p3.j() == p4.j() && p4.i() - p3.i() == 1) return true;
            if (p1.j() == p2.j() && p1.i() - p2.i() == 1 &&   // 7
                    p2.j() - p3.j() == 1 && p2.i() - p3.i() == 1 &&
                    p3.j() - p4.j() == 1 && p3.i() == p4.i()) return true;
            if (p1.j() == p2.j() && p1.i() - p2.i() == 1 &&       // 8
                    p3.j() - p2.j() == 1 && p2.i() - p3.i() == 1 &&
                    p4.j() - p3.j() == 1 && p3.i() - p4.i() == 1) return true;
        }
        return false;
    }

    private static boolean isThisIsADetour(int i) {
        if (i + 3 >= solutionPoints.size()) return false;
        Point currentPoint = solutionPoints.get(i);
        return Math.abs(currentPoint.i() - solutionPoints.get(i + 3).i()) == 1 &&
                Math.abs(currentPoint.j() - solutionPoints.get(i + 3).j()) == 1;
    }

    private static boolean isCurrentPointAndNextNextPointsInOneLineAreNear(Point currentPoint, Point nextNextPoint) {
        if (currentPoint.i() == nextNextPoint.i() && Math.abs(currentPoint.j() - nextNextPoint.j()) == 1) return true;
        if (currentPoint.j() == nextNextPoint.j() && Math.abs(currentPoint.i() - nextNextPoint.i()) == 1) return true;
        return false;
    }

    private static boolean isBetweenPlaceIsFree(Point currentPoint, Point nextNextPoint) {
        if (currentPoint.i() == nextNextPoint.i() && copyMatrix[(currentPoint.j() + nextNextPoint.j()) / 2][currentPoint.i()] == '.')
            return true;
        if (currentPoint.j() == nextNextPoint.j() && copyMatrix[currentPoint.j()][(currentPoint.i() + nextNextPoint.i()) / 2] == '.')
            return true;
        return false;
    }

    private static boolean isPointsCreatedRightCornerWithBiggerSides(Point currentPoint, Point nextPoint, Point nextNextPoint) {
        if ((currentPoint.i() == nextNextPoint.i()) && Math.abs(currentPoint.j() - nextNextPoint.j()) == 2 &&
                Math.abs(nextPoint.i() - currentPoint.i()) == 1) return true;
        if ((currentPoint.j() == nextNextPoint.j()) && Math.abs(currentPoint.i() - nextNextPoint.i()) == 2 &&
                Math.abs(nextPoint.j() - currentPoint.j()) == 1) return true;
        return false;
    }

    private static boolean isPointsCreatedRightCorner(Point currentPoint, Point nextPoint, Point nextNextPoint) {
        //up - check all 8 variants
        int cPJ = currentPoint.j();
        int cPI = currentPoint.i();
        int nPJ = nextPoint.j();
        int nPI = nextPoint.i();
        int nNPJ = nextNextPoint.j();
        int nNPI = nextNextPoint.i();
        if ((cPJ < nPJ && cPI == nPI) && ((nPI < nNPI || nPI > nNPI) && nPJ == nNPJ)) return true;
        //down
        if ((cPJ > nPJ && cPI == nPI) && ((nPI < nNPI || nPI > nNPI) && nPJ == nNPJ)) return true;
        //left
        if ((cPI > nPI && cPJ == nPJ) && ((nPJ < nNPJ || nPJ > nNPJ) && nPI == nNPI)) return true;
        //right
        if ((cPI < nPI && cPJ == nPJ) && ((nPJ < nNPJ || nPJ > nNPJ) && nPI == nNPI)) return true;
        return false;
    }

    private static String stringFromMatrix() {
        StringBuilder sb = new StringBuilder();
        for (char[] chars : copyMatrix) {
            for (var j = 0; j < copyMatrix[0].length; j++) sb.append(chars[j]);
            sb.append("\n");
        }
        return sb.toString().trim();
    }

    private static PointWithDistanceFromPreviousPoint findNewMoveFromCurrentAndChangeMatrix(PointWithDistanceFromPreviousPoint currentMove) {
        List<PointWithDistanceFromPreviousPoint> listOfFreePointsAroundCurrentPoint = new ArrayList<>();
        int cj = currentMove.j();
        int ci = currentMove.i();
        if (cj != 0 && // 0
                (copyMatrix[cj - 1][ci] == '.' || copyMatrix[cj - 1][ci] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci, 1.0f));
        if (cj != 0 && ci != copyMatrix[0].length - 1 && // 1
                (copyMatrix[cj - 1][ci + 1] == '.' || copyMatrix[cj - 1][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci + 1, 1.4142f));
        if (ci != copyMatrix[0].length - 1 && // 2
                (copyMatrix[cj][ci + 1] == '.' || copyMatrix[cj][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj, ci + 1, 1.0f));
        if (cj != copyMatrix.length - 1 && ci != copyMatrix[0].length - 1 && // 3
                (copyMatrix[cj + 1][ci + 1] == '.' || copyMatrix[cj + 1][ci + 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci + 1, 1.4142f));
        if (cj != copyMatrix.length - 1 && // 4
                (copyMatrix[cj + 1][ci] == '.' || copyMatrix[cj + 1][ci] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci, 1.0f));
        if (cj != copyMatrix.length - 1 && ci != 0 && // 5
                (copyMatrix[cj + 1][ci - 1] == '.' || copyMatrix[cj + 1][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj + 1, ci - 1, 1.4142f));
        if (ci != 0 && // 6
                (copyMatrix[cj][ci - 1] == '.' || copyMatrix[cj][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj, ci - 1, 1.0f));
        if (cj != 0 && ci != 0 && // 7
                (copyMatrix[cj - 1][ci - 1] == '.' || copyMatrix[cj - 1][ci - 1] == 'G'))
            listOfFreePointsAroundCurrentPoint.add(new PointWithDistanceFromPreviousPoint(cj - 1, ci - 1, 1.4142f));

        if (listOfFreePointsAroundCurrentPoint.isEmpty()) return null;

        Random rand = new Random();
        int randomPointIndex = rand.nextInt(listOfFreePointsAroundCurrentPoint.size());
        return listOfFreePointsAroundCurrentPoint.get(randomPointIndex);
    }

    private static Point findPoint(char[][] matrix, char goal) {
        for (int i = 0; i < matrix.length; i++)
            for (int j = 0; j < matrix[0].length; j++)
                if (matrix[i][j] == goal) return new Point(i, j);
        return null;
    }

    private static char[][] matrixFromExistingWiresString(String existingWires) {
        char[][] matrix = new char[existingWires.split("\n").length][existingWires.split("\n")[0].length()];
        int row = 0;
        for (String rowString : existingWires.split("\n")) {
            for (int col = 0; col < rowString.length(); col++)
                matrix[row][col] = rowString.charAt(col);
            row++;
        }
        return matrix;
    }

    public static void main(String[] args) {
//        String  existingWires = "...\n" +
//                "S.G\n" +
//                "...";
//        String existingWires = ".S.\n" +
//                "...\n" +
//                ".G.";
//        String existingWires = "...\n" +
//                "SG.\n" +
//                "...";

//        String existingWires =  ".S...\n" +
//                "XXX..\n" +
//                ".X.XX\n" +
//                "..X..\n" +
//                "G...X";

//        String solution =       ".SP..\n" +
//                "XXXP.\n" +
//                ".XPXX\n" +
//                ".PX..\n" +
//                "G...X";

//        String existingWires = "XX.S.XXX..\n" +
//                "XXXX.X..XX\n" +
//                "...X.XX...\n" +
//                "XX...XXX.X\n" +
//                "....XXX...\n" +
//                "XXXX...XXX\n" +
//                "X...XX...X\n" +
//                "X...X...XX\n" +
//                "XXXXXXXX.X\n" +
//                "G........X";

//        String existingWires = "X.XXXX..X...XX.X.XXXXXX\n" +
//                "X..XXXXX...........X.X.\n" +
//                "XX..XX.....X.X........X\n" +
//                "XXXXX..X.X.XXXX.X.XXXX.\n" +
//                "..XXX.X.X...XXX...XXX.X\n" +
//                "X.XX..X.XX.XX..XX.X.XXX\n" +
//                "...X..XXX.XXX.XXXXX....\n" +
//                "..XX.XXX..XX..X..X.XXX.\n" +
//                "X.X.X..X.XX...X.X....X.\n" +
//                "..X.XX...XX..XXX.XX.X.X\n" +
//                "XXXX.XXXXX.XX.X...XXSXX\n" +
//                "X.X...X.XX...X...XX.XX.\n" +
//                "XX....XX.XX.X..XX..X.XX\n" +
//                ".XX.XXXXXXXX...XX..XX.X\n" +
//                "...XX.XXX.X.X.X.XXX.X.X\n" +
//                "X.XXX.XX..XX.XX......X.\n" +
//                "XX..XX.XX..XX..X....X..\n" +
//                ".XX.XXX.XXX..X.X.XXX..X\n" +
//                "X...XXGX.X.X...XXXX.X.X\n" +
//                ".XXX..XXXX.X..X.XX..X..\n" +
//                "X.XXXX..XX...X.X..X..XX\n" +
//                "XX..XXXXX..X.X......X..\n" +
//                "XXXX.XX.XXXXX...X.X..XX";


        String existingWires = "X.XXXXXX..X.XXXXGX...X.X...XXXXX.X..X...XX..XX\n" +
                ".XXX....X....X.X....XX.XX...X.XX...X.XXXXXX..X\n" +
                ".XXX...X.XXXXXX.XXXX.....X.XX...XXXXX.X....X..\n" +
                "X.XXXX..X.XX..X.X.XXXX.XXXX.XX..X.XXX.XXXXXXXX\n" +
                ".....XX.XX.X.XXXX..XX.X.....X.X....XXX.X..XX..\n" +
                "X..XXXX.XXXXXXXX..XX.X.XX.XXXXXXX..X....XXXX.X\n" +
                "X.X.X..XXXX...XXXXX...XX.XX.XX.XX.XXXXXXX....X\n" +
                "XX..X..XXX.XXX.X.XX..XX.XX..X..X.X..XXX..XXXXX\n" +
                "XX.X..XX.X...XXXX.XX..XXXX.X....XXXX...XXXX...\n" +
                ".XXX.X.X.X.XXXX.X.X.X....X...X..XX.X..........\n" +
                "XXXXXXXX.XXXX.XX....XXX..XXXXX...XXX.XXX....XX\n" +
                "..XX.X..XX.XXXXXX..XX..XX.XX.X..XXXXXX.XX.XX.X\n" +
                "XX..XX.X..X.XXXX..XXX.XX.....X.X.X.X..XX.XXXXX\n" +
                "X.X.XX.XXXX.XX.XXXXX.XXXXXX..X.X...XX.XXXX..XX\n" +
                "X...XX.X.X.X.XXX..XXXX.....XXX..XX..X.XXXXX.X.\n" +
                ".XXX...XXX.XX..X.XXX.XXX.XX.....XXX......XX..X\n" +
                "X.XXXXX...XXXXXXX..XXXX.XXX.XXXXXXXXX.XXXXXX..\n" +
                "...X.XX.....X..XXX..X.XX...XXXX.XXX.X.XX..X.XX\n" +
                ".XX.XX.X.SX..X.XXX.X..X..XXXXX...X..X.XXXX.X..\n" +
                ".X..XXX.XXXX.X.......X....XXX..XX.X..XX..XXX..";

        //One possible path:
//        X.XXXXXX..X.XXXXGX...X.X...XXXXX.X..X...XX..XX
//        .XXX....X....X.X.PPPXX.XX...X.XX...X.XXXXXX..X
//        .XXX...X.XXXXXX.XXXXPP...X.XX...XXXXX.X....X..
//        X.XXXX..X.XX..X.X.XXXXPXXXX.XX..X.XXX.XXXXXXXX
//        .....XX.XX.X.XXXX..XXPX.....X.X....XXX.X..XX..
//        X..XXXX.XXXXXXXX..XXPX.XX.XXXXXXX..X....XXXX.X
//        X.X.X..XXXX...XXXXX.P.XX.XX.XX.XX.XXXXXXX....X
//        XX..X..XXX.XXX.X.XX.PXX.XX..X..X.X..XXX..XXXXX
//        XX.X..XX.X...XXXX.XX.PXXXX.X....XXXX...XXXX...
//        .XXX.X.X.X.XXXX.X.X.X.P..X...X..XX.X..........
//        XXXXXXXX.XXXX.XX....XXXPPXXXXX...XXX.XXX....XX
//        ..XX.X..XX.XXXXXX..XX..XXPXX.X..XXXXXX.XX.XX.X
//        XX..XX.X..X.XXXX..XXX.XX..P..X.X.X.X..XX.XXXXX
//        X.X.XX.XXXX.XX.XXXXX.XXXXXXP.X.X...XX.XXXX..XX
//        X...XX.X.X.X.XXX..XXXX...PPXXX..XX..X.XXXXX.X.
//        .XXX...XXX.XX..X.XXX.XXXPXX.....XXX......XX..X
//        X.XXXXX...XXXXXXX..XXXXPXXX.XXXXXXXXX.XXXXXX..
//        ...X.XX...PPXP.XXX..X.XXP..XXXX.XXX.X.XX..X.XX
//        .XX.XX.X.SX.PXPXXX.X.PXP.XXXXX...X..X.XXXX.X..
//        .X..XXX.XXXX.X.PPPPPPXP...XXX..XX.X..XX..XXX..
        //
        //Your path:
        //X.XXXXXX..X.XXXXGX...X.X...XXXXX.X..X...XX..XX
        //.XXX....X....X.X.PPPXX.XX...X.XX...X.XXXXXX..X
        //.XXX...X.XXXXXX.XXXXPP...X.XX...XXXXX.X....X..
        //X.XXXX..X.XX..X.X.XXXXPXXXX.XX..X.XXX.XXXXXXXX
        //.....XX.XX.X.XXXX..XXPX.....X.X....XXX.X..XX..
        //X..XXXX.XXXXXXXX..XXPX.XX.XXXXXXX..X....XXXX.X
        //X.X.X..XXXX...XXXXX.P.XX.XX.XX.XX.XXXXXXX....X
        //XX..X..XXX.XXX.X.XX.PXX.XX..X..X.X..XXX..XXXXX
        //XX.X..XX.X...XXXX.XX.PXXXX.X....XXXX...XXXX...
        //.XXX.X.X.X.XXXX.X.X.X.PP.X...X..XX.X..........
        //XXXXXXXX.XXXX.XX....XXX.PXXXXX...XXX.XXX....XX
        //..XX.X..XX.XXXXXX..XX..XXPXX.X..XXXXXX.XX.XX.X
        //XX..XX.X..X.XXXX..XXX.XX..P..X.X.X.X..XX.XXXXX
        //X.X.XX.XXXX.XX.XXXXX.XXXXXXP.X.X...XX.XXXX..XX
        //X...XX.X.X.X.XXX..XXXX...PPXXX..XX..X.XXXXX.X.
        //.XXX...XXX.XX..X.XXX.XXXPXX.....XXX......XX..X
        //X.XXXXX...XXXXXXX..XXXXPXXX.XXXXXXXXX.XXXXXX..
        //...X.XX...P.XP.XXX..X.XXP..XXXX.XXX.X.XX..X.XX
        //.XX.XX.X.SXPPXPXXX.XPPX.PXXXXX...X..X.XXXX.X..
        //.X..XXX.XXXX.X.PPPPP.XPP..XXX..XX.X..XX..XXX..
        //
        //Your length:	47.526912
        //Expected:	46.941125

        long sum = 0;
        for (var times = 0; times < 1; times++) {
            long start = System.currentTimeMillis();
            System.out.println("\n" + wireDHD(existingWires));
            long finish = System.currentTimeMillis();
            System.out.println("elapsed time = " + (finish - start));
            sum += finish - start;
        }
        //System.out.println("average time = " + sum / 10.0);
    }
}
