package com.frank.family;

import android.graphics.Point;
import android.view.WindowManager;

public class Common {

    public static int      _screenWidth;
    public static int      _screenHeight;
    public static PointXYZ Eye = new PointXYZ();
    public static double   square;
    public static float times;
    public static double angleY = 0;
    public static float DepthZ;

    public static void getScreenSize(WindowManager windowManager) {
        Point outSize = new Point();
        windowManager.getDefaultDisplay().getRealSize(outSize);
        _screenWidth = outSize.x;
        _screenHeight = outSize.y;
    }

    public static void resetPointF(PointF thisP, PointXYZ P) {
        thisP.x = (P.x - Eye.x) / (Eye.z - P.z) * (Eye.z - DepthZ) + Eye.x;
        thisP.y = (P.y - Eye.y) / (Eye.z - P.z) * (Eye.z - DepthZ) + Eye.y;
    }

    public static double getDistance(float x1, float y1, float x2, float y2) {
        square = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        return Math.sqrt(square);
    }

    public static void getNewSizePoint(float firstDistance, float lastDistance,
        float StartX, float StartY, float EndX, float EndY,
        PointXYZ ObjCenter, PointXYZ pOld, PointXYZ pNew, boolean isChange) {

        pNew.reset(pOld.x - ObjCenter.x, pOld.y - ObjCenter.y, pOld.z - ObjCenter.z);
        times = lastDistance / firstDistance;
        if(isChange) {
            pNew.reset(pNew.x * times, pNew.y * times, pNew.z * times);
        }

        getDistance(StartX, StartY, EndX, EndY);
        double a2 = square;
        double b = getDistance(ObjCenter.x, ObjCenter.y, StartX, StartY);
        double b2 = square;
        double c = getDistance(ObjCenter.x, ObjCenter.y, EndX, EndY);
        double c2 = square;

        double Acos = (b2 + c2 - a2) / (2 * b * c);
        double angleA;
        if (Math.abs(Acos) > 1) {
            angleA = 0;
        } else {
            angleA = Math.acos(Acos);
        }
        int dir = getDir(ObjCenter.x, ObjCenter.y, StartX, StartY, EndX, EndY);
        angleA *= dir;
        double x = pNew.x * Math.cos(angleA) - pNew.y * Math.sin(angleA);
        double y = pNew.y * Math.cos(angleA) + pNew.x * Math.sin(angleA);
        pNew.reset((float) x, (float) y, pNew.z);
        pNew.reset(pNew.x + ObjCenter.x, pNew.y + ObjCenter.y, pNew.z + ObjCenter.z);
    }

    public static int getDir(float x1, float y1, float x2, float y2, float x3, float y3) {
        return (x1 * y2 + x2 * y3 + x3 * y1 - x1 * y3 - x2 * y1 - x3 * y2) > 0 ? 1 : -1;
    }

    public static void getNewPoint(float StartX, float StartY, float EndX, float EndY,
            PointXYZ ObjCenter, PointXYZ pOld, PointXYZ pNew) {
        pNew.reset(pOld.x - ObjCenter.x, pOld.y - ObjCenter.y, pOld.z - ObjCenter.z);

        double a2 = (StartX - EndX) * (StartX - EndX);
        double b = getDistance(ObjCenter.x, ObjCenter.z, StartX, DepthZ);
        double b2 = square;
        double c = getDistance(ObjCenter.x, ObjCenter.z, EndX, DepthZ);
        double c2 = square;

        double Acos = (b2 + c2 - a2) / (2 * b * c);
        double angleA;
        if (Math.abs(Acos) > 1) {
            angleA = 0;
        } else {
            angleA = Math.acos(Acos) * 2;
        }
        if (EndX > StartX) angleA *= -1;
        angleY = angleA;
        double x = pNew.x * Math.cos(angleA) - pNew.z * Math.sin(angleA);
        double z = pNew.z * Math.cos(angleA) + pNew.x * Math.sin(angleA);
        pNew.reset((float) x, pNew.y, (float) z);

        //---------
        a2 = (StartY - EndY) * (StartY - EndY);
        b = getDistance(ObjCenter.y, ObjCenter.z, StartY, DepthZ);
        b2 = square;
        c = getDistance(ObjCenter.y, ObjCenter.z, EndY, DepthZ);
        c2 = square;

        Acos = (b2 + c2 - a2) / (2 * b * c);
        if (Math.abs(Acos) > 1) {
            angleA = 0;
        } else {
            angleA = Math.acos(Acos) * 2;
        }
        if (EndY > StartY) angleA *= -1;
        double y = pNew.y * Math.cos(angleA) - pNew.z * Math.sin(angleA);
        z = pNew.z * Math.cos(angleA) + pNew.y * Math.sin(angleA);

        pNew.reset(pNew.x, (float) y, (float) z);

        pNew.reset(pNew.x + ObjCenter.x, pNew.y + ObjCenter.y, pNew.z + ObjCenter.z);
    }

    public static class PointXYZ {
        public float x, y, z;

        public PointXYZ() {
        }

        public PointXYZ(float X, float Y, float Z) {
            x = X;
            y = Y;
            z = Z;
        }

        void reset(float X, float Y, float Z) {
            x = X;
            y = Y;
            z = Z;
        }
    }

    public static class PointF {
        public float x, y;

        PointF() {
        }
    }
}
