package com.frank.family;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.BitmapDrawable;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.View;

import java.nio.ByteBuffer;

public class CubeLayout extends View {

    protected static int               PointCount = 8;
    protected static int               FaceCount = 6;
    protected        float             edgeLength;
    protected        float[]           pX;
    protected        float[]           pY;
    protected        float[]           pZ;
    protected        Common.PointXYZ[] P;
    protected        Common.PointXYZ[] oldP;
    protected        Common.PointF[]   p;
    public static    Common.PointXYZ CubeCenter = new Common.PointXYZ();
    public static float firstDistance, lastDistance;
    public static float StartX, StartY, EndX, EndY;
    public static boolean  isSingle  = true;
    private MainActivity activity;
    private int[][] face = {{0, 3, 2, 1}, {3, 7, 6, 2}, {1, 2, 6, 5}
            , {0, 1, 5, 4}, {4, 5, 6, 7}, {0, 4, 7, 3}};
    private Bitmap[] originalCubeBMP = new Bitmap[6];
    private byte[][] originalCubeByteArray = new byte[6][];
    private Bitmap newCubeBMP;
    private byte[] newCubeByteArray;
    float diff0, diff1, diff2, diff3, diff5;

    float[][] facePointsX = new float[3][4];
    float[][] facePointsY = new float[3][4];
    private SurfaceHolder mSurfaceHolder;
    private Canvas mCanvas;
    private boolean isRun = true;

    public CubeLayout(Context context) {
        super(context);
        activity = (MainActivity)context;
        createCube();
    }

    static {
        System.loadLibrary("native-lib");
    }

    public static native int isCubeVisible(float xE, float yE, float zE,
                                           float xF, float yF, float zF,
                                           float xG, float yG, float zG);

    public native void InitializationCube(int width00, int height00,
     byte[] in1, byte[] in2, byte[] in3,  byte[] in4, byte[] in5, byte[] in6, byte[] out);

    public native void transformsCube(int count, int [] index
                                      , float[][] fPointsX, float[][] fPointsY);

    public void resetCube()
    {
        resetCube0();
        for (int i = 0; i < PointCount; i++) {
            oldP[i].reset(pX[i], pY[i], pZ[i]);
        }
        StartX = 0;
        StartY = 0;
        EndX = 0;
        EndY = 0;
        convert();
    }

    private void resetCube0()
    {
        pX[0] = CubeCenter.x;
        pX[2] = pX[0];
        pX[4] = pX[0];
        pX[6] = pX[0];
        pX[3] = CubeCenter.x - diff5;
        pX[7] = CubeCenter.x - diff5;
        pX[1] = CubeCenter.x + diff5;
        pX[5] = CubeCenter.x + diff5;

        pY[0] = CubeCenter.y - diff1;
        pY[6] = CubeCenter.y + diff1;
        pY[2] = CubeCenter.y + diff0;
        pY[5] = CubeCenter.y + diff0;
        pY[7] = CubeCenter.y + diff0;
        pY[1] = CubeCenter.y - diff0;
        pY[3] = CubeCenter.y - diff0;
        pY[4] = CubeCenter.y - diff0;

        pZ[0] = CubeCenter.z;
        pZ[6] = CubeCenter.z;
        pZ[2] = CubeCenter.z + diff3;
        pZ[4] = CubeCenter.z - diff3;
        pZ[1] = CubeCenter.z + diff2;
        pZ[3] = CubeCenter.z + diff2;
        pZ[5] = CubeCenter.z - diff2;
        pZ[7] = CubeCenter.z - diff2;

//        pX[0] = CubeCenter.x - edgeLength / 2;
//        pX[3] = pX[0];
//        pX[4] = pX[0];
//        pX[7] = pX[0];
//        pX[1] = pX[0] + edgeLength;
//        pX[2] = pX[1];
//        pX[5] = pX[1];
//        pX[6] = pX[1];
//        pY[0] = CubeCenter.y - edgeLength / 2;
//        pY[1] = pY[0];
//        pY[2] = pY[0];
//        pY[3] = pY[0];
//
//        pY[4] = pY[0] + edgeLength;
//        pY[5] = pY[4];
//        pY[6] = pY[4];
//        pY[7] = pY[4];
//
//        pZ[2] = 0;
//        pZ[3] = pZ[2];
//        pZ[6] = pZ[2];
//        pZ[7] = pZ[2];
//
//        pZ[0] = pZ[2] - edgeLength;
//        pZ[1] = pZ[0];
//        pZ[4] = pZ[0];
//        pZ[5] = pZ[0];
//        StartX = 400;
//        StartY = 100;
//        EndX = 35;
//        EndY = 410;
    }

    private void createCube() {
        pX = new float[PointCount];
        pY = new float[PointCount];
        pZ = new float[PointCount];
        P = new Common.PointXYZ[PointCount];
        oldP = new Common.PointXYZ[PointCount];
        p = new Common.PointF[PointCount];
        edgeLength = Common._screenWidth / 2f;
        CubeCenter.reset(Common._screenWidth / 2f, Common._screenHeight / 4f, -edgeLength / 2);

        diff0 = 0.289f * edgeLength;
        diff1 = 0.866f * edgeLength;
        diff3 = 0.816f * edgeLength;
        diff2 = 0.408f * edgeLength;
        diff5 = 0.707f * edgeLength;
        resetCube0();
        originalCubeBMP[0] = ((BitmapDrawable) getResources().getDrawable(R.drawable.dage)).getBitmap();
        originalCubeBMP[1] = ((BitmapDrawable) getResources().getDrawable(R.drawable.erge)).getBitmap();
        originalCubeBMP[2] = ((BitmapDrawable) getResources().getDrawable(R.drawable.dajie)).getBitmap();
        originalCubeBMP[3] = ((BitmapDrawable) getResources().getDrawable(R.drawable.erjie)).getBitmap();
        originalCubeBMP[4] = ((BitmapDrawable) getResources().getDrawable(R.drawable.me)).getBitmap();
        originalCubeBMP[5] = ((BitmapDrawable) getResources().getDrawable(R.drawable.juan)).getBitmap();
        newCubeBMP = Bitmap.createBitmap(Common._screenWidth, Common._screenHeight / 2, Bitmap.Config.ARGB_8888);

        bmp2byteCube();
        InitializationCube(originalCubeBMP[0].getWidth(), originalCubeBMP[0].getHeight()
                    ,originalCubeByteArray[0]
                    ,originalCubeByteArray[1]
                    ,originalCubeByteArray[2]
                    ,originalCubeByteArray[3]
                    ,originalCubeByteArray[4]
                    ,originalCubeByteArray[5]
                    ,newCubeByteArray);
        for (int i = 0; i < PointCount; i++) {
            P[i] = new Common.PointXYZ(pX[i], pY[i], pZ[i]);
            oldP[i] = new Common.PointXYZ(pX[i], pY[i], pZ[i]);
            p[i] = new Common.PointF();
        }
        convert();
    }

    private void bmp2byteCube() {

        int bytes;
        ByteBuffer buf;

        for (int i = 0; i < FaceCount; i++) {
            bytes = originalCubeBMP[i].getByteCount();
            buf = ByteBuffer.allocate(bytes);
            originalCubeBMP[i].copyPixelsToBuffer(buf);
            originalCubeByteArray[i] = buf.array();
        }
        bytes = newCubeBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        newCubeBMP.copyPixelsToBuffer(buf);
        newCubeByteArray = buf.array();
    }

    public void saveOldPoints() {
        for (int i = 0; i < PointCount; i++) {
            oldP[i].reset(P[i].x, P[i].y, P[i].z);
        }
    }

    public void convert() {
        for (int i = 0; i < PointCount; i++) {
            Common.getNewPoint(StartX, StartY, EndX, EndY, CubeCenter, oldP[i], P[i]);
            Common.resetPointF(p[i], P[i]);
        }
    }

    public boolean convert2() {
        if (firstDistance == 0) return false;

        for (int i = 0; i < PointCount; i++) {
            Common.getNewSizePoint(firstDistance, lastDistance,
                    StartX, StartY, EndX, EndY, CubeCenter, oldP[i], P[i], true);
            Common.resetPointF(p[i], P[i]);
        }
        return true;
    }

    public void drawSolid(Canvas canvas, Paint paint) {
        int[] index = {0, 0, 0};
        int[] bmpIndex = {0, 0, 0};
        int count = 0;
        for (int i = 0; i < FaceCount; i++) {
            if (isCubeVisible(P[face[i][1]].x, P[face[i][1]].y, P[face[i][1]].z,
                    P[face[i][2]].x, P[face[i][2]].y, P[face[i][2]].z,
                    P[face[i][3]].x, P[face[i][3]].y, P[face[i][3]].z) > 0) {
                index[count] = i;
                bmpIndex[count++] = i;
            }
        }
        for(int i = 0; i < count; i++) {
            for(int j = 0; j < 4; j++) {
                facePointsX[i][j] = p[face[index[i]][j]].x;
                facePointsY[i][j] = p[face[index[i]][j]].y;
            }
        }
        try {
            transformsCube(count, index, facePointsX, facePointsY);
            newCubeBMP.copyPixelsFromBuffer(ByteBuffer.wrap(newCubeByteArray));
            canvas.drawBitmap(newCubeBMP, 0, 0, paint);
        } catch (Exception e) {
            String a = e.toString();
        }
    }

    public void turn() {
        StartX = 100;
        StartY = CubeCenter.y;
        saveOldPoints();
        firstDistance = 0;
        isSingle = true;
        EndX = 30;
        EndY = CubeCenter.y;
        convert();
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getAction() & MotionEvent.ACTION_MASK) {
            case MotionEvent.ACTION_DOWN: {
                activity.stopDrawTimer();
                StartX = event.getX();
                StartY = event.getY();
                saveOldPoints();
                firstDistance = 0;
                isSingle = event.getPointerCount() == 1;
                break;
            }
            case MotionEvent.ACTION_MOVE: {
                EndX = event.getX();
                EndY = event.getY();
                if (event.getPointerCount() == 2) {
                    if (firstDistance == 0) {
                        float tmp;
                        for (int i = 0; i < PointCount; i++) {
                            tmp = (float) Common.getDistance(p[i].x, p[i].y, CubeCenter.x, CubeCenter.y);
                            if (tmp > firstDistance) {
                                firstDistance = tmp;
                            }
                        }
                    }
                    isSingle = false;
                    lastDistance = (float) Common.getDistance(event.getX(0),
                            event.getY(0),
                            event.getX(1),
                            event.getY(1));
                    lastDistance /= 2;
                    if (convert2())
                    {
                        invalidate();
                    }
                } else if (event.getPointerCount() == 1 && isSingle) {
                    convert();
                    invalidate();
                }
                break;
            }
            case MotionEvent.ACTION_UP: {
                activity.setDelay(5000);
                activity.startDrawTimer();
            }
        }
        return true;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        Paint paint = new Paint();
        canvas.drawColor(Color.BLACK);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.WHITE);
        drawSolid(canvas, paint);
    }
}
