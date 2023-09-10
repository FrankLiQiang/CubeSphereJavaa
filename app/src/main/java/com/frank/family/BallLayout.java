package com.frank.family;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.BitmapDrawable;

import android.view.MotionEvent;
import android.view.View;

import java.nio.ByteBuffer;

public class BallLayout extends View {

    protected static float ballR, r;

    public static Common.PointXYZ BallCenter = new Common.PointXYZ();
    private Bitmap originalBallBMP, newBallBMP, fatherBMP, motherBMP;
    private byte[] ballByteArray, newBallByteArray, fatherByteArray, motherByteArray;

    private Common.PointXYZ Arctic = new Common.PointXYZ();
    private Common.PointXYZ Meridian = new Common.PointXYZ();
    private Common.PointF ArcticF = new Common.PointF();
    private Common.PointF MeridianF = new Common.PointF();
    private Common.PointF Meridian0F = new Common.PointF();
    private Common.PointXYZ OldArctic = new Common.PointXYZ();
    private Common.PointXYZ OldMeridian = new Common.PointXYZ();
    private boolean isStopTimer = true;
    private MainActivity activity;

    public static float firstDistance, lastDistance;
    public static float StartX, StartY, EndX, EndY;
    public static boolean  isSingle  = true;

    public BallLayout(Context context) {
        super(context);
        activity = (MainActivity)context;
        fatherBMP = ((BitmapDrawable) getResources().getDrawable(R.drawable.father)).getBitmap();
        motherBMP = ((BitmapDrawable) getResources().getDrawable(R.drawable.mother)).getBitmap();
        originalBallBMP = ((BitmapDrawable) getResources().getDrawable(R.drawable.all)).getBitmap();
        newBallBMP = Bitmap.createBitmap(Common._screenWidth, Common._screenHeight / 2, Bitmap.Config.ARGB_8888);
        createBall();
    }

    static {
        System.loadLibrary("native-lib");
    }

    public native void InitializationBall(float z, float cx, float cy, float cz);

    public native void InitializationBall2(int width, int height, float R, float r, byte[] in
            ,int widthP, int heightP, byte[] inF, byte[] inM, byte[] out);

    public native int transformsBall(float ArcticX, float ArcticY
            , float ArcticZ, float MeridianX, float MeridianY, float MeridianZ, float r);

    public void createBall() {
        ballR = Common._screenWidth / 2f;
        BallCenter.reset(Common._screenWidth / 2f, Common._screenHeight / 4f, -ballR);
        InitializationBall(Common.DepthZ, BallCenter.x, BallCenter.y, BallCenter.z);
        Arctic.reset(BallCenter.x, BallCenter.y - ballR, BallCenter.z);
        Meridian.reset(BallCenter.x - ballR, BallCenter.y, BallCenter.z);
        Common.resetPointF(ArcticF, Arctic);
        Common.resetPointF(MeridianF, Meridian);
        Common.PointXYZ Meridian0 = new Common.PointXYZ();
        Meridian0.reset(BallCenter.x - ballR, BallCenter.y, BallCenter.z);
        Common.resetPointF(Meridian0F, Meridian0);
        r = BallCenter.x - Meridian0F.x;
        newBallBMP = Bitmap.createBitmap(Common._screenWidth, Common._screenHeight, Bitmap.Config.ARGB_8888);
        bmp2byteBall();
        InitializationBall2(originalBallBMP.getWidth(), originalBallBMP.getHeight()
                , ballR, r, ballByteArray, fatherBMP.getWidth(), motherBMP.getHeight()
                , fatherByteArray, motherByteArray, newBallByteArray);
    }

    private void bmp2byteBall() {

        int bytes;
        ByteBuffer buf;

        bytes = originalBallBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        originalBallBMP.copyPixelsToBuffer(buf);
        ballByteArray = buf.array();

        bytes = newBallBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        newBallBMP.copyPixelsToBuffer(buf);
        newBallByteArray = buf.array();

        bytes = fatherBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        fatherBMP.copyPixelsToBuffer(buf);
        fatherByteArray = buf.array();

        bytes = motherBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        motherBMP.copyPixelsToBuffer(buf);
        motherByteArray = buf.array();
    }

    public void drawPicBall(Canvas canvas, Paint paint) {
        try {
            transformsBall(Arctic.x, Arctic.y, Arctic.z, Meridian.x, Meridian.y, Meridian.z, r);
            newBallBMP.copyPixelsFromBuffer(ByteBuffer.wrap(newBallByteArray));
            canvas.drawBitmap(newBallBMP, 0, 0, paint);
            if (!isStopTimer)
            {
                //new Timer().schedule(new A(), 50);
            }
        } catch (Exception e) {
            String a = e.toString();
        }
    }

    public void saveOldPointsBall() {
        OldArctic.reset(Arctic.x, Arctic.y, Arctic.z);
        OldMeridian.reset(Meridian.x, Meridian.y, Meridian.z);
    }

    public void convertBall() {
        Common.getNewPoint(StartX, StartY, EndX, EndY, BallCenter, OldArctic, Arctic);
        Common.getNewPoint(StartX, StartY, EndX, EndY, BallCenter, OldMeridian, Meridian);
        Common.resetPointF(ArcticF, Arctic);
        Common.resetPointF(MeridianF, Meridian);
    }

    public void convertBall2() {
        Common.getNewSizePoint(firstDistance, lastDistance, StartX, StartY, EndX, EndY, BallCenter, OldArctic, Arctic, false);
        r = lastDistance / 2;
        Common.getNewSizePoint(firstDistance, lastDistance, StartX, StartY, EndX, EndY, BallCenter, OldMeridian, Meridian, false);
    }

    public void turn() {
        StartX = 30;
        StartY = BallCenter.y;
        saveOldPointsBall();
        isSingle = true;
        EndX = 100;
        EndY = BallCenter.y;
        convertBall();
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getAction() & MotionEvent.ACTION_MASK) {
            case MotionEvent.ACTION_DOWN: {
                activity.stopDrawTimer();
                StartX = event.getX();
                StartY = event.getY();
                saveOldPointsBall();
                if (event.getPointerCount() == 2) {
                    firstDistance = (float) Common.getDistance(event.getX(0),
                                    event.getY(0),
                                    event.getX(1),
                                    event.getY(1));
                }
                isSingle = event.getPointerCount() == 1;
                break;
            }
            case MotionEvent.ACTION_MOVE: {
                EndX = event.getX();
                EndY = event.getY();
                if (event.getPointerCount() == 2) {
                    isSingle = false;
                    lastDistance = (float) Common.getDistance(event.getX(0),
                            event.getY(0),
                            event.getX(1),
                            event.getY(1));
                    convertBall2();
                    invalidate();
                } else if (event.getPointerCount() == 1 && isSingle) {
                    convertBall();
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

        drawPicBall(canvas, paint);
    }
}
