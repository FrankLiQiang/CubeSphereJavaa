package com.frank.family;

import android.app.Activity;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.drawable.BitmapDrawable;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.LinearLayout;

import androidx.annotation.RequiresApi;

import java.io.IOException;
import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends Activity {

    private Handler _handler = null;
    private boolean isStopTimer = true;
    private  CubeLayout cube;
    private BallLayout ball;
    private int delayA = 0;
    private int delayB = 0;
    private A taskA    = null;
    private B taskB    = null;
    private Bitmap backgroundBMP;
    private byte[] backgroundByteArray;

    static {
        System.loadLibrary("native-lib");
    }

    public static native void setEYE(int width, int height, float eyeX, float eyeY, float eyeZ
            , int bgW, int bgH, byte[] bg);

    public native void EndDraw();

    @RequiresApi(api = Build.VERSION_CODES.P)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        initBeepSound();
        Common.getScreenSize(getWindowManager());
        Common.DepthZ = Common._screenWidth / 2f;
        Common.Eye.reset(Common._screenWidth / 2f, Common._screenHeight / 4f, Common._screenHeight * 2);
        backgroundBMP = ((BitmapDrawable) getResources().getDrawable(R.drawable.bg)).getBitmap();
        bmp2byteCube();
        setEYE(Common._screenWidth, Common._screenHeight / 2, Common.Eye.x, Common.Eye.y, Common.Eye.z
                ,backgroundBMP.getWidth()
                ,backgroundBMP.getHeight()
                ,backgroundByteArray);

        // 设置页面全屏 刘海屏 显示
        Window window = getWindow();
        WindowManager.LayoutParams lp = window.getAttributes();
        lp.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
        window.setAttributes(lp);
        final View decorView = window.getDecorView();
        decorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        // 设置页面全屏 刘海屏 显示

        cube = new CubeLayout(this);
        ball = new BallLayout(this);
        LinearLayout.LayoutParams mParams = new LinearLayout.LayoutParams(Common._screenWidth, Common._screenHeight / 2);
        cube.setLayoutParams(mParams);
        ball.setLayoutParams(mParams);

        LinearLayout linearLayout = new LinearLayout(this);
        linearLayout.setOrientation(LinearLayout.VERTICAL);
        linearLayout.addView(cube, mParams);
        linearLayout.addView(ball, mParams);
        hideNavigationBar();
        setContentView(linearLayout);

        _handler = new MyHandler(this);
        startDrawTimer();
    }

    private void hideNavigationBar() {
        Window window = getWindow();
        View decorView = window.getDecorView();
        decorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    private void bmp2byteCube() {
        int bytes;
        ByteBuffer buf;
        bytes = backgroundBMP.getByteCount();
        buf = ByteBuffer.allocate(bytes);
        backgroundBMP.copyPixelsToBuffer(buf);
        backgroundByteArray = buf.array();
    }

    private static final float BEEP_VOLUME = 9.10f;
    private MediaPlayer mediaPlayer;
    private final MediaPlayer.OnCompletionListener beepListener = new MediaPlayer.OnCompletionListener() { // 声音
        public void onCompletion(MediaPlayer mediaPlayer) {
            mediaPlayer.seekTo(0);
        }
    };

    private void initBeepSound() {
        if (mediaPlayer == null) {
            mediaPlayer = new MediaPlayer();
            mediaPlayer.setAudioStreamType(AudioManager.STREAM_MUSIC);
            mediaPlayer.setOnCompletionListener(beepListener);

            AssetFileDescriptor file = getResources().openRawResourceFd(R.raw.music);
            try {
                mediaPlayer.setDataSource(file.getFileDescriptor(), file.getStartOffset(), file.getLength());
                file.close();
                mediaPlayer.setVolume(BEEP_VOLUME, BEEP_VOLUME);
                mediaPlayer.setLooping(true);
                mediaPlayer.prepare();
                mediaPlayer.start();
            } catch (IOException e) {
                mediaPlayer = null;
            }
        }
    }

    class A extends TimerTask {
        @Override
        public void run() {
            if (!isStopTimer) {
                Message msg = new Message();
                msg.what = 1;
                _handler.sendMessage(msg);
            }
        }

    }

    class B extends TimerTask {
        @Override
        public void run() {
            if (!isStopTimer) {
                Message msg = new Message();
                msg.what = 2;
                _handler.sendMessage(msg);
            }
        }
    }

    public void setDelay(int iDelay) {
        delayA = iDelay;
        delayB = iDelay;
    }

    public void startDrawTimer() {
        isStopTimer = false;
        taskA = new A();
        new Timer().schedule(taskA, delayA);
    }

    public void stopDrawTimer() {
        isStopTimer = true;
        if (taskA != null) {
            taskA.cancel();
            taskA = null;
        }
        if (taskB != null) {
            taskB.cancel();
            taskB = null;
        }
    }

    public void handleObj(Message msg) {
        if (!isStopTimer) {
            if (msg.what == 1) {
                if(delayA > 0)
                {
                    cube.resetCube();
                }
                delayA = 0;
                cube.turn();
                taskB = new B();
                new Timer().schedule(taskB, 0);
            } else {
                if(delayB > 0)
                {
                    ball.createBall();
                }
                delayB = 0;
                ball.turn();
                taskA = new A();
                new Timer().schedule(taskA, 0);
            }
        }
    }

    static class MyHandler extends Handler {
        private WeakReference<MainActivity> _outer;

        public MyHandler(MainActivity activity) {
            _outer = new WeakReference<MainActivity>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            MainActivity outer = _outer.get();
            if (outer != null) {
                outer.handleObj(msg);
            }
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        EndDraw();
        stopDrawTimer();
        this.finish();
        System.exit(0);
    }

    @Override
    public void onBackPressed() {
        EndDraw();
        stopDrawTimer();
        this.finish();
        System.exit(0);
    }
}
