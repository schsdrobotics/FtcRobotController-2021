package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

public class CameraHandler {
    private Camera camera;
    private OpMode opMode;
    private WebcamName camName;
    private CameraManager cameraManager;
    private Handler callbackHandler;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private CameraCaptureSession cameraCaptureSession;

    public CameraHandler(OpMode opMode) {
        this.opMode = opMode;
        this.cameraManager = ClassFactory.getInstance().getCameraManager();

        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(2));
        frameQueue.setEvictAction(Bitmap::recycle);

        camName = opMode.hardwareMap.get(WebcamName.class, "camera");
        Deadline deadline = new Deadline(Integer.MAX_VALUE, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, camName, null);
        if (camera == null) {
            throw new RuntimeException("Failed to connect to camera: " + camName.getDeviceName());
        }

        int format = ImageFormat.YUY2;
        CameraCharacteristics cameraCharacteristics = camName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), format)) {
            throw new RuntimeException("image format not supported");
        }

        final Size size = cameraCharacteristics.getDefaultSize(format);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(format, size);
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();

        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(format, size, fps);
                        session.startCapture(captureRequest, (CameraCaptureSession.CaptureCallback) (session1, request1, cameraFrame) -> {
                                    /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                     * for the duration of the callback. So we copy here manually. */
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                },
                                Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) -> {})
                        );
                        synchronizer.finish(session);
                    } catch (CameraException | RuntimeException e) {
                        session.close();
                        synchronizer.finish(null);
                        throw new RuntimeException("Exception during capture");
                    }
                }}));
        } catch (CameraException | RuntimeException e) {
            synchronizer.finish(null);
            throw new RuntimeException("Exception starting camera");
        }

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        cameraCaptureSession = synchronizer.getValue();
        if (cameraCaptureSession == null) throw new RuntimeException("Failed to start capture session");
    }

    public void capture() {
        File file = new File( "C:/Users/OWNER/Desktop/webcam-frame.jpg");
        try {
            if (file.exists()) file.delete();
            file.createNewFile();
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                Bitmap bmp = frameQueue.poll();
                if (bmp != null) {
                    bmp.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                    bmp.recycle();
                    System.out.println("picture taken");
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to capture frame", e);
        }
    }

    public static boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}
