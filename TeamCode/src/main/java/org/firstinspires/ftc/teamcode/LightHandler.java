package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class LightHandler {
    private static final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final DigitalChannel red;
    private final DigitalChannel green;
    private long startTime = System.currentTimeMillis();
    private final List<State> instructions = new ArrayList<>();
    private int currentIndex = 0;
    public static final long unit = 200;

    public LightHandler(HardwareMap map) {
        red = map.get(DigitalChannel.class, "red");
        green = map.get(DigitalChannel.class, "green");
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setColor(Color color) {
        red.setState(color.red);
        green.setState(color.green);
    }

    public LightHandler dot() {
        instructions.add(State.DOT);
        return this;
    }

    public LightHandler dash() {
        instructions.add(State.DASH);
        return this;
    }

    public LightHandler pause() {
        instructions.add(State.PAUSE);
        return this;
    }

    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    public void runAuto(LinearOpMode opMode) {
        executor.submit(() -> {
            // Set up to blink robopandas in morse code
            this
                .pause()
                //R
                .dot().dash().dot().pause()
                //O
                .dash().dash().dash().pause()
                //B
                .dash().dot().dot().dot().pause()
                //O
                .dash().dash().dash().pause()
                //P
                .dot().dash().dash().dot().pause()
                //A
                .dot().dash().pause()
                //N
                .dash().dot().pause()
                //D
                .dash().dot().dot().pause()
                //A
                .dot().dash().pause()
                //S
                .dot().dot().dot();
            opMode.waitForStart();
            resetTimer();
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) tick();
            setColor(Color.OFF);
        });
    }

    public void tick() {
        if (System.currentTimeMillis() - startTime < instructions.get(currentIndex).on) {
            setColor(Color.GREEN);
        }
        else if (System.currentTimeMillis() - startTime < instructions.get(currentIndex).off) {
            setColor(Color.OFF);
        }
        else if (currentIndex < instructions.size()-1) {
            currentIndex++;
            resetTimer();
        }
        else {
            setColor(Color.OFF);
        }
    }

    public enum State {
        DOT(LightHandler.unit, LightHandler.unit*2),
        DASH(LightHandler.unit*3, LightHandler.unit*4),
        PAUSE(0, LightHandler.unit*2);

        // Time on / off in milliseconds
        public final long on;
        public final long off;

        State(long on, long off) {
            this.on = on;
            this.off = off;
        }
    }

    public enum Color {
        OFF(false, false),
        RED(true, false),
        YELLOW(true, true),
        GREEN(false, true);

        public final boolean red;
        public final boolean green;

        Color(boolean red, boolean green) {
            this.red = red;
            this.green = green;
        }
    }
}
