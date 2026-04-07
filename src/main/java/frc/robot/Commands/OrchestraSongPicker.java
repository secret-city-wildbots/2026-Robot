package frc.robot.Commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class OrchestraSongPicker extends Command {

    private final Orchestra _orchestra = new Orchestra();

    private final String[] _songs = new String[] {
        "song1.chrp",
        "song2.chrp",
        "song3.chrp",
        "song4.chrp",
        "song5.chrp",
        "song6.chrp",
        "song7.chrp",
        "song8.chrp",
        "song9.chrp",  /* the remaining songs play better with three or more FXs */
        "song10.chrp",
        "song11.chrp",
    };

    private int _songSelection = 0;
    private int _timeToPlayLoops = 0;

    private final Joystick _joy;
    private int _lastButton = 0;
    private int _lastPOV = 0;

    public OrchestraSongPicker(Joystick joystick) {
        this._joy = joystick;

        TalonFX[] motors = new TalonFX[] {
            new TalonFX(47, "rio"),
            new TalonFX(48, "rio")
        };

        for (TalonFX motor : motors) {
            _orchestra.addInstrument(motor, 0);
        }
    }

    /** Load a song by offset (+1 next, -1 previous, 0 reload current) */
    private void loadMusicSelection(int offset) {
        _songSelection += offset;

        if (_songSelection >= _songs.length) {
            _songSelection = 0;
        }
        if (_songSelection < 0) {
            _songSelection = _songs.length - 1;
        }

        _orchestra.loadMusic(_songs[_songSelection]);
        System.out.println("[OrchestraPicker] Song selected: " + _songs[_songSelection] + " | D-pad left/right to change.");

        /* Schedule play after a delay — gives Orchestra time to parse the chirp file.
           Calling play() immediately after loadMusic can result in an invalid action error. */
        _timeToPlayLoops = 10;
    }

    @Override
    public void initialize() {
        _lastButton = 0;
        _lastPOV = 0;
        loadMusicSelection(0);
    }

    @Override
    public void execute() {
        /* Auto-play after load delay */
        if (_timeToPlayLoops > 0) {
            --_timeToPlayLoops;
            if (_timeToPlayLoops == 0) {
                System.out.println("[OrchestraPicker] Auto-playing: " + _songs[_songSelection]);
                _orchestra.play();
            }
        }

        int btn = getButton();
        int currentPOV = _joy.getPOV();

        /* Button press handling */
        if (_lastButton != btn) {
            _lastButton = btn;

            switch (btn) {
                case 1: /* toggle play / pause */
                    if (_orchestra.isPlaying()) {
                        _orchestra.pause();
                        System.out.println("[OrchestraPicker] Paused.");
                    } else {
                        _orchestra.play();
                        System.out.println("[OrchestraPicker] Playing...");
                    }
                    break;

                case 2: /* toggle play / stop */
                    if (_orchestra.isPlaying()) {
                        _orchestra.stop();
                        System.out.println("[OrchestraPicker] Stopped.");
                    } else {
                        _orchestra.play();
                        System.out.println("[OrchestraPicker] Playing...");
                    }
                    break;
            }
        }

        /* D-pad song navigation */
        if (_lastPOV != currentPOV) {
            _lastPOV = currentPOV;

            switch (currentPOV) {
                case 90:  /* D-pad right — next song */
                    loadMusicSelection(+1);
                    break;
                case 270: /* D-pad left — previous song */
                    loadMusicSelection(-1);
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[OrchestraPicker] end(), interrupted: " + interrupted);
        _orchestra.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** @return 0 if no button pressed, index of button otherwise */
    private int getButton() {
        for (int i = 1; i < 9; ++i) {
            if (_joy.getRawButton(i)) {
                return i;
            }
        }
        return 0;
    }
}
