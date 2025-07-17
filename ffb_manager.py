from evdev import InputDevice, ecodes, ff, UInput
from glob import glob

FF_SINE = 0
FF_SQUARE = 1
FF_TRIANGLE = 2
FF_SAW_UP = 3
FF_SAW_DOWN = 4

FF_SPRING = 0x0001
FF_CONSTANT = 0x0002
FF_PERIODIC = 0x0003
FF_SINE = 0x0004




class ForceFeedbackManager:
    def __init__(self, name_keyword="Logitech", device_path=None):
        self.device_path = device_path or self._autodetect_device(name_keyword)
        self.dev = InputDevice(self.device_path)
        self.ui = UInput.from_device(self.dev, name="FFB Device")
        self.effects = {}

    def _autodetect_device(self, keyword):
        for path in glob('/dev/input/event*'):
            try:
                dev = InputDevice(path)
                if keyword in dev.name:
                    return path
            except Exception:
                continue
        raise FileNotFoundError(f"No input device found matching keyword: {keyword}")

    def play_spring(self, strength=0.5, center=0, deadband=0):
        force = int(max(min(strength, 1.0), 0.0) * 0x7fff)
        effect = ff.Effect(
            ff_id=-1,
            type=FF_SPRING,
            direction=0,
            trigger=ff.Trigger(0, 0),
            replay=ff.Replay(0x7fff, 0),
            u=ff.EffectType(
                ff_spring=ff.Spring(
                    right_saturation=force,
                    left_saturation=force,
                    right_coeff=0x1000,
                    left_coeff=0x1000,
                    deadband=deadband,
                    center=center
                )
            )
        )
        self._apply_effect("spring", effect)

    def play_constant(self, magnitude=0.5, duration_ms=1000):
        mag = int(max(min(magnitude, 1.0), -1.0) * 0x7fff)
        effect = ff.Effect(
            ff_id=-1,
            type=ff.CONSTANT,
            direction=0,
            trigger=ff.Trigger(0, 0),
            replay=ff.Replay(duration_ms, 0),
            u=ff.EffectType(
                ff_constant=ff.Constant(
                    level=mag,
                    envelope=ff.Envelope(attack_length=0, attack_level=0,
                                         fade_length=0, fade_level=0)
                )
            )
        )
        self._apply_effect("constant", effect)

    def play_periodic(self, wave_type=FF_SINE, magnitude=0.3, period_ms=100, duration_ms=1000):
        mag = int(max(min(magnitude, 1.0), 0.0) * 0x7fff)
        effect = ff.Effect(
            ff_id=-1,
            type=wave_type,
            direction=0,
            trigger=ff.Trigger(0, 0),
            replay=ff.Replay(duration_ms, 0),
            u=ff.EffectType(
                ff_periodic=ff.Periodic(
                    waveform=wave_type,
                    magnitude=mag,
                    offset=0,
                    phase=0,
                    period=period_ms,
                    envelope=ff.Envelope(0, 0, 0, 0),
                    custom_len=0,
                    custom_data=None
                )
            )
        )
        self._apply_effect(f"periodic_{wave_type}", effect)

    def _apply_effect(self, key, effect):
        if key in self.effects:
            self.ui.erase_effect(self.effects[key])
        eid = self.ui.upload_effect(effect)
        self.effects[key] = eid
        self.ui.write(ecodes.EV_FF, eid, 1)

    def stop_all(self):
        for eid in self.effects.values():
            self.ui.erase_effect(eid)
        self.effects.clear()

    def close(self):
        self.stop_all()
        self.ui.close()
        self.dev.close()