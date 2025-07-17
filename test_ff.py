from evdev import InputDevice, ecodes, ff
import time

# Replace this with your actual event device
dev = InputDevice('/dev/input/event9')  # Find yours with `evtest`

# Define a sine wave "bump" vibration
effect = ff.Effect(
    type=ecodes.FF_PERIODIC,
    id=-1,
    direction=0,  # 0 = along X axis
    trigger=ff.Trigger(button=0, interval=0),
    replay=ff.Replay(length=500, delay=0),  # 500ms duration
    effect_type=ff.EffectType(
        ff_periodic=ff.Periodic(
            waveform=ecodes.FF_SINE,       # Try FF_SQUARE or FF_SAW_UP for sharper bumps
            period=100,                    # Wave period in ms (shorter = more jittery)
            magnitude=0x6000,              # Strength of force
            offset=0,
            phase=0,
            envelope=ff.Envelope(attack_length=100, attack_level=0x2000,
                                 fade_length=100, fade_level=0)
        )
    )
)

# Upload and play the effect
effect_id = dev.upload_effect(effect)
dev.write(ecodes.EV_FF, effect_id, 1)
dev.syn()

# Wait for the effect to play
time.sleep(0.5)

# Remove the effect from memory
dev.erase_effect(effect_id)
