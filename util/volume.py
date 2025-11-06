#!/usr/bin/env python3

import selectors
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import alsaaudio as aa

# Type aliases
MixerType = aa.Mixer
CallbackType = Callable[[MixerType, int], None]


def handle_mixer_event(mixer_obj: MixerType, fd_mask: int) -> None:
    """Callback function to handle the event."""
    # Read the event to clear the ALSA buffer and get the new volume/status
    # We don't necessarily need the return values, but the read() call
    # is necessary to process the event within ALSA's C library.
    mixer_obj.handleevents()
    update_mixer(mixer_obj)


def update_mixer(mixer_obj: MixerType) -> None:
    # Now you can query the mixer for updated information
    volume: List[int] = mixer_obj.getvolume(units=aa.VOLUME_UNITS_PERCENTAGE)
    db: List[int] = mixer_obj.getvolume(units=aa.VOLUME_UNITS_DB)
    mute_status: List[int] = mixer_obj.getmute()
    # db values are typically in hundredths of dB depending on backend; keep original scaling
    print(
        f"Mixer event detected! New Volume: {volume[0]}%, "
        f"New Volume (dB): {db[0]/100.0}dB, Muted: {bool(mute_status[0])}"
    )


def monitor_mixer() -> None:
    # 1. Initialize the ALSA Mixer object
    # Use the appropriate control name for your system (e.g., 'Master', 'PCM')
    try:
        mixer: MixerType = aa.Mixer(control="Speaker", device="speaker")
    except aa.ALSAAudioError as e:
        print(f"Error opening mixer: {e}")
        print("Available mixers:", aa.mixers(cardindex=0))
        return

    print(f"Monitoring '{mixer.mixer()}' mixer ")

    # 2. Get the pollable file descriptor and event mask
    # polldescriptors() returns a list of tuples: [(fd, event_mask), ...]
    descriptors: List[Tuple[int, int]] = mixer.polldescriptors()
    if not descriptors:
        print("Could not get poll descriptors for the mixer. Exiting.")
        return

    # In most cases, there will be only one descriptor
    fd, event_mask = descriptors[0]

    # Translate the ALSA event mask to the selectors module's mask if necessary.
    # The aa event mask is often compatible with the 'select' module's
    # POLLIN/POLLOUT, which map closely to selectors.EVENT_READ/EVENT_WRITE.
    # We are interested in read events (changes).
    events: int = selectors.EVENT_READ

    # 3. Create a DefaultSelector instance
    sel: selectors.BaseSelector = selectors.DefaultSelector()

    # 4. Register the file descriptor with the selector and associate the callback
    # The 'data' field stores the mixer object so the callback can access it.
    data: Dict[str, Any] = {"mixer": mixer, "callback": handle_mixer_event}  # type: ignore[var-annotated]
    sel.register(fd, events, data=data)

    # 5. Event loop
    print("Waiting for mixer events (Ctrl+C to exit)...")
    try:
        while True:
            # Wait for events with a timeout
            ready_list: List[Tuple[selectors.SelectorKey, int]] = sel.select(timeout=1.0)
            if not ready_list:
                print("Timeout, doing other work...")  # Optional: handle timeouts
                continue

            for key, mask in ready_list:
                # Retrieve the stored data (mixer object and callback)
                stored: Dict[str, Any] = key.data  # type: ignore[assignment]
                callback: CallbackType = stored["callback"]
                mixer_obj: MixerType = stored["mixer"]

                # Call the handler function
                callback(mixer_obj, mask)

    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    finally:
        # 6. Close the selector
        sel.close()


if __name__ == "__main__":
    monitor_mixer()
