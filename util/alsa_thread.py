# Run alsa mixer polling in a separate thread

import os
import time
import selectors
import threading
import alsaaudio as aa

# Create a selector object
sel = selectors.DefaultSelector()

# Communicate shutdown to the thread
shutdown_event = threading.Event()

def handle_mixer_event(mixer_obj, fd_mask) -> None:
    """Callback function to handle the event."""
    # Read the event to clear the ALSA buffer and get the new volume/status
    # We don't necessarily need the return values, but the read() call
    # is necessary to process the event within ALSA's C library.
    mixer_obj.handleevents()
    update_mixer(mixer_obj)

def handle_message_event(read_fd, mask) -> None:
    """Callback function to handle data when the read end of the pipe is ready."""
    data = os.read(read_fd, 1024)
    if data:
        print(f"Received message: {data.decode('utf-8')}")
    else:
        # If read returns empty bytes, the write end is closed
        print("Main thread closed the pipe. Unregistering selector.")
        sel.unregister(read_fd)
        os.close(read_fd)

def update_mixer(mixer_obj) -> None:
    # Now you can query the mixer for updated information
    volume = mixer_obj.getvolume(units=aa.VOLUME_UNITS_PERCENTAGE)
    db = mixer_obj.getvolume(units=aa.VOLUME_UNITS_DB)
    mute_status = mixer_obj.getmute()
    # db values are typically in hundredths of dB depending on backend; keep original scaling
    print(
        f"Mixer event detected! New Volume: {volume[0]}%, "
        f"New Volume (dB): {db[0]/100.0}dB, Muted: {bool(mute_status[0])}"
    )

def monitor_mixer(read_fd, control, device) -> None:
    """A thread worker to monitor or inititiate ALSA mixer events."""

    # 1. Initialize the ALSA Mixer object
    # Use the appropriate control name for your system (e.g., 'Master', 'PCM')
    try:
        mixer = aa.Mixer(control=control, device=device)
    except aa.ALSAAudioError as e:
        print(f"Error opening mixer: {e}")
        print("Available mixers:", aa.mixers(cardindex=0))
        return

    print(f"Monitoring '{mixer.mixer()}' mixer ")

    # 2. Get the pollable file descriptor and event mask
    # polldescriptors() returns a list of tuples: [(fd, event_mask), ...]
    descriptors = mixer.polldescriptors()
    if not descriptors:
        print("Could not get poll descriptors for the mixer. Exiting.")
        return

    # In most cases, there will be only one descriptor
    fd, event_mask = descriptors[0]

    # 4. Register the file descriptor with the selector and associate the callback
    # The 'data' field stores the mixer object so the callback can access it.
    mixer_data = {"params": mixer, "callback": handle_mixer_event}  # type: ignore[var-annotated]
    sel.register(fd, selectors.EVENT_READ, data=mixer_data)

    # 4a. Also register the read pipe
    read_data = {"params": read_fd, "callback": handle_message_event}
    sel.register(read_fd, selectors.EVENT_READ, data=read_data)
    # 5. Event loop
    print("Waiting for mixer events (Ctrl+C to exit)...")
    while not shutdown_event.is_set():
        # Wait for events with a timeout
        ready_list = sel.select()
        if not ready_list:
            print("Timeout, doing other work...")  # Optional: handle timeouts
            continue

        for key, mask in ready_list:
            # Retrieve the stored data (mixer object and callback)
            print(f"Event on fd {key.fd}")
            stored = key.data
            callback = stored["callback"]

            # Call the handler function
            callback(stored["params"], mask)
    sel.close()

if __name__ == "__main__":
    read_fd, write_fd = os.pipe()
    control="Speaker"
    device="speaker"
    thread  = threading.Thread(target=monitor_mixer, args=(read_fd, control, device))
    thread.start()
    try:
        while True:
            time.sleep(2)
            message = "Hello from main thread!"
            os.write(write_fd, message.encode('utf-8'))

    except KeyboardInterrupt:
        print("Main thread interrupted by user.")
    finally:
        shutdown_event.set()
        os.close(write_fd)  # This will fire the thread selector to exit
        thread.join()
