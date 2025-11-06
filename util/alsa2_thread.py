# Run alsa mixer polling in a separate thread

import json
import logging
import os
import time
import selectors
import threading
import alsaaudio as aa

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Communicate shutdown to the thread
shutdown_event = threading.Event()


class MixerThread:
    def __init__(self, read_fd):
        self.read_fd = read_fd
        self.mixer = None
        self.sel = selectors.DefaultSelector()

    def update_mixer(self) -> None:
        if self.mixer:
            # Now you can query the mixer for updated information
            logging.info("Updating mixer status...")
            mixer_obj = self.mixer
            volume = mixer_obj.getvolume(units=aa.VOLUME_UNITS_PERCENTAGE)
            db = mixer_obj.getvolume(units=aa.VOLUME_UNITS_DB)
            mute_status = mixer_obj.getmute()
            # db values are typically in hundredths of dB depending on backend; keep original scaling
            logging.info(
                f"Mixer event detected! New Volume: {volume[0]}%, "
                f"New Volume (dB): {db[0]/100.0}dB, Muted: {bool(mute_status[0])}"
            )
        else:
            logging.warning("Mixer object is not initialized.")


    def monitor_mixer(self, control: str, device: str) -> None:
        '''
        # 1. Initialize the ALSA Mixer object
        # Use the appropriate control name for your system (e.g., 'Master', 'PCM')
        '''
        if self.mixer is not None:
            logger.warning("Mixer is already being monitored.")
            return
        try:
            self.mixer = aa.Mixer(control=control, device=device)
        except aa.ALSAAudioError as e:
            logger.warning(f"Error opening mixer: {e}")
            logger.info("Available mixers: %s", aa.mixers(cardindex=0))
            return

        logger.info("Monitoring '%s' mixer", self.mixer.mixer())

        # Get the pollable file descriptor and event mask
        # polldescriptors() returns a list of tuples: [(fd, event_mask), ...]
        descriptors = self.mixer.polldescriptors()
        if not descriptors:
            logger.warning("Could not get poll descriptors for the mixer. Exiting.")
            return

        # In most cases, there will be only one descriptor
        fd, event_mask = descriptors[0]

        # Register the file descriptor with the selector and associate the callback
        # The 'data' field stores the mixer object so the callback can access it.
        mixer_data = {"params": None, "callback": self.handle_mixer_event}
        self.sel.register(fd, selectors.EVENT_READ, data=mixer_data)

    def handle_mixer_event(self, _) -> None:
        """Callback function to handle the event."""
        # Read the event to clear the ALSA buffer and get the new volume/status
        # We don't necessarily need the return values, but the read() call
        # is necessary to process the event within ALSA's C library.
        if self.mixer:
            self.mixer.handleevents()
            self.update_mixer()
        else:
            logging.warning("Mixer object is not initialized in event handler.")

    def handle_message_event(self, read_fd) -> None:
        """Callback function to handle data when the read end of the pipe is ready."""
        data = os.read(read_fd, 1024)
        if data:
            logger.info("Received message: %s", data.decode('utf-8'))
            message = json.loads(data.decode('utf-8'))
            if message.get("command") == "start_monitoring":
                control = message.get("control", "Master")
                device = message.get("device", "default")
                self.monitor_mixer(control, device)
        else:
            # If read returns empty bytes, the write end is closed
            logger.info("Main thread closed the pipe. Unregistering selector.")
            self.sel.unregister(read_fd)
            os.close(read_fd)

    def run(self):
        """A thread worker to monitor or inititiate ALSA mixer events."""

        # Register the read pipe
        read_data = {"params": read_fd, "callback": self.handle_message_event}
        self.sel.register(read_fd, selectors.EVENT_READ, data=read_data)

        # Event loop
        logger.info("Waiting for mixer events (Ctrl+C to exit)...")
        while not shutdown_event.is_set():
            # Wait for events without a timeout
            ready_list = self.sel.select()
            if not ready_list:
                logger.info("Empty ready list, closing")
                break

            for key, _ in ready_list:
                # Retrieve the stored data (mixer object and callback)
                stored = key.data
                callback = stored["callback"]

                # Call the handler function
                callback(stored["params"])
        self.sel.close()

if __name__ == "__main__":
    read_fd, write_fd = os.pipe()
    control="Speaker"
    device="speaker"
    monitor_mixer = MixerThread(read_fd)
    thread  = threading.Thread(target=monitor_mixer.run)
    thread.start()

    # Send a message to the thread to initiate monitoring
    message = {"command": "start_monitoring", "control": control, "device": device}
    os.write(write_fd, json.dumps(message).encode('utf-8'))
    try:
        while True:
            time.sleep(2)
            #os.write(write_fd, json.dumps(message).encode('utf-8'))

    except KeyboardInterrupt:
        print("Main thread interrupted by user.")
    finally:
        shutdown_event.set()
        os.close(write_fd)  # This will fire the thread selector to exit
        thread.join()
