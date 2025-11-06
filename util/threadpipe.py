import selectors
import os
import threading
import time

# Create a selector object
sel = selectors.DefaultSelector()

def worker_thread(write_fd):
    """The function run by the worker thread."""
    time.sleep(1) # Simulate some work
    message = "Hello from the worker thread!"
    # Write the message to the pipe (need to encode to bytes)
    os.write(write_fd, message.encode('utf-8'))
    time.sleep(1)
    message = "Worker thread signing off."
    os.write(write_fd, message.encode('utf-8'))
    os.close(write_fd) # Close the write end when done

def read_message(conn, mask):
    """Callback function to handle data when the read end of the pipe is ready."""
    data = os.read(conn.fileno(), 1024) # Read data from the pipe's file descriptor
    if data:
        print(f"Received: {data.decode('utf-8')}")
    else:
        # If read returns empty bytes, the write end is closed
        print("Worker thread closed the pipe. Unregistering selector.")
        sel.unregister(conn)
        os.close(conn.fileno())

# --- Main part of the script ---
if __name__ == "__main__":
    # Create a pipe: read_fd and write_fd are integer file descriptors
    read_fd, write_fd = os.pipe()

    # Convert the read file descriptor to a file object for the selector
    # We open it in non-blocking mode as required by selectors
    read_pipe = os.fdopen(read_fd, 'rb', 0)
    
    # Register the read pipe with the selector for read events, using our callback
    # The 'data' parameter stores the callback function
    sel.register(read_pipe, selectors.EVENT_READ, read_message)

    # Start the worker thread
    thread = threading.Thread(target=worker_thread, args=(write_fd,))
    thread.start()

    print("Main thread running selector loop. Waiting for messages from worker...")

    try:
        while sel.get_map(): # Loop as long as there are registered objects
            events = sel.select(timeout=None) # Block until events occur
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)
    except KeyboardInterrupt:
        print("Selector loop interrupted by user.")
    finally:
        sel.close()
        thread.join() # Wait for the worker thread to finish
        print("Program finished.")
