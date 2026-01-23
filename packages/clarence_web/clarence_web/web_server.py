import http.server
import os
import rclpy
import socketserver
import signal
import threading

def main():
    print(f'Hi from clarence_web at {os.getcwd()}')
    from ament_index_python.packages import get_package_share_directory
    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('clarence_web')
    print(f'Package share directory: {package_share_directory}')


    # --- Configuration ---
    # The directory you want to serve files from.
    # os.path.expanduser handles the '~' for the home directory.
    SERVE_DIRECTORY = os.path.join(package_share_directory, 'site')
    PORT = 8000
    # -------------------

    # Check if the directory exists
    if not os.path.isdir(SERVE_DIRECTORY):
        print(f"Error: Directory '{SERVE_DIRECTORY}' not found.")
        print("Please create it or change the SERVE_DIRECTORY variable.")
        exit(1)

    # Change the current working directory to the one we want to serve
    try:
        os.chdir(SERVE_DIRECTORY)
    except OSError as e:
        print(f"Error changing directory: {e}")
        exit(1)

    # Handler to serve files from the new current directory
    Handler = http.server.SimpleHTTPRequestHandler
    server = socketserver.TCPServer(("", PORT), Handler)

    def run_server():
        with server:
            print(f"Serving at port {PORT}")
            server.serve_forever()

    server_thread = threading.Thread(target=run_server)

    def signal_handler(sig, frame):
        print('Signal received, shutting down the web server...')
        server.shutdown()
        server_thread.join()
        rclpy.shutdown()
        exit(0)

    # Create and start the server
    # server_thread.daemon = True
    server_thread.start()
    signal.signal(signal.SIGINT, signal_handler)
    print(f"Serving files from: {SERVE_DIRECTORY}")
    print(f"Access the server at: http://localhost:{PORT}")

    with server:
        try:
            server.serve_forever()
        except Exception as e:
            print(f"Web server has shut down: {e}")
        server_thread.join()


if __name__ == '__main__':
    main()
