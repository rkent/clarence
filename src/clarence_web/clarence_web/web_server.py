import http.server
import socketserver
import os


def main():
    print(f'Hi from clarence_web at {os.getcwd()}')
    from ament_index_python.packages import get_package_share_directory
    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('clarence_web')
    print(f'Package share directory: {package_share_directory}')
    # exit(0)


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

    # Create and start the server
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"Serving files from: {SERVE_DIRECTORY}")
        print(f"Access the server at: http://localhost:{PORT}")
        httpd.serve_forever()
        print("Server stopped.")
        rospy.shutdown()


if __name__ == '__main__':
    main()
