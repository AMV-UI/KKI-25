import pyudev


def get_webcam_device_idx(target_serial):
    """
    Finds the /dev/videoX path for a webcam with specific VID/PID.

    Args:
        target_vid (str): Vendor ID (e.g., '046d')
        target_pid (str): Product ID (e.g., '0825')

    Returns:
        id: The device node id (e.g., '/dev/video0' will output 0) or None if not found.
    """
    context = pyudev.Context()

    # Webcams reside in the 'video4linux' subsystem
    for device in context.list_devices(subsystem="video4linux"):
        # Get the device attributes safely
        serial = device.get("ID_SERIAL")

        # Check if IDs match (comparing as lowercase strings)
        if serial == target_serial:
            # OPTIONAL: Filter out metadata/index nodes.
            # Many cameras create two nodes (e.g., video0 and video1).
            # Usually, the 'capture' device is the one you want.
            capabilities = device.get("ID_V4L_CAPABILITIES", "")
            if ":capture:" in capabilities:
                return int(device.device_node[10])

    return None

