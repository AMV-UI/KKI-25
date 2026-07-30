from inputs import get_gamepad


def main():
    print("Listening for gamepad inputs... Press Ctrl+C to stop.")

    try:
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type != "Sync":
                    print(
                        f"Type: {event.ev_type} | Code: {event.code} | State: {event.state}"
                    )

    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error finding gamepad: {e}")


if __name__ == "__main__":
    main()
