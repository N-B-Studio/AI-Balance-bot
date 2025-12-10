from inputs import get_gamepad

print("Listening for controller inputs...")

while True:
    events = get_gamepad()
    for event in events:
        print(f"{event.code}: {event.state}")