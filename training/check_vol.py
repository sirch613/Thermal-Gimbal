import modal

app = modal.App("check-vol")
vol = modal.Volume.from_name("drone-weights-vol")

@app.local_entrypoint()
def main():
    print("Listing files in volume:")
    try:
        entries = vol.listdir("/")
        for entry in entries:
            print(f"- {entry.path}")
    except Exception as e:
        print(f"Error listing volume: {e}")
