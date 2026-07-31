
# Depends on the version...
import pylsl
streams = pylsl.resolve_streams()

#from pylsl import resolve_stream
#streams = resolve_stream()

# Resolve all available LSL streams (waits up to 5 seconds)
print("Looking for LSL streams...")
# Print name and type of each stream
for i, stream in enumerate(streams):
    print(f"{i+1}. Name: {stream.name()}, Type: {stream.type()}, Source ID: {stream.source_id()}")

    inlet = pylsl.StreamInlet(stream)
    print(f"Stream Inlet info: {inlet.info()}")
    print(f"Channel count: {inlet.info().channel_count()}")