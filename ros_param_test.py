import time
import roslibpy

# Connect to rosbridge (localhost or your ROS master IP)
client = roslibpy.Ros(host='192.168.8.1', port=9090)

print('Connecting to ROS bridge...')
client.run()
print('Connected to ROS bridge ✅')

# Define the parameter object
smiley_param = roslibpy.Param(client, '/smiley_param')

try:
    while client.is_connected:
        # Set to "smiling"
        smiley_param.set('smiling')
        print('Set /smiley_param = smiling')
        time.sleep(2)

        # Set to "charging"
        smiley_param.set('waving')
        print('Set /smiley_param = waving')
        time.sleep(2)

except KeyboardInterrupt:
    print('Interrupted, shutting down...')
finally:
    client.terminate()
    print('Disconnected cleanly.')
