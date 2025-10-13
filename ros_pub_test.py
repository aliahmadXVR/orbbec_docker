import time
import roslibpy

# Connect to rosbridge (localhost for now)
client = roslibpy.Ros(host='localhost', port=9090)

print('Connecting to ROS bridge...')
client.run()
print('Connected to ROS bridge ✅')

# Define the topic
talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
talker.advertise()

count = 0
try:
    while client.is_connected:
        message = f'Hello from Chatbot! Count: {count}'
        talker.publish(roslibpy.Message({'data': message}))
        print(f'Published: {message}')
        count += 1
        time.sleep(1)   # 1 Hz
except KeyboardInterrupt:
    print('Interrupted, shutting down...')
finally:
    talker.unadvertise()
    client.terminate()
    print('Disconnected cleanly.')
