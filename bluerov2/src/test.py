from pymavlink import mavutil
port = 14551
m = mavutil.mavlink_connection('udpin:0.0.0.0:{}'.format(port))
m.wait_heartbeat()
print('connection success!')