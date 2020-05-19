#from pydbus import SessionBus
import pydbus
bus = pydbus.SessionBus()

# Create an object that will proxy for a particular remote object.
remote_object = bus.get(
    "org.example.TestServer", # Bus name
    "/org/example/TestObject" # Object path
)

# Introspection returns an XML document containing information
# about the methods supported by an interface.
print ("Introspection data:\n")
print (remote_object.Introspect()[0])

# Get the power management object
#print(remote_object.Properties)
print(remote_object.Ping())

print(remote_object.Introspect())
print(remote_object.Mul(5,15))
#a = ((5,15))
#print(remote_object.Adder(a))

version = remote_object.Get('org.example.TestServer', "Version")
print(version)
#iface = remote_object.Interface(wifi, dbus_interface='org.freedesktop.DBus.Properties')

# creating proxy 'Get' method
#m = iface.get_dbus_method("Get", dbus_interface=None)

# getting Id of active connection
#Id = m("org.freedesktop.NetworkManager.Connection.Active", "Id")

