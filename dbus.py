from pydbus import SessionBus
from pydbus.generic import signal
from gi.repository import GLib


bus = SessionBus()

# Create an object that will proxy for a particular remote object.
remote_object = bus.get(
    "br.copel.inversandro.DBusTutorial", # Bus name
    "/br/copel/inversandro/DBusTutorial" # Object path
)

# Introspection returns an XML document containing information
# about the methods supported by an interface.
print ("Introspection data:\n")
print (remote_object.Introspect()[0])

# Get the power management object
print(remote_object.Sum(5,15))

print(remote_object.Mul(5,15))

print(remote_object.Introspect())


loop = GLib.MainLoop()
dbus_filter = "/br/copel/inversandro/DBusTutorial"


def cb_server_signal_emission(*args):
    """
    Callback on emitting signal from server
    """
    print("Message: ", args)
    print("Data: ", str(args[4][0]))


if __name__=="__main__":
    # Subscribe to bus to monitor for server signal emissions
    bus.subscribe(object = dbus_filter, signal_fired = cb_server_signal_emission)    
    loop.run()

