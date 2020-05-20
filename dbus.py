from pydbus import SessionBus
from pydbus.generic import signal
from gi.repository import GLib
import xml.etree.ElementTree as ET


bus = SessionBus()

# Create an object that will proxy for a particular remote object.
remote_object = bus.get(
    "br.wdbus.Example", # Bus name
    "/br/wdbus/Example" # Object path
)

# Introspection returns an XML document containing information
# about the methods supported by an interface.
print ("Introspection data:\n")
print (remote_object.Introspect())

# Get the power management object
print(remote_object.Sum(5,15))

print(remote_object.Mul(5,15))

tree = ET.ElementTree(ET.fromstring(remote_object.Introspect()))
root = tree.getroot()
root.tag
for child in root:
    print(child.tag, child.attrib)


loop = GLib.MainLoop()
dbus_filter = "/br/wdbus/Example"


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

