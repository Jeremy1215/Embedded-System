import dbus
import dbus.mainloop.glib
from gi.repository import GLib
import sys

# DBus configuration
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()

# Bluetooth adapter (typically hci0 on Raspberry Pi)
ADAPTER_PATH = "/org/bluez/hci0"
adapter = bus.get_object("org.bluez", ADAPTER_PATH)
adapter_iface = dbus.Interface(adapter, "org.bluez.Adapter1")

# UUIDs (must match your iPhone app)
SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB"
CHARACTERISTIC_UUID = "0000FFE1-0000-1000-8000-00805F9B34FB"
CCCD_UUID = "00002902-0000-1000-8000-00805F9B34FB"

class BLEManager:
    def __init__(self):
        self.device = None
        self.char = None
        self.mainloop = GLib.MainLoop()
        
    def start_scan(self):
        """Start device discovery"""
        print("Scanning for BLE devices...")
        adapter_iface.StartDiscovery()
        GLib.timeout_add(10000, self.stop_scan)  # Scan for 10 seconds
        
    def stop_scan(self):
        """Stop discovery and list devices"""
        adapter_iface.StopDiscovery()
        print("\nDiscovered devices:")
        objects = bus.get_object("org.bluez", "/")
        manager = dbus.Interface(objects, "org.freedesktop.DBus.ObjectManager")
        for path, _ in manager.GetManagedObjects().items():
            if "org.bluez.Device1" in path:
                dev = bus.get_object("org.bluez", path)
                props = dbus.Interface(dev, "org.freedesktop.DBus.Properties")
                addr = props.Get("org.bluez.Device1", "Address")
                name = props.Get("org.bluez.Device1", "Name") if "Name" in props.GetAll("org.bluez.Device1") else "Unknown"
                print(f"- {name} ({addr})")
                if "iPhone" in name:  # Replace with your device name
                    self.device = dev
                    self.connect_device(path)
        return False
        
    def connect_device(self, path):
        """Connect to the device"""
        print(f"\nConnecting to device at {path}...")
        device_props = dbus.Interface(self.device, "org.freedesktop.DBus.Properties")
        device_props.Set("org.bluez.Device1", "Connected", dbus.Boolean(True))
        
        # Wait for services to resolve
        GLib.timeout_add(2000, self.discover_services, path)
        
    def discover_services(self, device_path):
        """Discover services and characteristics"""
        print("Discovering services...")
        objects = bus.get_object("org.bluez", "/")
        manager = dbus.Interface(objects, "org.freedesktop.DBus.ObjectManager")
        
        for path, interfaces in manager.GetManagedObjects().items():
            if device_path in path:
                if "org.bluez.GattCharacteristic1" in interfaces:
                    uuid = interfaces["org.bluez.GattCharacteristic1"]["UUID"]
                    if uuid == CHARACTERISTIC_UUID:
                        print(f"Found target characteristic at {path}")
                        self.char = bus.get_object("org.bluez", path)
                        self.enable_notifications()
                        return False
        print("Target characteristic not found!")
        return False
        
    def enable_notifications(self):
        """Enable notifications on the characteristic"""
        print("Enabling notifications...")
        char_iface = dbus.Interface(self.char, "org.bluez.GattCharacteristic1")
        
        # Get CCCD descriptor
        cccd_path = char_iface.object_path + "/desc" + CCCD_UUID[-8:]
        try:
            cccd = bus.get_object("org.bluez", cccd_path)
            cccd_iface = dbus.Interface(cccd, "org.bluez.GattDescriptor1")
            
            # Write 0x0002 to CCCD (enable indications)
            cccd_iface.WriteValue([dbus.Byte(0x02), dbus.Byte(0x00)], {})
            print("Notifications enabled!")
            
            # Register signal handler for notifications
            bus.add_signal_receiver(
                self.on_notification,
                signal_name="PropertiesChanged",
                dbus_interface="org.freedesktop.DBus.Properties",
                path=self.char.object_path
            )
            
        except dbus.exceptions.DBusException as e:
            print(f"Failed to enable notifications: {e}")
            
    def on_notification(self, interface, changed_props, invalidated_props):
        """Handle incoming notifications"""
        if "Value" in changed_props:
            value = bytes(changed_props["Value"])
            try:
                print(f"Received: {value.decode('utf-8')}")
            except UnicodeDecodeError:
                print(f"Received raw data: {value.hex()}")
                
    def run(self):
        """Start the main event loop"""
        try:
            self.start_scan()
            self.mainloop.run()
        except KeyboardInterrupt:
            print("\nDisconnecting...")
            if self.device:
                device_props = dbus.Interface(self.device, "org.freedesktop.DBus.Properties")
                device_props.Set("org.bluez.Device1", "Connected", dbus.Boolean(False))
            self.mainloop.quit()
            sys.exit(0)

if __name__ == "__main__":
    print("Starting BLE Central (Python 3.11 with dbus)")
    manager = BLEManager()
    manager.run()