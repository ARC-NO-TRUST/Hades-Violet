#!/usr/bin/env python3

import dbus
import dbus.mainloop.glib
import dbus.exceptions
from gi.repository import GLib
import sys
import time

BLUEZ_SERVICE_NAME = 'org.bluez'
ADAPTER_IFACE = 'org.bluez.Adapter1'
ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

CUSTOM_MESSAGE = "B1: 1,2.30"

class BLEAdvertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        super().__init__(bus, self.path)

        self.ad_type = 'peripheral'
        self.manufacturer_data = {
            0xFFFF: dbus.Array(
                [dbus.Byte(b) for b in CUSTOM_MESSAGE.encode()],
                signature='y'
            )
        }

    def get_properties(self):
        return {
            ADVERTISEMENT_IFACE: {
                'Type': self.ad_type,
                'ManufacturerData': self.manufacturer_data,
                'LocalName': 'RPiBLE',
                'IncludeTxPower': dbus.Boolean(True),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(dbus_interface='org.freedesktop.DBus.Properties',
                         in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        return self.get_properties()[interface]

    @dbus.service.method(ADVERTISEMENT_IFACE,
                         in_signature='', out_signature='')
    def Release(self):
        print('Advertisement released')

def find_adapter(bus):
    obj = bus.get_object(BLUEZ_SERVICE_NAME, '/')
    manager = dbus.Interface(obj, 'org.freedesktop.DBus.ObjectManager')
    objects = manager.GetManagedObjects()

    for path, interfaces in objects.items():
        if ADAPTER_IFACE in interfaces:
            return path
    raise Exception('Bluetooth adapter not found')

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = find_adapter(bus)
    adapter = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADVERTISING_MANAGER_IFACE)

    advertisement = BLEAdvertisement(bus, 0)
    ad_manager.RegisterAdvertisement(advertisement.get_path(), {},
                                     reply_handler=lambda: print("Advertisement registered"),
                                     error_handler=lambda e: print(f"Failed to register: {e}"))

    try:
        loop = GLib.MainLoop()
        GLib.timeout_add_seconds(15, loop.quit)  # Stop after 15 seconds
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        ad_manager.UnregisterAdvertisement(advertisement.get_path())
        dbus.service.Object.remove_from_connection(advertisement)
        print("Advertisement stopped")

if __name__ == '__main__':
    main()
