#!/bin/sh

# Creating D-Bus rule file for wsbrd which is necessary when it is executed as root
WSBRD_DBUS_CONF_FILE=/etc/dbus-1/system.d/com.silabs.Wisun.BorderRouter.conf

WSBRD_DBUS_CONF_FILE_CONTENT=$(cat <<EOF
<!DOCTYPE busconfig PUBLIC "-//freedesktop//DTD D-Bus Bus Configuration 1.0//EN"
"http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy context="default">
    <allow own="com.silabs.Wisun.BorderRouter"/>
    <allow send_destination="com.silabs.Wisun.BorderRouter"/>
    <allow receive_sender="com.silabs.Wisun.BorderRouter"/>
    <allow send_interface="com.silabs.Wisun.BorderRouter"/>
    <allow receive_interface="com.silabs.Wisun.BorderRouter"/>
    <allow send_interface="org.freedesktop.DBus.Introspectable"/>
    <allow send_interface="org.freedesktop.DBus.Properties"/>
    <allow receive_interface="org.freedesktop.DBus.Introspectable"/>
    <allow receive_interface="org.freedesktop.DBus.Properties"/>
  </policy>
</busconfig>
EOF
)

if [ ! -f "$WSBRD_DBUS_CONF_FILE" ]; then
  echo "Writing WSBRD D-Bus configuration file"
  echo $WSBRD_DBUS_CONF_FILE_CONTENT | sudo tee $WSBRD_DBUS_CONF_FILE
else
  echo "File WSBRD D-Bus configuration file already exists"
fi