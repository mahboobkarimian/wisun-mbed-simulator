# enable core generation
sudo sysctl -w kernel.core_pattern=core.%u.%p.%t
sudo ip tuntap add mode tun tun0
sudo ip addr add fd12:3456::2e99:8528:350:bbb7/64 dev tun0
sudo ip link set tun0 up
