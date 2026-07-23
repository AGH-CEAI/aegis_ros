# Remote setup

How to configure a remote machine (not the onboard robot PC) to talk to the station over the network.

> [!CAUTION]
> Before doing anything on real hardware, read the safety note in [aegis_bringup](../aegis_bringup/README.md#run-the-project).
> Never run the stack against the physical robot unattended.

## 1. Hosts

The UR driver, gripper and F/T sensor connect by hostname (see `robot_ip`/`ip_address` args in `aegis_control`/`aegis_description`).
`/etc/hosts` is per-machine — add these on every remote machine yourself, copying them into `/etc/hosts` doesn't happen automatically:

```
192.168.0.100   geonosis
192.168.100.10  aegis
192.168.100.10  aegis_ur
192.168.100.20  aegis_pc
192.168.100.30  aegis_ft
```

If running inside the [aegis_docker](https://github.com/AGH-CEAI/aegis_docker) container, `geonosis` is already added via `extra_hosts` in `docker-compose.yaml` — you only need to add it on the host if you run anything outside the container.

## 2. ROS domain ID

All machines that should see the same ROS 2 graph need the same `ROS_DOMAIN_ID` (containers default to `42`):

```bash
export ROS_DOMAIN_ID=42
```

If discovery doesn't work across subnets/VLANs, check multicast isn't blocked, or configure DDS discovery peers explicitly.

## 3. Cameras

No `/etc/hosts` entries needed for cameras:

- **Basler** ([pylon_cameras.yaml](../aegis_control/config/cameras/pylon_cameras.yaml)): identified by `device_user_id`.
  Static IP is configured once per camera via the Pylon IP Configurator (see main [README](../README.md#driver-installation---basler-cameras-manually)).
- **Luxonis** ([depthai_cameras.yaml](../aegis_control/config/cameras/depthai_cameras.yaml)): PoE-powered, but identified by its MxID (the `i_mx_id` parameter), not IP.

## 4. Verification

```bash
ping aegis_ur
ping aegis_ft
ping geonosis
ros2 topic list
```
