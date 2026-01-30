# Troubleshooting

## "Received message has timestamp ... older for ... which is more than allowed timeout"

If you see an error like:
```text
[bicycle_steering_controller]: Received message has timestamp ... older for 5038.62... which is more then allowed timeout (2.0000).
```

the **bicycle_steering_controller** is rejecting `TwistStamped` messages because their `header.stamp` is too far in the past compared to the controller's current time (allowed age is set by `reference_timeout`, default 2.0 s). The large "older for" value (e.g. 5038 s) means a **time/clock mismatch**, not just slow joystick updates.

### Common causes

1. **Joystick on another machine with wrong or unsynced clock**  
   The machine running `teleop_twist_joy` may have its clock far behind the robot (e.g. after sleep, or no NTP). Messages are stamped with that machine's time, so the robot sees them as very old.

2. **Mixed use of simulation time**  
   If the controller (or controller_manager) uses `use_sim_time:=true` (e.g. from another launch) but the joystick uses wall clock (or the opposite), stamps and "now" can differ by a large amount.

### What to do

- **Same machine, same time source:** Run `teleop_twist_joy` on the same machine as the robot and in the same ROS context (same `use_sim_time`). For real hardware, do not set `use_sim_time` unless you intentionally use a `/clock` publisher.
- **Joystick on another computer:** Sync that computer's clock (e.g. enable NTP, run `sudo ntpdate` or equivalent). Ensure it is not 80+ minutes behind the robot.
- **Optional workaround:** Increasing `reference_timeout` in `bringup/config/carlikebot_controllers.yaml` (e.g. to 5.0) only helps for a few seconds of delay; it does **not** fix a 5038 s gap. Fix the clock/time source so stamps and controller time match.

### Time sync when using Mac (joystick) + Raspberry Pi (robot)

Both machines should use NTP so their clocks stay within about a second of each other (well within the 2 s `reference_timeout`).

1. **Raspberry Pi (robot)**  
   Raspberry Pi OS uses `systemd-timesyncd` by default. Ensure it is enabled and synced:
   ```bash
   sudo timedatectl status   # check "System clock synchronized: yes"
   sudo timedatectl set-ntp true   # enable NTP if off
   ```
   If the Pi has no internet, you can sync it once to your Mac (see step 4).

2. **Mac (joystick)**  
   - **System Settings → General → Date & Time**  
   - Turn on **Set time and date automatically** and choose a time server (e.g. **Time.apple.com**).  
   After wake from sleep, the Mac may take a moment to resync; wait a few seconds or run:
   ```bash
   sudo sntp -sS time.apple.com   # force sync (macOS)
   ```

3. **Verify**  
   On both machines run:
   ```bash
   date -u +"%Y-%m-%d %H:%M:%S %Z"
   ```
   The printed times should differ by at most 1–2 seconds.

4. **Pi has no internet: sync Pi to Mac once**  
   On the Pi, set the time from the Mac over SSH (run on the Pi; replace `mac.local` with your Mac's hostname or IP, and ensure SSH key or password auth works):
   ```bash
   sudo date -s "$(ssh mac.local date -u +"%Y-%m-%d %H:%M:%S")"
   ```
   Or plug the Pi into the network so it can use NTP (e.g. `pool.ntp.org`) and skip this step.
