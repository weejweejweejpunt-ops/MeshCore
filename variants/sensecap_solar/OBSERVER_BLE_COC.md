# SenseCAP Solar P1: Bluetooth CoC observer on the DMC packet-log firmware

Experimental port of the Bluetooth LE CoC observer firmware from
[throny/meshcore-ble-observer](https://github.com/throny/meshcore-ble-observer)
(`v1.18.0-coc`, MIT) onto `dmc-dev-packetlog` (MeshCore 1.18.0 plus the DMC packet
filter, `dc.gate` and region rules). The repeater sends its packet log over
Bluetooth to a nearby Linux host, which forwards it to MQTT with
[meshcoretomqtt](https://github.com/throny/meshcoretomqtt/tree/feature/repeater-observer-coc-20261029)
(`coc` transport). The DMC filter keeps working because it only decides what the
repeater forwards; the observer log is written before that decision.

**Status:** it builds, and the native tests pass (352, including the 89 observer
tests). It has **not** been tested on hardware yet.

## Environment

`SenseCap_Solar_repeater_observer` in `variants/sensecap_solar/platformio.ini`.
It sets `SENSECAP_OBSERVER_TRANSPORT`, `SENSECAP_OBSERVER_COC_CHANNELS=2`
and `CONFIG_MAX_KEYLEN=19`. Without `SENSECAP_OBSERVER_COC_CHANNELS` the build
still succeeds but is the non-Bluetooth variant (it reports `-obs544`).

## Build

1. Install the PlatformIO packages: `pio pkg install -e SenseCap_Solar_repeater_observer`.
2. Patch the pinned Adafruit framework once. **Do not skip this:** without it the
   firmware compiles, but the Bluetooth channels are never configured.

   ```sh
   patch -p1 -d ~/.platformio/packages/framework-arduinoadafruitnrf52 \
     < variants/sensecap_solar/observer-bluefruit-pre-enable.patch
   ```

   The framework file `libraries/Bluefruit52Lib/src/bluefruit.cpp` should have
   SHA-256 `e0948ab89ce44748e26d5fd950caa9bafb00da78e16fcc6e77f84a38cfd798f6`
   before patching (the framework is pinned to the same commit as upstream 1.18.0).
3. `pio run -e SenseCap_Solar_repeater_observer`, then convert with
   `python3 bin/uf2conv/uf2conv.py .pio/build/SenseCap_Solar_repeater_observer/firmware.hex -c -o firmware.uf2 -f 0xADA52840`.
4. A good build reports `v1.18.0.01-coc` and defines `bluefruit_configure_peripheral`.

Native tests: `pio test -e native`.

## What differs from throny's v1.18.0 patch stack

Patches 0001 to 0008 and `selected-544-runtime.patch` apply to `dmc-dev-packetlog`
with three hunks resolved by hand in `examples/simple_repeater/MyMesh.h` and
`MyMesh.cpp` (an include, the version string, and the `start ota` branch in
`handleCommand`). Two test-only changes were needed: `CONFIG_MAX_KEYLEN=19` in
`[env:native]`, and the shared `File`/`FsMock` test stubs are skipped when the
observer test brings its own (`OBSERVER_TEST_OWN_FILESYSTEM`).

## Things to know

- The observer replaces the repeater's host for Bluetooth. Never run
  `observer enroll` just to troubleshoot: it replaces the enrolled host and needs
  the USB console.
- `filter ...` and `dc.gate` commands are admin or USB only. The host may only
  send the time sync and the fixed read-only commands in throny's allowlist. The filter statistics that
  the ESP32 MQTT observer publishes are not published by this firmware.
- Rolling back: flash throny's `v1.18.0-coc.uf2` again.
- DMC syncs with upstream often; expect to redo the three `MyMesh` hunks on a new
  DMC release.
