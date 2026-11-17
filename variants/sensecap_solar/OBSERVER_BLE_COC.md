# SenseCAP Solar P1: Bluetooth CoC observer on the DMC packet-log firmware

Experimental port of the Bluetooth LE CoC observer firmware by throny
([throny/MeshCore, branch observer-coc](https://github.com/throny/MeshCore/tree/observer-coc),
tag `v1.18.0-coc.2`, MIT; it started as
[throny/meshcore-ble-observer](https://github.com/throny/meshcore-ble-observer),
now archived) onto `dmc-dev-packetlog` at DMC v1.18.0.1 (MeshCore 1.18.0 plus the DMC
packet filter, `dc.gate` and region rules). The repeater sends its packet log over
Bluetooth to a nearby Linux host, which forwards it to MQTT with
[meshcoretomqtt](https://github.com/throny/meshcoretomqtt) (`coc` transport, see its
`docs/coc.md`). The DMC filter keeps working because it only decides what the
repeater forwards; the observer log is written before that decision.

**Status:** it builds, and the native tests pass (354 in `native`, 95 in
`native_legacy_keys`). It has **not** been tested on hardware yet, and neither has
throny's `v1.18.0-coc.2`.

## Environment

`SenseCap_Solar_repeater_observer` in `variants/sensecap_solar/platformio.ini`.
It sets `SENSECAP_OBSERVER_TRANSPORT`, `SENSECAP_OBSERVER_COC_CHANNELS=2`,
`CONFIG_MAX_KEYLEN=19` and `OBSERVER_LEGACY_PREF_KEYS=1`. The last two only exist to
migrate a device that was enrolled with the older firmware (long `observer_peer_*`
key names): on first boot the old names are read and replaced by the short
`observer_p_*` names, so the enrollment is kept. Remove both flags once every such
device has booted this firmware once.

Without `SENSECAP_OBSERVER_COC_CHANNELS` the build still succeeds but is the
non-Bluetooth variant (it reports `-obs544`).

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
3. Build through DMC's `build.sh`, so the version carries the DMC tag
   (`<version>-<channel>-dutchmeshcore.nl-<hash>`):

   ```sh
   FIRMWARE_VERSION=v1.18.0.1 DMC_CHANNEL=coc sh build.sh build-firmware SenseCap_Solar_repeater_observer
   ```

   The UF2 lands in `out/`. A plain `pio run -e SenseCap_Solar_repeater_observer`
   also works, but reports `v1.18.0.1-coc-dev` (no DMC tag).
4. A good build reports `v1.18.0.1-coc-dutchmeshcore.nl-<hash>` on `ver` and defines
   `bluefruit_configure_peripheral`.

Tests: `pio test -e native`, `pio test -e native_legacy_keys`, and
`tools/observer/run_coc_tests.sh` (the standalone CoC tests, a Cortex-M4 compile check
and a check of the framework patch; needs the packages from step 1).

## What differs from throny's observer-coc

The observer sources (`src/observer/`, the tests) are throny's `v1.18.0-coc.2`
unchanged. The integration into the repeater (`examples/simple_repeater/main.cpp`,
`MyMesh.h`, `MyMesh.cpp`, `src/Dispatcher.cpp`, `SerialBLEInterface.cpp`) is rebased
onto the DMC repeater, which moved its UITask code to `src/helpers/ui/server`.
Other differences: the repeater version string follows DMC's format (see Build); the
shared `File`/`FsMock` test stubs are skipped when the observer test brings its own
(`OBSERVER_TEST_OWN_FILESYSTEM`); and throny's release scripts are not included.

## Things to know

- The observer replaces the repeater's host for Bluetooth. Never run
  `observer enroll` just to troubleshoot: it replaces the enrolled host and needs
  the USB console.
- `filter ...` and `dc.gate` commands are admin or USB only. The host may only
  send the time sync and the fixed read-only commands in throny's allowlist. The filter statistics that
  the ESP32 MQTT observer publishes are not published by this firmware.
- Rolling back: flash the previous UF2 (throny's `v1.18.0-coc.uf2` or the earlier
  DMC-based build) or DMC's own `SenseCap_Solar_repeater-v1.18.0.1-packetlog-*.uf2`
  (that one has no observer). After this firmware has booted once, the enrollment
  is saved under the new `observer_p_*` names, which the older firmware does not
  read. Going back to it will most likely show `enrolled=0`, so the host has to be
  enrolled again (untested).
- DMC syncs with upstream often; expect to redo the `MyMesh` and `main.cpp` hunks on a
  new DMC release.
