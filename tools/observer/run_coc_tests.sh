#!/usr/bin/env bash
# Builds and runs the standalone observer CoC tests (test/test_observer_coc),
# cross-compiles the CoC core for Cortex-M4, and checks the framework patch
# against the pinned framework source. Never touches Bluetooth or hardware.
#
# Needs g++ and the PlatformIO packages of the observer environment:
#   pio pkg install -e SenseCap_Solar_repeater_observer
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
packages="${PLATFORMIO_PACKAGES_DIR:-${PLATFORMIO_CORE_DIR:-$HOME/.platformio}/packages}"
framework="$packages/framework-arduinoadafruitnrf52"
cmsis="$packages/framework-cmsis/CMSIS/Core/Include"
arm_cxx="$packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-g++"
arm_nm="$packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
bluefruit="libraries/Bluefruit52Lib/src/bluefruit.cpp"
patch_file="$root/variants/sensecap_solar/observer-bluefruit-pre-enable.patch"
# sha256 of the unmodified framework file the patch was written against.
framework_original_sha256="e0948ab89ce44748e26d5fd950caa9bafb00da78e16fcc6e77f84a38cfd798f6"

for path in "$framework/$bluefruit" "$cmsis" "$arm_cxx"; do
  [ -e "$path" ] || { echo "missing: $path (run pio pkg install -e SenseCap_Solar_repeater_observer)" >&2; exit 1; }
done

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
passed=0
ok() { echo "ok: $*"; passed=$((passed + 1)); }
fail() { echo "FAIL: $*" >&2; exit 1; }

warnings=(-std=c++11 -Wall -Wextra -Werror -Wno-error=reorder -I "$root/src")
nrf=(-DNRF52840_XXAA
     -isystem "$framework/cores/nRF5/nordic/softdevice/s140_nrf52_7.3.0_API/include"
     -isystem "$framework/cores/nRF5/nordic/nrfx/mdk"
     -isystem "$cmsis")

# 1. Standalone host tests. Each is a program that asserts and prints PASS.
host_test() {
  local name="$1"; shift
  g++ "${warnings[@]}" "$@" "$root/test/test_observer_coc/$name.cpp" -o "$work/$name"
  local out
  out="$("$work/$name")" || fail "$name exited non-zero: $out"
  grep -q PASS <<<"$out" || fail "$name did not report PASS: $out"
  ok "$name: $out"
}
host_test test_protocol
host_test test_delivery -DCOC_QUEUE_LIMIT=544
host_test test_server -DSVCALL_AS_NORMAL_FUNCTION "${nrf[@]}"
host_test test_nordic_port -DSVCALL_AS_NORMAL_FUNCTION "${nrf[@]}"

# 2. Cortex-M4 objects: the core builds for the target, and S140Config only
#    exports the framework hook when CoC channels are configured (1 or 2).
arm=("$arm_cxx" "${warnings[@]}" -mcpu=cortex-m4 -mthumb "${nrf[@]}")
"${arm[@]}" -c "$root/test/test_observer_coc/test_protocol.cpp" -o "$work/protocol.o"
"${arm[@]}" -DCOC_QUEUE_LIMIT=256 -c "$root/test/test_observer_coc/test_delivery.cpp" -o "$work/delivery.o"
ok "protocol and delivery compile for Cortex-M4"
for channels in none 1 2; do
  flags=()
  [ "$channels" = none ] || flags=(-DSENSECAP_OBSERVER_COC_CHANNELS="$channels")
  "${arm[@]}" "${flags[@]}" -c "$root/src/observer/coc/S140Config.cpp" -o "$work/s140.o"
  if "$arm_nm" --defined-only "$work/s140.o" | grep -q bluefruit_configure_peripheral; then
    [ "$channels" != none ] || fail "S140Config exports the hook without CoC channels"
  else
    [ "$channels" = none ] || fail "S140Config lacks the hook with $channels channel(s)"
  fi
done
if "${arm[@]}" -DSENSECAP_OBSERVER_COC_CHANNELS=3 -c "$root/src/observer/coc/S140Config.cpp" \
     -o "$work/invalid.o" 2>"$work/invalid.log"; then
  fail "S140Config accepted 3 channels"
fi
grep -q "one or two channels" "$work/invalid.log" || fail "unexpected error for 3 channels"
ok "S140Config hook export and channel-count guard"

# 3. Firmware version follows the CoC channel setting unless set explicitly.
version_of() {
  sed -n '/^#ifndef FIRMWARE_VERSION/,/^#endif/p' "$root/examples/simple_repeater/MyMesh.h" \
    | { cat; echo FIRMWARE_VERSION; } | g++ -E -P -x c++ "$@" - | tr -d '[:space:]'
}
[ "$(version_of)" = '"v1.18.0.1-obs544-dev"' ] || fail "default version"
[ "$(version_of -DSENSECAP_OBSERVER_COC_CHANNELS=0)" = '"v1.18.0.1-obs544-dev"' ] || fail "version with 0 channels"
[ "$(version_of -DSENSECAP_OBSERVER_COC_CHANNELS=1)" = '"v1.18.0.1-coc-dev"' ] || fail "version with 1 channel"
[ "$(version_of -DSENSECAP_OBSERVER_COC_CHANNELS=2)" = '"v1.18.0.1-coc-dev"' ] || fail "version with 2 channels"
[ "$(version_of '-DFIRMWARE_VERSION="custom"')" = '"custom"' ] || fail "explicit version"
[ "$(version_of '-DFIRMWARE_VERSION="v1.18.0.1-coc-dutchmeshcore.nl-abc1234"')" = '"v1.18.0.1-coc-dutchmeshcore.nl-abc1234"' ] || fail "build.sh version"
ok "firmware version selection"

# 4. Event dispatch invariants: one Bluefruit.begin and one event callback,
#    with the legacy handler running before the observer handler.
runtime="$root/src/observer/ObserverBleRuntime.cpp"
[ "$(grep -c 'Bluefruit.begin(' "$runtime")" = 1 ] || fail "Bluefruit.begin must appear once"
[ "$(grep -c 'Bluefruit.setEventCallback(' "$runtime")" = 1 ] || fail "setEventCallback must appear once"
dispatch="$(sed -n '/void dispatch(ble_evt_t\* event)/,/^}/p' "$runtime")"
legacy_line="$(grep -n 'legacy(event)' <<<"$dispatch" | cut -d: -f1)"
observer_line="$(grep -n 'observer(event)' <<<"$dispatch" | cut -d: -f1)"
[ -n "$legacy_line" ] && [ -n "$observer_line" ] && [ "$legacy_line" -lt "$observer_line" ] \
  || fail "dispatch must call legacy before observer"
ok "single BLE runtime owner and dispatch order"

# 5. The framework patch applies to the pinned original and calls the hook
#    before sd_ble_enable. Works whether or not the installed copy is patched.
mkdir -p "$work/fw/$(dirname "$bluefruit")"
cp "$framework/$bluefruit" "$work/fw/$bluefruit"
if [ "$(sha256sum < "$work/fw/$bluefruit" | cut -d' ' -f1)" != "$framework_original_sha256" ]; then
  patch -s -R -p1 -d "$work/fw" < "$patch_file" || fail "installed framework is neither pinned nor pinned+patch"
  [ "$(sha256sum < "$work/fw/$bluefruit" | cut -d' ' -f1)" = "$framework_original_sha256" ] \
    || fail "installed framework does not match the pinned original"
  echo "note: installed framework already carries the patch"
fi
patch -s -p1 -d "$work/fw" < "$patch_file"
call="bluefruit_configure_peripheral(CONN_CFG_PERIPHERAL, ram_start)"
call_line="$(grep -nF "VERIFY_STATUS ( $call, false )" "$work/fw/$bluefruit" | cut -d: -f1)"
enable_line="$(grep -nF 'sd_ble_enable(&ram_start)' "$work/fw/$bluefruit" | head -1 | cut -d: -f1)"
[ -n "$call_line" ] && [ -n "$enable_line" ] && [ "$call_line" -lt "$enable_line" ] \
  || fail "patched framework must call the hook before sd_ble_enable"
grep -qF "if (bluefruit_configure_peripheral)" "$work/fw/$bluefruit" || fail "hook must stay optional"
ok "framework patch applies to the pinned original"

echo "$passed checks passed"
