#include "ObserverTransport.h"
#if !defined(OBSERVER_TRANSPORT_PLATFORM_TEST)
#include <utility/bonding.h>
#endif
#include <new>
#include <algorithm>
#include <stdio.h>

#include "ObserverPreference.h"

#if defined(SENSECAP_OBSERVER_TRANSPORT)

namespace observer {

namespace {
constexpr uint8_t kServiceUuid[] = {0x1b, 0x5f, 0x6f, 0x02, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
constexpr uint8_t kLogUuid[] = {0x1b, 0x5f, 0x6f, 0x03, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
constexpr uint8_t kCommandUuid[] = {0x1b, 0x5f, 0x6f, 0x04, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
constexpr uint8_t kAckUuid[] = {0x1b, 0x5f, 0x6f, 0x05, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
constexpr uint8_t kStatsUuid[] = {0x1b, 0x5f, 0x6f, 0x06, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
constexpr uint8_t kCocDiscoveryUuid[] = {0x1b, 0x5f, 0x6f, 0x07, 0x6b, 0x56, 0x48, 0x5e, 0xa3, 0x50, 0x61, 0x3c, 0x4a, 0x5d, 0xc2, 0x10};
constexpr uint16_t kCocDescriptorBytes = 8;
#endif
constexpr uint16_t kAdvertisingInterval = 1600;  // 1 second
constexpr uint16_t kNotificationBytes = 244;
}

Transport* Transport::instance_ = nullptr;
Transport transport(false);
LogMirror log_mirror;

Transport::Transport(bool enabled)
    : enabled_(enabled), policy_(enabled),
      service_(BLEUuid(kServiceUuid)), log_characteristic_(BLEUuid(kLogUuid)),
      command_characteristic_(BLEUuid(kCommandUuid)), ack_characteristic_(BLEUuid(kAckUuid)),
      stats_characteristic_(BLEUuid(kStatsUuid))
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
      , discovery_characteristic_(BLEUuid(kCocDiscoveryUuid))
#endif
      {}

void Transport::attachPreferences(bool enabled, const PeerIdentity& peer, bool has_peer) {
  enabled_ = enabled;
  policy_ = Policy(enabled, peer, has_peer);
}

void Transport::setPreferenceCallbacks(bool (*save_enabled)(bool),
                                       bool (*save_peer)(bool, const PeerIdentity&)) {
  save_enabled_ = save_enabled;
  save_peer_ = save_peer;
}

bool Transport::begin(const char* name, bool enabled) {
  if (!enabled || !save_enabled_) return false;
  if (initialization_failed_) return failInitialization();
  enabled_ = true;
  policy_.setEnabled(true);
  if (initialized_) return true;
  instance_ = this;
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  coc_runtime::setObserverCallback(onCoCBLEEvent);
#else
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
#endif
  // Suppress normal Bluetooth indication only; bootloader LEDs are independent.
  Bluefruit.autoConnLed(false);
  digitalWrite(LED_BLUE, 1 - LED_STATE_ON);
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  if (!coc_runtime::beginBluefruitOnce()) return failInitialization();
#else
  if (!Bluefruit.begin(1, 0)) return failInitialization();
#endif
  core_ready_ = true;
  Bluefruit.setTxPower(0);
  Bluefruit.setName(name);
  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setIOCaps(true, false, false);
  Bluefruit.Security.setPairPasskeyCallback(onPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairComplete);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Security.setSecuredCallback(onSecured);
  if (service_.begin() != NRF_SUCCESS) return failInitialization();
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  discovery_characteristic_.setProperties(CHR_PROPS_READ);
  discovery_characteristic_.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  discovery_characteristic_.setMaxLen(kCocDescriptorBytes);
  discovery_characteristic_.setReadAuthorizeCallback(onDiscovery);
  if (discovery_characteristic_.begin() != NRF_SUCCESS) return failInitialization();
#else
  log_characteristic_.setProperties(CHR_PROPS_NOTIFY);
  log_characteristic_.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  log_characteristic_.setMaxLen(kNotificationBytes);
  if (log_characteristic_.begin() != NRF_SUCCESS) return failInitialization();
  command_characteristic_.setProperties(CHR_PROPS_WRITE);
  command_characteristic_.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_WITH_MITM);
  command_characteristic_.setMaxLen(kCommandBytes - 1);
  command_characteristic_.setWriteCallback(onCommand);
  if (command_characteristic_.begin() != NRF_SUCCESS) return failInitialization();
  ack_characteristic_.setProperties(CHR_PROPS_WRITE);
  ack_characteristic_.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_WITH_MITM);
  ack_characteristic_.setMaxLen(kCommandBytes - 1);
  ack_characteristic_.setWriteCallback(onAck);
  if (ack_characteristic_.begin() != NRF_SUCCESS) return failInitialization();
  stats_characteristic_.setProperties(CHR_PROPS_READ);
  stats_characteristic_.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  stats_characteristic_.setMaxLen(128);
  stats_characteristic_.setReadAuthorizeCallback(onStats);
  if (stats_characteristic_.begin() != NRF_SUCCESS) return failInitialization();
#endif
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(service_);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(kAdvertisingInterval, kAdvertisingInterval);
  Bluefruit.Advertising.restartOnDisconnect(false);
  initialized_ = true;
  // Accepted opt-in, not entropy readiness. The loop acquires entropy before
  // allocating capture storage or making the peripheral discoverable.
  return true;
}

bool Transport::beginFromPreferences(const char* name) {
  if (!enabled_) return false;
  return begin(name, true);
}

bool Transport::failInitialization() {
  // Bluefruit has no safe end()/partial-GATT rollback API. Fail closed and
  // require reboot instead of registering duplicate services on another opt-in.
  initialization_failed_ = true;
  initialized_ = false;
  enabled_ = false;
  policy_.setEnabled(false);
  policy_.closeEnrollment();
  onboarding_ = false;
  if (core_ready_) Bluefruit.Advertising.stop();
  releaseQueue();
  if (save_enabled_) save_enabled_(false);
  return false;
}

void Transport::serviceAdvertising() {
  const bool discoverable = enabled_ && initialized_ && nonce_ready_ && log_queue_ &&
      (policy_.hasPeer() || policy_.enrollmentOpen(millis()));
  if (!discoverable) {
    if (Bluefruit.Advertising.isRunning()) Bluefruit.Advertising.stop();
  } else if (connectionHandle() == BLE_CONN_HANDLE_INVALID && !Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.start(0);
  }
}

bool Transport::authorizedAndSecured(uint16_t handle) const {
  BLEConnection* connection = Bluefruit.Connection(handle);
  if (!connection || !connection->connected() || !connection->bonded() || !connection->secured()) return false;
  const ble_gap_addr_t peer_address = connection->getPeerAddr();
  PeerIdentity identity{};
  identity.type = peer_address.addr_type;
  memcpy(identity.address, peer_address.addr, sizeof(identity.address));
  const bool allowed = policy_.authorizes(true, identity, millis());
  if (!allowed && !policy_.enrollmentOpen(millis())) connection->disconnect();
  return allowed;
}

bool Transport::authorizeCoC(uint16_t handle) const {
  return handle == connectionHandle() && authorizedAndSecured(handle);
}

bool Transport::authorizeCoCLink(uint16_t handle) {
  return transport.authorizeCoC(handle);
}

void Transport::setConnectionHandle(uint16_t handle) {
  __atomic_store_n(&connection_handle_, handle, __ATOMIC_RELEASE);
}

uint16_t Transport::connectionHandle() const {
  return __atomic_load_n(&connection_handle_, __ATOMIC_ACQUIRE);
}

bool Transport::refreshNonce(bool rotate) {
  if (nonce_ready_ && !rotate) return true;
  if (!core_ready_) return false;
  const uint32_t now = millis();
  if (entropy_attempted_ && static_cast<uint32_t>(now - entropy_attempt_at_) < kEntropyRetryMs) return false;
  entropy_attempted_ = true;
  entropy_attempt_at_ = now;
  uint8_t available = 0;
  if (sd_rand_application_bytes_available_get(&available) != NRF_SUCCESS || available < 8) return false;
  uint8_t bytes[8]{};
  if (sd_rand_application_vector_get(bytes, sizeof(bytes)) != NRF_SUCCESS) return false;
  uint64_t nonce = 0;
  for (unsigned index = 0; index < sizeof(bytes); ++index) nonce |= static_cast<uint64_t>(bytes[index]) << (index * 8);
  if (nonce == 0 || nonce == boot_nonce_) return false;
  boot_nonce_ = nonce;
  nonce_ready_ = true;
  return true;
}

bool Transport::enable(bool enabled) {
  if (!save_enabled_ || !save_enabled_(enabled)) return false;
  if (!initialized_) {
    if (!enabled) { enabled_ = false; policy_.setEnabled(false); return true; }
    enabled_ = true;
    policy_.setEnabled(true);
    if (!begin("SenseCAP Observer", true)) return false;
    return true;
  }
  enabled_ = enabled;
  policy_.setEnabled(enabled);

  if (!enabled) {
    closeEnrollment();
    __atomic_store_n(&passkey_allowed_handle_, BLE_CONN_HANDLE_INVALID, __ATOMIC_RELEASE);
    __atomic_store_n(&passkey_pending_, false, __ATOMIC_RELEASE);
    disable_pending_ = true;
    enabled_ = false;
    policy_.setEnabled(false);
    Bluefruit.Advertising.stop();
    if (connectionHandle() != BLE_CONN_HANDLE_INVALID) {
      BLEConnection* connection = Bluefruit.Connection(connectionHandle());
      if (connection) connection->disconnect();
    }
    __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
    return true;
  }
  disable_pending_ = false;
  policy_.setEnabled(true);
  return true;
}

bool Transport::revokePeer() {
  if (!policy_.hasPeer()) return true;
  ble_gap_addr_t address{};
  address.addr_type = policy_.peer().type;
  memcpy(address.addr, policy_.peer().address, sizeof(address.addr));
  if (!persistPeer(false, PeerIdentity{})) return false;
  bond_remove_key(BLE_GAP_ROLE_PERIPH, &address);
  policy_.revoke();
  if (connectionHandle() != BLE_CONN_HANDLE_INVALID) {
    BLEConnection* connection = Bluefruit.Connection(connectionHandle());
    if (connection) connection->disconnect();
  }
  return true;
}

bool Transport::beginEnrollment() {
  if (!enabled_ || !initialized_ || !nonce_ready_ || !log_queue_ || !Serial) return false;
  if (connectionHandle() != BLE_CONN_HANDLE_INVALID) {
    BLEConnection* connection = Bluefruit.Connection(connectionHandle());
    if (connection) connection->disconnect();
    __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
  }
  if (policy_.hasPeer()) {
    if (!revokePeer()) return false;
    policy_.setEnabled(true);
  }
  onboarding_ = true;
  enrollment_handle_ = BLE_CONN_HANDLE_INVALID;
  enrollment_started_ = millis();
  policy_.openEnrollment(enrollment_started_, kEnrollmentWindowMs);
  if (!policy_.enrollmentOpen(enrollment_started_)) { onboarding_ = false; return false; }
  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setIOCaps(true, false, false);
  Bluefruit.Security.setPairPasskeyCallback(onPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairComplete);
  serviceAdvertising();
  Serial.println("Observer BLE enrollment open for 60 seconds; pair the intended Pi now.");
  return true;
}

bool Transport::processAck(const char* bytes, size_t length) {
  Ack ack{};
  if (!parseAck(bytes, length, ack)) return false;
  const bool acknowledged = log_queue_ && log_queue_->acknowledge(ack.boot_nonce, ack.sequence);
  if (acknowledged) {
    delivery_.acknowledged();
    __atomic_store_n(&complete_record_sent_, false, __ATOMIC_RELEASE);
  }
  return acknowledged;
}

void Transport::onConnect(uint16_t handle) {
  if (instance_) instance_->callback_events_.postConnect(handle);
}

void Transport::onDisconnect(uint16_t handle, uint8_t) {
  if (instance_) instance_->callback_events_.postDisconnect(handle);
}

void Transport::onSecured(uint16_t handle) {
  if (instance_) instance_->callback_events_.postSecured(handle);
}

void Transport::onCommand(uint16_t handle, BLECharacteristic*, uint8_t* data, uint16_t length) {
  if (!instance_ || !__atomic_load_n(&instance_->secured_, __ATOMIC_ACQUIRE) || !instance_->authorizedAndSecured(handle) || length == 0 ||
      length >= kCommandBytes) return;
  if (length >= 4 && memcmp(data, "ack ", 4) == 0) return;
  instance_->command_ingress_.submitCommand(reinterpret_cast<const char*>(data), length);
}

void Transport::onAck(uint16_t handle, BLECharacteristic*, uint8_t* data, uint16_t length) {
  if (!instance_ || !__atomic_load_n(&instance_->secured_, __ATOMIC_ACQUIRE) || !instance_->authorizedAndSecured(handle) || !length ||
      length >= kCommandBytes) return;
  instance_->ack_ingress_.submit(reinterpret_cast<const char*>(data), length);
}

void Transport::onStats(uint16_t handle, BLECharacteristic*, ble_gatts_evt_read_t* request) {
  if (!instance_ || !instance_->authorizedAndSecured(handle)) return;
  char buffer[128]{};
  const StatsValues stats = instance_->stats_snapshot_.read();
  const uint16_t length = static_cast<uint16_t>(formatStats(stats, buffer, sizeof(buffer)));
  if (request->offset > length) return;
  ble_gatts_rw_authorize_reply_params_t reply{};
  reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
  reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
  reply.params.read.update = 1;
  reply.params.read.len = length - request->offset;
  reply.params.read.offset = request->offset;
  reply.params.read.p_data = reinterpret_cast<uint8_t*>(buffer + request->offset);
  sd_ble_gatts_rw_authorize_reply(handle, &reply);
}

#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
void Transport::onCoCBLEEvent(ble_evt_t* event) {
  if (instance_ && event) instance_->coc_server_.event(*event);
}

void Transport::onDiscovery(uint16_t handle, BLECharacteristic*, ble_gatts_evt_read_t* request) {
  if (!instance_ || !request || !instance_->authorizeCoC(handle)) return;
  uint8_t descriptor[kCocDescriptorBytes] = {
      1, static_cast<uint8_t>(SENSECAP_OBSERVER_COC_CHANNELS),
      static_cast<uint8_t>(coc::Server::kDataPsm), static_cast<uint8_t>(coc::Server::kDataPsm >> 8),
      static_cast<uint8_t>(SENSECAP_OBSERVER_COC_CHANNELS == 2 ? coc::Server::kControlPsm : 0),
      static_cast<uint8_t>(SENSECAP_OBSERVER_COC_CHANNELS == 2 ? coc::Server::kControlPsm >> 8 : 0),
      static_cast<uint8_t>(coc::kSduBytes), static_cast<uint8_t>(coc::kSduBytes >> 8)};
  if (request->offset > sizeof(descriptor)) return;
  ble_gatts_rw_authorize_reply_params_t reply{};
  reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
  reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
  reply.params.read.update = 1;
  reply.params.read.len = sizeof(descriptor) - request->offset;
  reply.params.read.offset = request->offset;
  reply.params.read.p_data = descriptor + request->offset;
  (void)sd_ble_gatts_rw_authorize_reply(handle, &reply);
}

size_t Transport::cocStats(char* bytes, size_t capacity) {
  if (!instance_ || !bytes || !capacity) return 0;
  return formatStats(instance_->stats_snapshot_.read(), bytes, capacity);
}

void Transport::cocTimerCallback(TimerHandle_t) {}

void Transport::cocWake() {
  if (!instance_ || !instance_->coc_timer_initialized_) return;
  instance_->coc_timer_.setPeriod(1);
  instance_->coc_timer_.start();
}

void Transport::cocSchedule(uint32_t delay_ms, bool enabled) {
  if (!instance_) return;
  Transport& t = *instance_;
  if (!t.coc_timer_initialized_) {
    t.coc_timer_.begin(1, cocTimerCallback, &t, false);
    t.coc_timer_initialized_ = t.coc_timer_.getHandle() != nullptr;
  }
  if (!t.coc_timer_initialized_) return;
  if (!enabled) {
    t.coc_timer_.stop();
    return;
  }
  t.coc_timer_.setPeriod(delay_ms ? delay_ms : 1);
  t.coc_timer_.start();
}
#endif

bool Transport::onPasskey(uint16_t handle, uint8_t const passkey[6], bool match_request) {
  if (!instance_ || match_request ||
      __atomic_load_n(&instance_->passkey_allowed_handle_, __ATOMIC_ACQUIRE) != handle) return false;
  memcpy(instance_->pending_passkey_, passkey, sizeof(instance_->pending_passkey_));
  __atomic_store_n(&instance_->passkey_state_, 1, __ATOMIC_RELEASE);
  __atomic_store_n(&instance_->passkey_pending_, true, __ATOMIC_RELEASE);
  return true;
}

void Transport::onPairComplete(uint16_t handle, uint8_t auth_status) {
  if (instance_) instance_->callback_events_.postPairComplete(handle, auth_status);
}

bool Transport::takeCommand(char* destination, size_t capacity) {
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  return coc_server_.takeCommand(destination, capacity);
#else
  return command_ingress_.takeCommand(destination, capacity);
#endif
}

#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
bool Transport::replyCoC(const uint8_t* bytes, size_t length) {
  return coc_server_.reply(bytes, length, static_cast<uint32_t>(millis()));
}
#endif

size_t Transport::writeLog(const uint8_t* bytes, size_t length) {
  if (!enabled_) return length;
  for (size_t index = 0; index < length; ++index) {
    if (!line_buffer_ || unavailable_line_bytes_ != 0) {
      // No allocation/entropy wait in the radio callback. If readiness changes
      // mid-line, discard its remaining tail rather than admit a partial RAW.
      ++unavailable_line_bytes_;
      if (bytes[index] == '\n') {
        if (log_queue_) log_queue_->rejectUnavailable(unavailable_line_bytes_);
        else {
          sequence_state_.skip();
          ++prequeue_unavailable_records_;
          prequeue_unavailable_bytes_ += unavailable_line_bytes_;
        }
        unavailable_line_bytes_ = 0;
      }
    } else line_buffer_->write(bytes[index]);
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
    if (bytes[index] == '\n') cocWake();
#endif
  }
  return length;
}

void Transport::service() {
  if (!initialized_) return;
  uint16_t event_handle = BLE_CONN_HANDLE_INVALID;
  uint8_t event_status = 0;
  if (callback_events_.takeDisconnect(event_handle) && connectionHandle() == event_handle) {
    setConnectionHandle(BLE_CONN_HANDLE_INVALID);
    admission_.disconnected(event_handle);
    __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
    delivery_.disconnected();
    __atomic_store_n(&complete_record_sent_, false, __ATOMIC_RELEASE);
    if (enrollment_handle_ == event_handle) enrollment_handle_ = BLE_CONN_HANDLE_INVALID;
    command_ingress_.clear();
    ack_length_ = 0;
    if (log_queue_) log_queue_->restartHead();
  }
  if (callback_events_.takeConnect(event_handle)) {
    BLEConnection* connection = Bluefruit.Connection(event_handle);
    const bool enrolling = onboarding_ && policy_.enrollmentOpen(millis());
    bool identity_known = false;
    PeerIdentity identity{};
    if (connection) {
      const ble_gap_addr_t address = connection->getPeerAddr();
      identity.type = address.addr_type;
      memcpy(identity.address, address.addr, sizeof(identity.address));
      // getPeerAddr() yields the persisted bond identity once the bond has loaded.
      // Stable public/static addresses and S140-resolved RPAs are also comparable
      // before slot assignment. Unresolved private addresses are held only under the
      // bounded timeout and cannot access data or commands before bonded authorization.
      identity_known = identityKnownAtConnect(connection->bonded(), address.addr_type, address.addr_id_peer);
    }
    const bool accepted = admitsCandidate(
        enabled_, connectionHandle() == BLE_CONN_HANDLE_INVALID,
        hasLiveConnection(connection != nullptr, connection && connection->connected()),
        enrolling, enrollment_handle_ == BLE_CONN_HANDLE_INVALID,
        policy_.hasPeer(), identity_known, identity, policy_.peer());
    if (!accepted) {
      if (connection) connection->disconnect();
    } else {
      setConnectionHandle(event_handle);
      __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
      connected_at_ = millis();
      admission_.connected(event_handle, connected_at_);
      if (enrolling) {
        enrollment_handle_ = event_handle;
        __atomic_store_n(&passkey_allowed_handle_, event_handle, __ATOMIC_RELEASE);
      }
    }
  }
  if (admission_.pending() && !policy_.enrollmentOpen(millis())) {
    const uint16_t handle = connectionHandle();
    BLEConnection* connection = Bluefruit.Connection(handle);
    if (connection && connection->bonded() && connection->secured() && policy_.hasPeer()) {
      const ble_gap_addr_t address = connection->getPeerAddr();
      PeerIdentity identity{};
      identity.type = address.addr_type;
      memcpy(identity.address, address.addr, sizeof(identity.address));
      if (!admission_.authorize(handle, identity, policy_.peer())) connection->disconnect();
    }
    uint16_t expired_handle = BLE_CONN_HANDLE_INVALID;
    if (admission_.takeDisconnect(millis(), expired_handle)) {
      BLEConnection* expired_connection = Bluefruit.Connection(expired_handle);
      if (expired_connection) expired_connection->disconnect();
    }
  }
  if (disable_pending_ && callback_events_.safeToReleaseQueue(enabled_, connectionHandle(), admission_.pending())) {
    if (line_buffer_) line_buffer_->reset();
    releaseQueue();
    __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
    disable_pending_ = false;
  }
  if (!enabled_) return;
  if (onboarding_ && (!Serial || static_cast<uint32_t>(millis() - enrollment_started_) >= kEnrollmentWindowMs)) {
    closeEnrollment();
  }
  // The loop is the sole snapshot writer; the GATT callback reads only this copy.
  updateStatsSnapshot();
  if (__atomic_exchange_n(&passkey_pending_, false, __ATOMIC_ACQ_REL) &&
      Serial && onboarding_ && policy_.enrollmentOpen(millis())) {
    Serial.print("Observer BLE passkey: ");
    if (__atomic_load_n(&passkey_state_, __ATOMIC_ACQUIRE) == 1) {
      Serial.write(pending_passkey_, sizeof(pending_passkey_));
      __atomic_store_n(&passkey_state_, 0, __ATOMIC_RELEASE);
    }
    Serial.println();
  }
  if (callback_events_.takePairComplete(event_handle, event_status) && onboarding_ &&
      event_handle == enrollment_handle_ && policy_.enrollmentOpen(millis())) {
    pair_complete_handle_ = event_handle;
    pair_complete_status_ = event_status;
    __atomic_store_n(&pair_complete_pending_, true, __ATOMIC_RELEASE);
  }
  if (pair_complete_pending_) {
    if (!__atomic_exchange_n(&pair_complete_pending_, false, __ATOMIC_ACQ_REL)) return;
    const uint16_t handle = pair_complete_handle_;
    const uint8_t status = pair_complete_status_;
    if (status == BLE_GAP_SEC_STATUS_SUCCESS && onboarding_ &&
        enrollment_handle_ == handle && policy_.enrollmentOpen(millis())) {
      BLEConnection* connection = Bluefruit.Connection(handle);
      if (!connection || !connection->bonded()) {
        if (connection) connection->disconnect();
      } else {
        const ble_gap_addr_t address = connection->getPeerAddr();
        PeerIdentity identity{};
        identity.type = address.addr_type;
        memcpy(identity.address, address.addr, sizeof(identity.address));
        if (!policy_.enroll(identity, millis()) || !persistPeer(true, identity)) {
          if (save_peer_) save_peer_(false, PeerIdentity{});
          connection->removeBondKey();
          policy_.revoke();
          connection->disconnect();
        } else {
          onboarding_ = false;
          __atomic_store_n(&passkey_allowed_handle_, BLE_CONN_HANDLE_INVALID, __ATOMIC_RELEASE);
          __atomic_store_n(&passkey_pending_, false, __ATOMIC_RELEASE);
          const uint16_t completed_handle = enrollment_handle_;
          enrollment_handle_ = BLE_CONN_HANDLE_INVALID;
          Bluefruit.Advertising.stop();
          policy_.closeEnrollment();
          if (completed_handle == connectionHandle()) disable_pending_ = true;
          if (connection->secured()) {
            admission_.authorize(handle, identity, identity);
            __atomic_store_n(&secured_, true, __ATOMIC_RELEASE);
          }
          Serial.println("Observer BLE peer pinned; enrollment closed.");
        }
      }
    }
  }
  if (callback_events_.takeSecured(event_handle) && connectionHandle() == event_handle &&
      !policy_.enrollmentOpen(millis())) {
    const bool allowed = authorizedAndSecured(event_handle);
    __atomic_store_n(&secured_, allowed, __ATOMIC_RELEASE);
    if (allowed) {
      BLEConnection* connection = Bluefruit.Connection(event_handle);
      if (connection) {
        const ble_gap_addr_t address = connection->getPeerAddr();
        PeerIdentity identity{};
        identity.type = address.addr_type;
        memcpy(identity.address, address.addr, sizeof(identity.address));
        admission_.authorize(event_handle, identity, policy_.peer());
      }
    }
  }
  if (onboarding_ && static_cast<uint32_t>(millis() - enrollment_started_) >= kEnrollmentWindowMs) {
    closeEnrollment();
  }
  if (!nonce_ready_) {
    if (!refreshNonce()) return;
  }
  if (!log_queue_ && !allocateQueue()) { failInitialization(); return; }
  serviceAdvertising();
  // This loop is the only entropy/queue-session owner. Enqueue never calls
  // SoftDevice or waits; exhausted admission loss is counted until fresh bytes
  // are available. Already queued records retain their own session identities.
  if (log_queue_ && log_queue_->sequenceExhausted() && refreshNonce(true)) {
    log_queue_->setBootNonce(boot_nonce_);
  }
  if (policy_.enrollmentOpen(millis())) {
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
    coc_server_.close();
    coc_server_.service(log_queue_, boot_nonce_, static_cast<uint32_t>(millis()));
#endif
    return;
  }
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  if (!enabled_) {
    coc_server_.close();
    coc_server_.service(static_cast<Queue*>(nullptr), 0, static_cast<uint32_t>(millis()));
    return;
  }
  coc_server_.service(log_queue_, boot_nonce_, static_cast<uint32_t>(millis()));
  return;
#else
  if (!ack_length_) ack_ingress_.take(ack_, sizeof(ack_), ack_length_);
  if (ack_length_) { processAck(ack_, ack_length_); ack_length_ = 0; }
  const uint16_t handle = connectionHandle();
  if (!enabled_ || !nonce_ready_ || !log_queue_ || !__atomic_load_n(&secured_, __ATOMIC_ACQUIRE) || !authorizedAndSecured(handle) ||
      !log_characteristic_.notifyEnabled(handle) || log_queue_->empty()) return;
  BLEConnection* connection = Bluefruit.Connection(handle);
  if (!connection) return;
  const uint16_t mtu = connection->getMtu();
  if (mtu < 23) return;
  const size_t notification_bytes = std::min<size_t>(kNotificationBytes, mtu - 3);
  const unsigned long now = millis();
  if (!delivery_.maySend(now)) return;
  if (__atomic_load_n(&complete_record_sent_, __ATOMIC_ACQUIRE)) {
    log_queue_->restartHead();
    delivery_.restartWholeRecord();
    __atomic_store_n(&complete_record_sent_, false, __ATOMIC_RELEASE);
  }
  RecordFragment fragment;
  if (!log_queue_->peek(fragment, notification_bytes - kFrameHeaderBytes)) return;
  uint8_t notification[kNotificationBytes];
  const size_t encoded = encodeFrame(fragment.boot_nonce, fragment.sequence, fragment.total_length,
                                     fragment.offset, fragment.bytes, fragment.length,
                                     notification, sizeof(notification));
  if (!encoded) return;
  delivery_.attempted(now);
  if (log_characteristic_.notify(handle, notification, encoded)) {
    if (!log_queue_->consume(fragment.boot_nonce, fragment.sequence, fragment.length)) return;
    const bool record_complete = (fragment.offset + fragment.length) == fragment.total_length;
    __atomic_store_n(&complete_record_sent_, record_complete, __ATOMIC_RELEASE);
    delivery_.fragmentSent(record_complete);
  } else {
    // This <=20-byte value fits one ATT notification at the minimum MTU.
    // A rejected notify accepted no fragment: retain the unsent cursor rather
    // than replaying its accepted prefix. Disconnect and ACK timeout still
    // restart the whole record; consume remains success-only.
    ++failed_notifications_;
    delivery_.fragmentFailed();
    __atomic_store_n(&complete_record_sent_, false, __ATOMIC_RELEASE);
  }
#endif
}

size_t LogMirror::write(uint8_t byte) { return transport.writeLog(&byte, 1); }
size_t LogMirror::write(const uint8_t* bytes, size_t length) { return transport.writeLog(bytes, length); }

bool Transport::closeEnrollment() {
  if (!onboarding_) return false;
  __atomic_store_n(&passkey_allowed_handle_, BLE_CONN_HANDLE_INVALID, __ATOMIC_RELEASE);
  __atomic_store_n(&passkey_pending_, false, __ATOMIC_RELEASE);
  policy_.closeEnrollment();
  onboarding_ = false;
  const uint16_t closing_handle = enrollment_handle_;
  if (enrollment_handle_ != BLE_CONN_HANDLE_INVALID) {
    BLEConnection* connection = Bluefruit.Connection(enrollment_handle_);
    if (connection) {
      if (connection->bonded() && !policy_.hasPeer()) connection->removeBondKey();
      connection->disconnect();
    }
  }
  enrollment_handle_ = BLE_CONN_HANDLE_INVALID;
  if (closing_handle != BLE_CONN_HANDLE_INVALID) disable_pending_ = true;
  __atomic_store_n(&secured_, false, __ATOMIC_RELEASE);
  Bluefruit.Advertising.stop();
  Serial.println("Observer BLE enrollment closed.");
  return true;
}

bool Transport::allocateQueue() {
  if (!log_queue_) {
    log_queue_ = new (std::nothrow) Queue(true, boot_nonce_, sequence_state_);
    if (!log_queue_) return false;
    line_buffer_ = new (std::nothrow) LineBuffer<544, kLogQueueBytes, Queue>(*log_queue_);
    if (!line_buffer_) { delete log_queue_; log_queue_ = nullptr; return false; }
  }
  return true;
}

void Transport::releaseQueue() {
  delete line_buffer_; line_buffer_ = nullptr;
  delete log_queue_; log_queue_ = nullptr;
}

void Transport::updateStatsSnapshot() {
  stats_snapshot_.publish(evictedRecords(), evictedBytes(), truncatedRecords(), truncatedBytes(), failedNotifications(),
                          prequeue_unavailable_records_ + (log_queue_ ? log_queue_->unavailableRecords() : 0),
                          prequeue_unavailable_bytes_ + (log_queue_ ? log_queue_->unavailableBytes() : 0));
}

bool Transport::persistPeer(bool has_peer, const PeerIdentity& peer) {
  return save_peer_ && save_peer_(has_peer, peer);
}

}  // namespace observer

#endif
