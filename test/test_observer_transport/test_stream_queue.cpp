#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <new>

// Observer capture uses nothrow allocation; inject real allocation failures
// without replacing normal allocations used by gtest/the fake platform.
inline int fake_fail_nothrow_after = -1, fake_nothrow_allocations = 0;
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  ++fake_nothrow_allocations;
  if (fake_fail_nothrow_after == 0) return nullptr;
  if (fake_fail_nothrow_after > 0) --fake_fail_nothrow_after;
  try { return ::operator new(size); } catch (...) { return nullptr; }
}

#include "observer/ObserverStreamQueue.h"
#include "observer/ObserverCommand.h"
#include "observer/ObserverLineBuffer.h"
#include "observer/ObserverRecordQueue.h"
#include "observer/ObserverProtocol.h"
#include "observer/ObserverPolicy.h"
#include "observer/ObserverPreference.h"
#include "observer/ObserverAckIngress.h"
#include "observer/ObserverDelivery.h"
#include "observer/ObserverCommandIngress.h"
#include "observer/ObserverStatsSnapshot.h"
#include "observer/ObserverConnectionAdmission.h"
#include "observer/ObserverCallbackState.h"
// This fixture brings its own File/FILESYSTEM; skip the shared mock stubs.
#define OBSERVER_TEST_OWN_FILESYSTEM 1
#include <Arduino.h>
#include <MeshCore.h>
// Storage adapter used to execute the real CommonCLI legacy load/save path in
// the native fixture without mocking NodePrefs or ConfigSerializer.
class File : public Stream {
 public:
  File() = default;
  File(std::string* bytes, bool writable) : bytes_(bytes), writable_(writable) {}
  explicit operator bool() const { return bytes_ != nullptr; }
  int available() override { return bytes_ ? static_cast<int>(bytes_->size() - position_) : 0; }
  int read() override { return bytes_ && position_ < bytes_->size() ? (*bytes_)[position_++] : -1; }
  size_t read(uint8_t* output, size_t length) {
    size_t count = 0;
    while (count < length && available() > 0) output[count++] = static_cast<uint8_t>(read());
    return count;
  }
  size_t write(uint8_t byte) override {
    if (!bytes_ || !writable_) return 0;
    bytes_->push_back(static_cast<char>(byte));
    return 1;
  }
  using Stream::write;
  size_t print(unsigned char value, int = DEC) override { return printNumber(value); }
  size_t print(int value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned int value, int = DEC) override { return printNumber(value); }
  size_t print(long value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned long value, int = DEC) override { return printNumber(value); }
  size_t print(long long value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned long long value, int = DEC) override { return printNumber(value); }
  size_t print(double value, int precision = 2) override {
    char text[48];
    snprintf(text, sizeof(text), "%.*f", precision, value);
    return Print::print(text);
  }
  void close() {}

 private:
  template <typename T>
  size_t printNumber(T value) {
    char text[48];
    snprintf(text, sizeof(text), "%lld", static_cast<long long>(value));
    return Print::print(text);
  }
  std::string* bytes_ = nullptr;
  size_t position_ = 0;
  bool writable_ = false;
};

class FILESYSTEM {
 public:
  bool exists(const char* path) const {
    return (strcmp(path, "/com_prefs") == 0 && has_legacy_) ||
           (strcmp(path, "/prefs.json") == 0 && has_prefs_);
  }
  File open(const char* path) {
    if (strcmp(path, "/com_prefs") == 0 && has_legacy_) return File(&legacy_, false);
    if (strcmp(path, "/prefs.json") == 0 && has_prefs_) return File(&prefs_, false);
    return {};
  }
  File open(const char* path, const char* mode, bool) {
    if (strcmp(path, "/prefs.json") != 0 || strcmp(mode, "w") != 0) return {};
    prefs_.clear();
    has_prefs_ = true;
    return File(&prefs_, true);
  }
  bool remove(const char* path) {
    if (strcmp(path, "/prefs.json") != 0) return false;
    prefs_.clear();
    has_prefs_ = false;
    return true;
  }
  void mkdir(const char*) {}
  void addLegacyPrefs(std::string bytes) { legacy_ = std::move(bytes); has_legacy_ = true; }
  const std::string& migratedPrefs() const { return prefs_; }
  bool hasMigratedPrefs() const { return has_prefs_; }

 private:
  std::string legacy_;
  std::string prefs_;
  bool has_legacy_ = false;
  bool has_prefs_ = false;
};
#include "observer/ObserverNodePrefs.h"

// The native test filter compiles the real serializer sources but omits the
// target-bound CommonRadioPrefs.cpp translation unit. These fallbacks satisfy
// its unrelated dynamic-radio vtable slots; preference serialization and
// CommonRadioPrefs::handleCommand remain production code, not test doubles.
bool CommonRadioPrefs::setByKey(const char*, const char*) { return false; }
bool CommonRadioPrefs::getByKey(const char*, char*, size_t) { return false; }

// Compile the actual transport against only a fake platform boundary. The fake
// enforces the real S140 ordering: entropy is unavailable before Bluefruit.begin.
#include <Arduino.h>
#define SENSECAP_OBSERVER_TRANSPORT
#define OBSERVER_TRANSPORT_PLATFORM_TEST
constexpr uint16_t BLE_CONN_HANDLE_INVALID = 0xffff;
constexpr int LED_BLUE = 7, LED_STATE_ON = 0;
inline int fake_blue_led = LED_STATE_ON;
inline void digitalWrite(int pin, int level) {
  if (pin == LED_BLUE) fake_blue_led = level;
}
constexpr int NRF_SUCCESS = 0, BANDWIDTH_MAX = 0, CHR_PROPS_NOTIFY = 1,
    CHR_PROPS_WRITE = 2, CHR_PROPS_READ = 4, SECMODE_ENC_WITH_MITM = 3,
    SECMODE_NO_ACCESS = 0, BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE = 0,
    BLE_GAP_ROLE_PERIPH = 0, BLE_GATTS_AUTHORIZE_TYPE_READ = 0,
    BLE_GATT_STATUS_SUCCESS = 0, BLE_GAP_SEC_STATUS_SUCCESS = 0;
struct ble_gap_addr_t { uint8_t addr_type = 0; uint8_t addr[6]{}; uint8_t addr_id_peer = 0; };
struct ble_gatts_evt_read_t { uint16_t offset = 0; };
struct ble_gatts_rw_authorize_reply_params_t {
  int type;
  struct { struct { int gatt_status, update, len, offset; uint8_t* p_data; } read; } params;
};
struct FakeSerial {
  bool open = true;
  std::string output;
  operator bool() const { return open; }
  int available() { return 0; } int read() { return -1; } int peek() { return -1; }
  void print(const char* text) { output += text; }
  void println(const char* text = "") { output += text; output += '\n'; }
  size_t write(const uint8_t* data, size_t size) { output.append(reinterpret_cast<const char*>(data), size); return size; }
} Serial;
struct BLEUuid { explicit BLEUuid(const uint8_t*) {} };
inline int fake_gatt_status = 0, fake_gatt_calls = 0, fake_notifications = 0;
inline bool fake_notification_ok = true;
inline std::vector<std::vector<uint8_t>> fake_notification_attempts;
inline std::vector<std::vector<uint8_t>> fake_accepted_notifications;
inline int fake_log_max_len = 0;
inline std::vector<int> fake_max_lengths;
struct BLECharacteristic;
using FakeWriteCallback = void (*)(uint16_t, BLECharacteristic*, uint8_t*, uint16_t);
inline std::vector<FakeWriteCallback> fake_writes;
inline void (*fake_read)(uint16_t, BLECharacteristic*, ble_gatts_evt_read_t*) = nullptr;
inline int fake_reads = 0;
struct BLEService {
  explicit BLEService(BLEUuid) {}
  int begin() { ++fake_gatt_calls; return fake_gatt_status; }
};
struct BLECharacteristic {
  explicit BLECharacteristic(BLEUuid) {}
  void setProperties(int) {} void setPermission(int, int) {} void setMaxLen(int length) { fake_log_max_len = length; fake_max_lengths.push_back(length); }
  void setWriteCallback(FakeWriteCallback cb) { fake_writes.push_back(cb); }
  void setReadAuthorizeCallback(decltype(fake_read) cb) { fake_read = cb; }
  int begin() { ++fake_gatt_calls; return fake_gatt_status; }
  bool notifyEnabled(uint16_t) { return true; }
  bool notify(uint16_t, const uint8_t* bytes, size_t length) {
    ++fake_notifications;
    fake_notification_attempts.emplace_back(bytes, bytes + length);
    if (!fake_notification_ok) return false;
    fake_accepted_notifications.emplace_back(bytes, bytes + length);
    return true;
  }
};
struct BLEConnection {
  bool live = false, bond = false, secure = false;
  unsigned disconnects = 0;
  uint16_t mtu = 23;
  uint16_t getMtu() { return mtu; }
  ble_gap_addr_t address{};
  bool connected() { return live; } bool bonded() { return bond; } bool secured() { return secure; }
  ble_gap_addr_t getPeerAddr() { return address; }
  void disconnect() { ++disconnects; } // Completion is deliberately asynchronous.
  void removeBondKey() { bond = false; }
};
struct FakeBluefruit {
  bool auto_led = true, auto_led_at_begin = true;
  bool core = false, begin_ok = true, entropy = false;
  int begins = 0, entropy_calls = 0, entropy_before_core = 0;
  uint64_t random = 42;
  BLEConnection connection;
  struct {
    bool running = false, restart = true, start_ok = true;
    int starts = 0, stops = 0;
    void addFlags(int) {} void addService(BLEService&) {} void setInterval(int, int) {}
    void restartOnDisconnect(bool value) { restart = value; }
    bool start(int) { ++starts; running = start_ok; return start_ok; }
    bool stop() { ++stops; running = false; return true; }
    bool isRunning() { return running; }
  } Advertising;
  struct { void addName() {} } ScanResponse;
  struct {
    bool mitm = false, display = false;
    bool (*passkey)(uint16_t, const uint8_t*, bool) = nullptr;
    void (*pair)(uint16_t, uint8_t) = nullptr;
    void (*secured)(uint16_t) = nullptr;
    void setMITM(bool value) { mitm = value; }
    void setIOCaps(bool d, bool, bool) { display = d; }
    void setPairPasskeyCallback(decltype(passkey) cb) { passkey = cb; }
    void setPairCompleteCallback(decltype(pair) cb) { pair = cb; }
    void setSecuredCallback(decltype(secured) cb) { secured = cb; }
  } Security;
  struct {
    void (*connect)(uint16_t) = nullptr;
    void (*disconnect)(uint16_t, uint8_t) = nullptr;
    void setConnectCallback(decltype(connect) cb) { connect = cb; }
    void setDisconnectCallback(decltype(disconnect) cb) { disconnect = cb; }
  } Periph;
  void configPrphBandwidth(int) {}
  void autoConnLed(bool enabled) { auto_led = enabled; }
  bool begin(int, int) { ++begins; auto_led_at_begin = auto_led; core = begin_ok; return begin_ok; }
  void setTxPower(int) {} void setName(const char*) {}
  BLEConnection* Connection(uint16_t handle) { return handle == 1 ? &connection : nullptr; }
} Bluefruit;
inline int sd_rand_application_bytes_available_get(uint8_t* available) {
  ++Bluefruit.entropy_calls;
  if (!Bluefruit.core) { ++Bluefruit.entropy_before_core; return 1; }
  *available = Bluefruit.entropy ? 8 : 0;
  return NRF_SUCCESS;
}
inline int sd_rand_application_vector_get(uint8_t* bytes, size_t size) {
  memcpy(bytes, &Bluefruit.random, size); return NRF_SUCCESS;
}
inline void bond_remove_key(int, ble_gap_addr_t*) {}
inline void sd_ble_gatts_rw_authorize_reply(uint16_t, ble_gatts_rw_authorize_reply_params_t*) { ++fake_reads; }
#include "observer/ObserverTransport.cpp"
inline unsigned fake_ota_resets = 0;
inline void enterOTADfu() { ++fake_ota_resets; }
#include "observer/ObserverOta.h"

TEST(ObserverAdminOta, CanonicalCommandResetsWithoutReinitializingTheObserverStack) {
  fake_ota_resets = 0;
  Bluefruit = FakeBluefruit{};
  ASSERT_TRUE(Bluefruit.begin(1, 0));
  char reply[96] = "";
  EXPECT_TRUE(observer::handleAdminOtaCommand("start ota", reply));
  EXPECT_EQ(fake_ota_resets, 1U);
  EXPECT_EQ(Bluefruit.begins, 1);
  // The hardware primitive must not return; never fabricate an OK if it does.
  EXPECT_STREQ(reply, "ERR: OTA reset returned");
  EXPECT_FALSE(observer::isAllowedReadCommand("start ota"));
}
TEST(ObserverAdminOta, InvalidSuffixCannotFallThroughToLegacyPrefixMatchedOta) {
  fake_ota_resets = 0;
  char reply[96] = "";
  for (const char* command : {"start ota extra", "start otax", "start ota "}) {
    EXPECT_TRUE(observer::handleAdminOtaCommand(command, reply));
    EXPECT_STREQ(reply, "ERR: use start ota without arguments");
  }
  EXPECT_EQ(fake_ota_resets, 0U);
}
TEST(ObserverAdminOta, OtherCommandsAreUntouched) {
  fake_ota_resets = 0;
  char reply[96] = "unchanged";
  for (const char* command : {"ver", "start", "", "observer enable", "reboot"}) {
    EXPECT_FALSE(observer::handleAdminOtaCommand(command, reply));
  }
  EXPECT_EQ(fake_ota_resets, 0U);
  EXPECT_STREQ(reply, "unchanged");
}

namespace observer {
struct TransportTestAccess {
  static void setHandle(Transport& t, uint16_t handle) { t.setConnectionHandle(handle); }
  static bool allocated(Transport& t) { return t.log_queue_ || t.line_buffer_; }
  static bool queued(Transport& t) { return t.log_queue_ && !t.log_queue_->empty(); }
  static uint64_t nonce(Transport& t) { return t.boot_nonce_; }
  static uint32_t head(Transport& t) { return t.log_queue_ ? t.log_queue_->headSequence() : 0; }
  static StatsValues stats(Transport& t) { t.updateStatsSnapshot(); return t.stats_snapshot_.read(); }
  static const void* queue(Transport& t) { return t.log_queue_; }
  static void dispose(Transport& t) { t.releaseQueue(); }
  static size_t queueSize(Transport& t) { return t.log_queue_ ? t.log_queue_->size() : 0; }
  static uint16_t headTotal(Transport& t) {
    RecordFragment fragment{};
    return (t.log_queue_ && t.log_queue_->peek(fragment, kMaximumRecordBytes)) ? fragment.total_length : 0;
  }
  static bool acknowledgeHead(Transport& t) {
    RecordFragment fragment{};
    return t.log_queue_ && t.log_queue_->peek(fragment, kMaximumRecordBytes) &&
           t.log_queue_->acknowledge(fragment.boot_nonce, fragment.sequence);
  }
};
}

namespace {
bool saved_enabled = false;
bool saveEnabled(bool enabled) { saved_enabled = enabled; return true; }
bool savePeer(bool, const observer::PeerIdentity&) { return true; }
class ObserverProductionLifecycle : public ::testing::Test {
 protected:
  observer::Transport t;
  void SetUp() override {
    Bluefruit = FakeBluefruit{}; Serial = FakeSerial{}; g_mock_millis = 0;
    fake_blue_led = LED_STATE_ON;
    fake_gatt_calls = fake_gatt_status = fake_notifications = 0; saved_enabled = false;
    fake_notification_ok = true;
    fake_notification_attempts.clear(); fake_accepted_notifications.clear(); fake_max_lengths.clear();
    fake_writes.clear(); fake_read = nullptr; fake_reads = 0;
    fake_fail_nothrow_after = -1; fake_nothrow_allocations = 0;
    t.setPreferenceCallbacks(saveEnabled, savePeer);
  }
  void TearDown() override {
    fake_notification_ok = true;
    observer::TransportTestAccess::dispose(t);
  }
  void ready(bool pinned = false) {
    observer::PeerIdentity peer{};
    t.attachPreferences(false, peer, pinned);
    Bluefruit.entropy = true;
    ASSERT_TRUE(t.enable(true)); t.service();
  }
  void connect(bool secure = true) {
    Bluefruit.Advertising.running = false;
    Bluefruit.connection.mtu = 23;
    Bluefruit.connection.live = true;
    Bluefruit.connection.bond = Bluefruit.connection.secure = secure;
    Bluefruit.Periph.connect(1); t.service();
    if (secure) { Bluefruit.Security.secured(1); t.service(); }
  }
  void disconnect() {
    Bluefruit.connection.live = false;
    const int starts = Bluefruit.Advertising.starts, gatt = fake_gatt_calls;
    const int allocations = fake_nothrow_allocations;
    const void* queue = observer::TransportTestAccess::queue(t);
    Bluefruit.Periph.disconnect(1, 0);
    EXPECT_EQ(Bluefruit.Advertising.starts, starts);
    EXPECT_EQ(fake_gatt_calls, gatt);
    EXPECT_EQ(fake_nothrow_allocations, allocations);
    EXPECT_EQ(observer::TransportTestAccess::queue(t), queue);
    t.service();
  }
};
TEST_F(ObserverProductionLifecycle, NotificationsUseNegotiatedMtuBudgetAndCompleteHeaders) {
  ready(true); connect();
  ASSERT_EQ(fake_max_lengths.front(), 244);
  const uint16_t mtus[] = {23, 24, 64, 247, 517};
  const size_t budgets[] = {20, 21, 61, 244, 244};
  const std::string line(254, 'x');
  const std::string complete_line = line + '\n';
  uint32_t expected_sequence = 1;
  uint16_t expected_offset = 0;
  for (size_t i = 0; i < 5; ++i) {
    Bluefruit.connection.mtu = mtus[i];
    t.writeLog(reinterpret_cast<const uint8_t*>(complete_line.data()), complete_line.size());
    const size_t before = fake_notification_attempts.size();
    g_mock_millis += 1000;
    t.service();
    ASSERT_GT(fake_notification_attempts.size(), before);
    const auto& frame = fake_notification_attempts.back();
    EXPECT_LE(frame.size(), budgets[i]);
    if (budgets[i] > 20) EXPECT_GT(frame.size(), 20U);
    if (budgets[i] == 21 || budgets[i] == 61) EXPECT_EQ(frame.size(), budgets[i]);
    ASSERT_GE(frame.size(), observer::kFrameHeaderBytes + 1);
    // Decode the wire header independently of the production decoder/encoder.
    uint64_t nonce = 0;
    for (unsigned byte = 0; byte < 8; ++byte)
      nonce |= static_cast<uint64_t>(frame[byte]) << (byte * 8);
    uint32_t sequence = 0;
    for (unsigned byte = 0; byte < 4; ++byte)
      sequence |= static_cast<uint32_t>(frame[8 + byte]) << (byte * 8);
    const uint16_t total = static_cast<uint16_t>(frame[12] | (uint16_t(frame[13]) << 8));
    const uint16_t offset = static_cast<uint16_t>(frame[14] | (uint16_t(frame[15]) << 8));
    const size_t payload = frame.size() - observer::kFrameHeaderBytes;
    EXPECT_EQ(nonce, 42U);
    EXPECT_EQ(sequence, expected_sequence);
    EXPECT_EQ(total, complete_line.size());
    EXPECT_EQ(offset, expected_offset);
    EXPECT_GT(payload, 0U);
    EXPECT_LE(payload, static_cast<size_t>(total - offset));
    if (payload <= total - offset) {
      expected_offset = static_cast<uint16_t>(offset + payload);
      if (expected_offset == total) expected_offset = 0;
    }
    g_mock_millis += 10;
  }
}

TEST_F(ObserverProductionLifecycle, MissingAndInvalidMtuDoNotNotifyOrConsume) {
  ready(true); connect();
  const uint8_t line[] = "invalid-mtu-guard\n";
  t.writeLog(line, sizeof(line)-1);
  observer::TransportTestAccess::setHandle(t, 2);
  g_mock_millis += 1000; t.service();
  EXPECT_TRUE(observer::TransportTestAccess::queued(t));
  EXPECT_TRUE(fake_notification_attempts.empty());
  observer::TransportTestAccess::setHandle(t, 1);
  for (uint16_t mtu : {uint16_t{0}, uint16_t{22}}) {
    Bluefruit.connection.mtu = mtu;
    g_mock_millis += 1000; t.service();
    EXPECT_TRUE(observer::TransportTestAccess::queued(t));
    EXPECT_TRUE(fake_notification_attempts.empty());
  }
  Bluefruit.connection.mtu = 23;
  g_mock_millis += 1000; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 1U);
  EXPECT_EQ(fake_notification_attempts.back().size(), 20U);
}

TEST_F(ObserverProductionLifecycle, ReconnectFromLargeMtuUsesNewDefaultMtu) {
  ready(true); connect();
  Bluefruit.connection.mtu = 247;
  const uint8_t first[] = "first-record\n";
  t.writeLog(first, sizeof(first)-1); g_mock_millis += 1000; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 1U);
  EXPECT_GT(fake_notification_attempts.back().size(), 20U);
  disconnect(); connect();
  const uint8_t second[] = "second-record\n";
  t.writeLog(second, sizeof(second)-1); g_mock_millis += 1000; t.service();
  ASSERT_GE(fake_notification_attempts.size(), 2U);
  EXPECT_LE(fake_notification_attempts.back().size(), 20U);
}

TEST_F(ObserverProductionLifecycle, FailedLargeMtuNotifyRetriesSameUnsentFragment) {
  ready(true); connect(); Bluefruit.connection.mtu = 247;
  const std::string line(254, 'x'); const std::string complete = line + '\n';
  t.writeLog(reinterpret_cast<const uint8_t*>(complete.data()), complete.size());
  g_mock_millis += 1000; t.service();
  ASSERT_EQ(fake_accepted_notifications.size(), 1U);
  const auto accepted = fake_accepted_notifications.back();
  ASSERT_EQ(accepted.size(), 244U);
  fake_notification_ok = false; g_mock_millis += 10; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 2U);
  const auto rejected = fake_notification_attempts.back();
  EXPECT_NE(rejected, accepted);
  fake_notification_ok = true; g_mock_millis += 10; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 3U);
  EXPECT_EQ(fake_notification_attempts.back(), rejected);
  EXPECT_EQ(fake_accepted_notifications.size(), 2U);
  EXPECT_TRUE(observer::TransportTestAccess::queued(t));
}

TEST_F(ObserverProductionLifecycle, Mtu247SendsFull244ByteNotification) {
  ready(true); connect(); Bluefruit.connection.mtu = 247;
  const std::string line(254, 'x'); const std::string complete = line + '\n';
  t.writeLog(reinterpret_cast<const uint8_t*>(complete.data()), complete.size());
  g_mock_millis += 1000; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 1U);
  ASSERT_EQ(fake_notification_attempts.back().size(), 244U);
  EXPECT_EQ(fake_notification_attempts.back()[12] | (uint16_t(fake_notification_attempts.back()[13]) << 8), 255U);
  EXPECT_EQ(fake_notification_attempts.back()[14] | (uint16_t(fake_notification_attempts.back()[15]) << 8), 0U);
}

TEST_F(ObserverProductionLifecycle, Mtu517SendsFull244ByteNotification) {
  ready(true); connect(); Bluefruit.connection.mtu = 517;
  const std::string line(254, 'x'); const std::string complete = line + '\n';
  t.writeLog(reinterpret_cast<const uint8_t*>(complete.data()), complete.size());
  g_mock_millis += 1000; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 1U);
  ASSERT_EQ(fake_notification_attempts.back().size(), 244U);
  EXPECT_EQ(fake_notification_attempts.back()[12] | (uint16_t(fake_notification_attempts.back()[13]) << 8), 255U);
  EXPECT_EQ(fake_notification_attempts.back()[14] | (uint16_t(fake_notification_attempts.back()[15]) << 8), 0U);
}

TEST_F(ObserverProductionLifecycle, DefaultOffNeverStartsCoreOrAllocates) {
  EXPECT_FALSE(t.begin("off", false)); t.service(); EXPECT_TRUE(t.enable(false)); t.service();
  EXPECT_EQ(Bluefruit.begins, 0); EXPECT_EQ(Bluefruit.entropy_calls, 0);
  EXPECT_EQ(fake_gatt_calls, 0); EXPECT_EQ(Bluefruit.Advertising.starts, 0);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
}
TEST_F(ObserverProductionLifecycle, NormalBluetoothLedIsDisabledBeforeCoreStartupAndDrivenOff) {
  ready(true);
  EXPECT_FALSE(Bluefruit.auto_led_at_begin);
  EXPECT_FALSE(Bluefruit.auto_led);
  EXPECT_EQ(fake_blue_led, 1 - LED_STATE_ON);
  connect(); disconnect();
  EXPECT_FALSE(Bluefruit.auto_led);
  EXPECT_EQ(fake_blue_led, 1 - LED_STATE_ON);
}
TEST_F(ObserverProductionLifecycle, CoreBeforeEntropyAndDelayedReadinessDoesNotUndoOptin) {
  t.attachPreferences(false, observer::PeerIdentity{}, true);
  ASSERT_TRUE(t.enable(true));
  EXPECT_TRUE(saved_enabled); EXPECT_EQ(Bluefruit.begins, 1);
  EXPECT_EQ(Bluefruit.entropy_before_core, 0);
  for (int i = 0; i < 20; ++i) t.service();
  EXPECT_LE(Bluefruit.entropy_calls, 1); EXPECT_TRUE(saved_enabled);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  EXPECT_FALSE(Bluefruit.Advertising.running);
  const uint8_t line[] = "RAW pending\n"; t.writeLog(line, sizeof(line)-1);
  EXPECT_EQ(observer::TransportTestAccess::nonce(t), 0U); EXPECT_EQ(fake_notifications, 0);
  Bluefruit.entropy = true; g_mock_millis += 100; t.service();
  EXPECT_TRUE(observer::TransportTestAccess::allocated(t));
  EXPECT_EQ(observer::TransportTestAccess::nonce(t), 42U);
  EXPECT_TRUE(Bluefruit.Advertising.running);
  t.writeLog(line, sizeof(line)-1); EXPECT_TRUE(observer::TransportTestAccess::queued(t));
}
TEST_F(ObserverProductionLifecycle, HardCoreFailureRollsBack) {
  Bluefruit.begin_ok = false;
  EXPECT_FALSE(t.enable(true)); EXPECT_FALSE(saved_enabled);
  EXPECT_FALSE(t.initialized()); EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  EXPECT_EQ(Bluefruit.entropy_calls, 0); EXPECT_FALSE(Bluefruit.Advertising.running);
}

TEST_F(ObserverProductionLifecycle, PreEntropyLossReservesIdentityAndDiscardsPartialLineTail) {
  ASSERT_TRUE(t.enable(true));
  const uint8_t pending[] = "RAW lost\nRAW par";
  t.writeLog(pending, sizeof(pending) - 1);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  Bluefruit.entropy = true; g_mock_millis += 100; t.service();
  const uint8_t resumed[] = "tial\nOK\n";
  t.writeLog(resumed, sizeof(resumed) - 1);
  EXPECT_EQ(observer::TransportTestAccess::head(t), 3U);
  const auto stats = observer::TransportTestAccess::stats(t);
  EXPECT_EQ(stats.unavailable_records, 2U);
  EXPECT_EQ(stats.unavailable_bytes, 21U);
  EXPECT_EQ(stats.truncated_records, 0U);
  EXPECT_EQ(stats.evicted_records, 0U);
}
TEST_F(ObserverProductionLifecycle, GattFailureRollsBackWithoutRetryingPartialRegistration) {
  fake_gatt_status = 1; Bluefruit.entropy = true;
  EXPECT_FALSE(t.enable(true)); EXPECT_FALSE(saved_enabled);
  const int calls = fake_gatt_calls;
  t.service(); EXPECT_FALSE(t.enable(true)); EXPECT_EQ(fake_gatt_calls, calls);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
}
TEST_F(ObserverProductionLifecycle, QueueAllocationFailureRollsBackAfterEntropy) {
  ASSERT_TRUE(t.enable(true)); Bluefruit.entropy = true; fake_fail_nothrow_after = 0;
  t.service(); EXPECT_FALSE(saved_enabled); EXPECT_FALSE(t.initialized());
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  EXPECT_FALSE(Bluefruit.Advertising.running); EXPECT_FALSE(t.enable(true));
}
TEST_F(ObserverProductionLifecycle, LineAllocationFailureAlsoReleasesQueue) {
  ASSERT_TRUE(t.enable(true)); Bluefruit.entropy = true; fake_fail_nothrow_after = 1;
  t.service(); EXPECT_FALSE(saved_enabled); EXPECT_FALSE(t.initialized());
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  EXPECT_FALSE(Bluefruit.Advertising.running);
}
TEST_F(ObserverProductionLifecycle, SecureDisconnectRestartsAndRetainsBacklog) {
  ready(true); connect();
  const uint8_t line[] = "RAW backlog\n"; t.writeLog(line, sizeof(line)-1);
  disconnect(); EXPECT_TRUE(Bluefruit.Advertising.running);
  EXPECT_TRUE(observer::TransportTestAccess::queued(t));
  connect(); g_mock_millis += 1000; t.service(); EXPECT_GT(fake_notifications, 0);
}
TEST_F(ObserverProductionLifecycle, RejectedNotifyRetriesUnsentFragmentWithoutRewinding) {
  ready(true); connect();
  const uint8_t line[] = "abcdefghijk\n";
  t.writeLog(line, sizeof(line)-1);
  t.service(); // accepted offset zero, four payload bytes
  ASSERT_EQ(fake_accepted_notifications.size(), 1U);
  fake_notification_ok = false;
  g_mock_millis += 10; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 2U);
  EXPECT_EQ(fake_notification_attempts.back()[14], 4U);
  fake_notification_ok = true;
  g_mock_millis += 9; t.service();
  EXPECT_EQ(fake_notification_attempts.size(), 2U); // pacing still applies
  g_mock_millis += 1; t.service();
  ASSERT_EQ(fake_notification_attempts.size(), 3U);
  EXPECT_EQ(fake_notification_attempts[1], fake_notification_attempts[2]);
  EXPECT_EQ(t.failedNotifications(), 1U);
  EXPECT_TRUE(observer::TransportTestAccess::queued(t)); // notify is not ACK
  fake_notification_ok = false;
  g_mock_millis += 10; t.service(); // reject the final fragment too
  EXPECT_EQ(fake_notification_attempts.back()[14], 8U);
  fake_notification_ok = true;
  g_mock_millis += 10; t.service();
  ASSERT_EQ(fake_accepted_notifications.size(), 3U);
  EXPECT_EQ(fake_accepted_notifications.back()[14], 8U);
  EXPECT_TRUE(observer::TransportTestAccess::queued(t));
  uint8_t ack[] = "ack 000000000000002a 1";
  ASSERT_EQ(fake_writes.size(), 2U);
  fake_writes[1](1, nullptr, ack, sizeof(ack)-1); t.service();
  EXPECT_FALSE(observer::TransportTestAccess::queued(t));
  const size_t attempts = fake_notification_attempts.size();
  g_mock_millis += 1000; t.service();
  EXPECT_EQ(fake_notification_attempts.size(), attempts); // no replay after ACK
}
TEST_F(ObserverProductionLifecycle, LongRecordCompletesWithIntermittentNotifyRejection) {
  ready(true); connect();
  const std::string line = std::string(129, 'x') + '\n';
  t.writeLog(reinterpret_cast<const uint8_t*>(line.data()), line.size());
  for (unsigned tick = 0; tick < 49; ++tick) {
    fake_notification_ok = tick % 3 != 2;
    t.service(); g_mock_millis += 10;
  }
  ASSERT_EQ(fake_accepted_notifications.size(), 33U);
  std::string received;
  for (const auto& frame : fake_accepted_notifications) {
    ASSERT_GE(frame.size(), 17U);
    EXPECT_LE(frame.size(), 20U);
    EXPECT_EQ(frame[8], 1U); EXPECT_EQ(frame[12], 130U);
    ASSERT_EQ(frame[14] | (frame[15] << 8), received.size());
    received.append(reinterpret_cast<const char*>(frame.data() + 16), frame.size() - 16);
  }
  EXPECT_EQ(received, line);
  EXPECT_EQ(t.failedNotifications(), 16U);
  EXPECT_TRUE(observer::TransportTestAccess::queued(t));
  fake_notification_ok = true;
  const size_t attempts = fake_notification_attempts.size();
  g_mock_millis += 989; t.service();
  EXPECT_EQ(fake_notification_attempts.size(), attempts);
  g_mock_millis += 1; t.service(); // full record ACK timeout still rewinds
  ASSERT_EQ(fake_notification_attempts.size(), attempts + 1);
  EXPECT_EQ(fake_notification_attempts.back()[14], 0U);
}
TEST_F(ObserverProductionLifecycle, DisconnectAfterPartialDeliveryStillReplaysWholeRecord) {
  ready(true); connect();
  const uint8_t line[] = "abcdefghijk\n";
  t.writeLog(line, sizeof(line)-1); t.service();
  g_mock_millis += 10; t.service();
  ASSERT_EQ(fake_accepted_notifications.back()[14], 4U);
  disconnect(); connect();
  ASSERT_EQ(fake_accepted_notifications.size(), 3U);
  EXPECT_EQ(fake_accepted_notifications.back()[14], 0U);
}
TEST_F(ObserverProductionLifecycle, EnrollmentPinsThenDisconnectRestartsWithoutUnbondedData) {
  ready(); ASSERT_TRUE(t.beginEnrollment()); connect(false);
  const uint8_t pin[] = "123456";
  EXPECT_TRUE(Bluefruit.Security.mitm); EXPECT_TRUE(Bluefruit.Security.display);
  ASSERT_TRUE(Bluefruit.Security.passkey(1, pin, false));
  EXPECT_EQ(Serial.output.find("123456"), std::string::npos);
  t.service(); EXPECT_NE(Serial.output.find("123456"), std::string::npos);
  Bluefruit.connection.bond = Bluefruit.connection.secure = true;
  Bluefruit.Security.pair(1, 0); EXPECT_TRUE(t.enrollmentOpen());
  t.service(); EXPECT_FALSE(t.enrollmentOpen());
  EXPECT_FALSE(Bluefruit.Security.passkey(1, pin, false));
  disconnect(); EXPECT_TRUE(Bluefruit.Advertising.running);
  const uint8_t line[] = "RAW backlog\n"; t.writeLog(line, sizeof(line)-1);
  connect(false); g_mock_millis += 1000; t.service(); EXPECT_EQ(fake_notifications, 0);
  uint8_t key_command[] = "get prv.key";
  ASSERT_EQ(fake_writes.size(), 2U);
  fake_writes[0](1, nullptr, key_command, sizeof(key_command)-1);
  char command[96]; EXPECT_FALSE(t.takeCommand(command, sizeof(command)));
  ble_gatts_evt_read_t request{}; ASSERT_NE(fake_read, nullptr);
  fake_read(1, nullptr, &request); EXPECT_EQ(fake_reads, 0);
  disconnect(); connect(); g_mock_millis += 1000; t.service(); EXPECT_GT(fake_notifications, 0);
}
TEST_F(ObserverProductionLifecycle, DisablePendingDisconnectNeverRestarts) {
  ready(true); connect(); ASSERT_TRUE(t.enable(false));
  EXPECT_FALSE(Bluefruit.Advertising.running);
  EXPECT_TRUE(observer::TransportTestAccess::allocated(t));
  disconnect(); EXPECT_FALSE(Bluefruit.Advertising.running);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  g_mock_millis += 1000; t.service(); EXPECT_FALSE(Bluefruit.Advertising.running);
}
TEST_F(ObserverProductionLifecycle, DisableWhileWaitingForEntropyStopsRetriesAndReenableRecovers) {
  ASSERT_TRUE(t.enable(true)); t.service();
  ASSERT_TRUE(t.enable(false)); const int calls = Bluefruit.entropy_calls;
  g_mock_millis += 1000; t.service(); EXPECT_EQ(Bluefruit.entropy_calls, calls);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t));
  ASSERT_TRUE(t.enable(true)); Bluefruit.entropy = true; t.service();
  EXPECT_EQ(Bluefruit.begins, 1); EXPECT_TRUE(observer::TransportTestAccess::allocated(t));
}
TEST_F(ObserverProductionLifecycle, ZeroEntropyNeverCreatesPredictableSession) {
  ASSERT_TRUE(t.enable(true)); Bluefruit.entropy = true; Bluefruit.random = 0;
  t.service(); EXPECT_TRUE(saved_enabled);
  EXPECT_FALSE(observer::TransportTestAccess::allocated(t)); EXPECT_FALSE(t.beginEnrollment());
  g_mock_millis += 100; Bluefruit.random = 43; t.service();
  EXPECT_EQ(observer::TransportTestAccess::nonce(t), 43U);
  EXPECT_TRUE(observer::TransportTestAccess::allocated(t));
}
TEST_F(ObserverProductionLifecycle, EnrollmentExpiresAndUsbLossSuppressesPasskey) {
  ready(); ASSERT_TRUE(t.beginEnrollment()); connect(false);
  const uint8_t pin[] = "123456";
  ASSERT_TRUE(Bluefruit.Security.passkey(1, pin, false)); Serial.open = false;
  t.service(); EXPECT_FALSE(t.enrollmentOpen());
  EXPECT_EQ(Serial.output.find("123456"), std::string::npos);
  EXPECT_FALSE(Bluefruit.Security.passkey(1, pin, false));
  disconnect(); Serial.open = true; ASSERT_TRUE(t.beginEnrollment());
  g_mock_millis += 60000; t.service(); EXPECT_FALSE(t.enrollmentOpen());
  EXPECT_FALSE(Bluefruit.Advertising.running); // No pinned peer, no open window.
}
} // namespace

namespace {

TEST(ObserverStreamQueue, PreservesOrderedBytesAcrossEnqueueAndDequeue) {
  observer::StreamQueue<16> queue(true);
  const uint8_t first[] = {'R', 'A', 'W', '\r', '\n'};
  const uint8_t second[] = {'R', 'X', '\n'};

  EXPECT_EQ(queue.enqueue(first, sizeof(first)), sizeof(first));
  EXPECT_EQ(queue.enqueue(second, sizeof(second)), sizeof(second));

  std::array<uint8_t, 8> output{};
  EXPECT_EQ(queue.dequeue(output.data(), output.size()), output.size());
  EXPECT_EQ(std::vector<uint8_t>(output.begin(), output.end()),
            (std::vector<uint8_t>{'R', 'A', 'W', '\r', '\n', 'R', 'X', '\n'}));
}

TEST(ObserverStreamQueue, CountsOverflowWithoutPartiallyAppendingARecord) {
  observer::StreamQueue<4> queue(true);
  const uint8_t kept[] = {1, 2, 3};
  const uint8_t dropped[] = {4, 5};

  EXPECT_EQ(queue.enqueue(kept, sizeof(kept)), sizeof(kept));
  EXPECT_EQ(queue.enqueue(dropped, sizeof(dropped)), 0U);
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(dropped));

  std::array<uint8_t, 4> output{};
  EXPECT_EQ(queue.dequeue(output.data(), output.size()), sizeof(kept));
  EXPECT_EQ(std::vector<uint8_t>(output.begin(), output.begin() + sizeof(kept)),
            (std::vector<uint8_t>{1, 2, 3}));
}

TEST(ObserverStreamQueue, WrapsWithoutReorderingBytes) {
  observer::StreamQueue<5> queue(true);
  const uint8_t first[] = {1, 2, 3, 4};
  const uint8_t second[] = {5, 6, 7};
  std::array<uint8_t, 2> first_output{};

  ASSERT_EQ(queue.enqueue(first, sizeof(first)), sizeof(first));
  ASSERT_EQ(queue.dequeue(first_output.data(), first_output.size()), first_output.size());
  ASSERT_EQ(queue.enqueue(second, sizeof(second)), sizeof(second));

  std::array<uint8_t, 5> output{};
  EXPECT_EQ(queue.dequeue(output.data(), output.size()), 5U);
  EXPECT_EQ(std::vector<uint8_t>(output.begin(), output.end()),
            (std::vector<uint8_t>{3, 4, 5, 6, 7}));
}

TEST(ObserverStreamQueue, ObserverOffDoesNotRetainOrCountBytes) {
  observer::StreamQueue<8> queue(false);
  const uint8_t bytes[] = {'R', 'A', 'W', '\r', '\n'};

  EXPECT_EQ(queue.enqueue(bytes, sizeof(bytes)), sizeof(bytes));
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.droppedRecords(), 0U);
  EXPECT_EQ(queue.droppedBytes(), 0U);
}

TEST(ObserverStreamQueue, PreservesClassicRawThenRxThenTxBytesWithoutSynthesizingTxRaw) {
  observer::StreamQueue<128> queue(true);
  const uint8_t raw[] = "12:00:00 - 1/1/2026 U RAW: AABB\r\n";
  const uint8_t rx[] = "12:00:00 - 1/1/2026 U: RX, len=2\r\n";
  const uint8_t tx[] = "12:00:00 - 1/1/2026 U: TX, len=2\r\n";
  std::vector<uint8_t> expected;
  expected.insert(expected.end(), raw, raw + sizeof(raw) - 1);
  expected.insert(expected.end(), rx, rx + sizeof(rx) - 1);
  expected.insert(expected.end(), tx, tx + sizeof(tx) - 1);

  ASSERT_EQ(queue.enqueue(raw, sizeof(raw) - 1), sizeof(raw) - 1);
  ASSERT_EQ(queue.enqueue(rx, sizeof(rx) - 1), sizeof(rx) - 1);
  ASSERT_EQ(queue.enqueue(tx, sizeof(tx) - 1), sizeof(tx) - 1);

  std::vector<uint8_t> output(expected.size());
  EXPECT_EQ(queue.dequeue(output.data(), output.size()), expected.size());
  EXPECT_EQ(output, expected);
}

TEST(ObserverLineBuffer, CommitsClassicLogOnlyWhenNewlineCompletesTheLine) {
  observer::StreamQueue<64> queue(true);
  observer::LineBuffer<64, 64> mirror(queue);
  const uint8_t raw_prefix[] = "RAW: AABB";
  const uint8_t raw_suffix[] = "\r\n";

  EXPECT_EQ(mirror.write(raw_prefix, sizeof(raw_prefix) - 1), sizeof(raw_prefix) - 1);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(mirror.write(raw_suffix, sizeof(raw_suffix) - 1), sizeof(raw_suffix) - 1);

  std::array<uint8_t, 11> output{};
  EXPECT_EQ(queue.dequeue(output.data(), output.size()), output.size());
  EXPECT_EQ(std::vector<uint8_t>(output.begin(), output.end()),
            (std::vector<uint8_t>{'R', 'A', 'W', ':', ' ', 'A', 'A', 'B', 'B', '\r', '\n'}));
}

TEST(ObserverLineBuffer, DropsWholeLineWhenQueueCannotFitCompletedClassicLog) {
  observer::StreamQueue<8> queue(true);
  observer::LineBuffer<32, 8> mirror(queue);
  const uint8_t line[] = "RAW: AABB\r\n";

  EXPECT_EQ(mirror.write(line, sizeof(line) - 1), sizeof(line) - 1);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(line) - 1);
}

TEST(ObserverCommand, AcceptsOnlyCanonicalUint32TimeAndFixedReadAllowlist) {
  uint32_t epoch = 0;
  EXPECT_TRUE(observer::parseCanonicalEpoch("time 1735689600", epoch));
  EXPECT_EQ(epoch, 1735689600U);
  EXPECT_FALSE(observer::parseCanonicalEpoch("time 01735689600", epoch));
  EXPECT_FALSE(observer::parseCanonicalEpoch("time 1 extra", epoch));
  EXPECT_FALSE(observer::parseCanonicalEpoch("time 4294967296", epoch));
  EXPECT_FALSE(observer::parseCanonicalEpoch("time reboot", epoch));
  EXPECT_TRUE(observer::isAllowedReadCommand("get prv.key"));
  EXPECT_TRUE(observer::isAllowedReadCommand("stats-packets"));
  EXPECT_TRUE(observer::isAllowedReadCommand("ver"));
  EXPECT_TRUE(observer::isAllowedReadCommand("board"));
  EXPECT_FALSE(observer::isAllowedReadCommand("get name extra"));
  EXPECT_FALSE(observer::isAllowedReadCommand("start ota"));
}

TEST(ObserverRecordQueue, ReassemblyRejectsMidLineNotificationFailureBeforeNextLine) {
  observer::RecordQueue<64, 4> queue(true);
  const uint8_t raw[] = "RAW: AABB\r\n";
  const uint8_t rx[] = "RX: CCDD\r\n";
  ASSERT_TRUE(queue.enqueue(raw, sizeof(raw) - 1));
  ASSERT_TRUE(queue.enqueue(rx, sizeof(rx) - 1));

  observer::RecordFragment first{};
  ASSERT_TRUE(queue.peek(first, 5));
  EXPECT_EQ(first.sequence, 1U);
  EXPECT_EQ(first.offset, 0U);
  EXPECT_EQ(first.length, 5U);
  queue.consume(first.length);  // The first notification reached the Pi.
  queue.dropHeadRecord();       // A later notification failed.

  observer::RecordFragment next{};
  ASSERT_TRUE(queue.peek(next, 5));
  EXPECT_EQ(next.sequence, 2U);
  EXPECT_EQ(next.offset, 0U);
  EXPECT_EQ(next.total_length, sizeof(rx) - 1);
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(raw) - 1);
}

TEST(ObserverRecordQueue, CapacityEvictsOldestCompleteRecordAndReportsLoss) {
  observer::RecordQueue<8, 2> queue(true);
  const uint8_t first[] = "RAW\n";
  const uint8_t second[] = "RX\n";
  const uint8_t third[] = "TX\n";

  ASSERT_TRUE(queue.enqueue(first, sizeof(first) - 1));
  ASSERT_TRUE(queue.enqueue(second, sizeof(second) - 1));
  EXPECT_TRUE(queue.enqueue(third, sizeof(third) - 1));

  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, 8));
  EXPECT_EQ(std::vector<uint8_t>(head.bytes, head.bytes + head.length),
            (std::vector<uint8_t>{'R', 'X', '\n'}));
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(first) - 1);
  EXPECT_EQ(queue.evictedRecords(), 1U);
  EXPECT_EQ(queue.evictedBytes(), sizeof(first) - 1);
}

TEST(ObserverRecordQueue, OverlongClassicLineIsCountedAsTruncationNotEviction) {
  observer::RecordQueue<16, 4> queue(true, 7);
  observer::LineBuffer<4, 16, observer::RecordQueue<16, 4>> line_buffer(queue);
  const uint8_t line[] = "abcde\n";

  ASSERT_EQ(line_buffer.write(line, sizeof(line) - 1), sizeof(line) - 1);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.evictedRecords(), 0U);
  EXPECT_EQ(queue.evictedBytes(), 0U);
  EXPECT_EQ(queue.truncatedRecords(), 1U);
  EXPECT_EQ(queue.truncatedBytes(), sizeof(line) - 1);
}

struct OverflowReporter {
  size_t bytes = 0;
  void drop(size_t count) { bytes += count; }
};

TEST(ObserverLineBuffer, BoundaryNewlineAndCrlfDropOnlyOverlongRecords) {
  for (const std::string ending : {std::string("\n"), std::string("\r\n")}) {
    observer::RecordQueue<1024, 8> queue(true, 7);
    observer::LineBuffer<256, 1024, decltype(queue)> buffer(queue);
    const std::string input = std::string(256, 'x') + ending + "OK\n" +
        std::string(300, 'y') + ending + "YES\r\n";
    for (const uint8_t byte : input) buffer.write(byte);
    EXPECT_EQ(queue.truncatedRecords(), 2U);
    EXPECT_EQ(queue.truncatedBytes(), 556U + ending.size() * 2);
    EXPECT_EQ(queue.size(), 2U);
    observer::RecordFragment head{};
    ASSERT_TRUE(queue.peek(head, 256));
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(head.bytes), head.length), "OK\n");
    EXPECT_EQ(head.sequence, 2U);
    EXPECT_FALSE(queue.acknowledge(7, 1)); // missing identity is never an ACK
    EXPECT_EQ(queue.size(), 2U);
    ASSERT_TRUE(queue.acknowledge(7, head.sequence));
    ASSERT_TRUE(queue.peek(head, 256));
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(head.bytes), head.length), "YES\r\n");
    EXPECT_EQ(head.sequence, 4U);
  }
}

TEST(ObserverRecordQueue, RejectedRecordsReserveIdentityWithoutAcknowledgingOrMovingHead) {
  observer::QueueSequenceState sequence;
  observer::RecordQueue<8, 4> queue(true, 9, sequence);
  const uint8_t line[] = "OK\n";
  ASSERT_TRUE(queue.enqueue(line, 3));
  queue.drop(257);
  EXPECT_FALSE(queue.acknowledge(9, 2));
  EXPECT_EQ(queue.headSequence(), 1U);
  EXPECT_EQ(queue.size(), 1U);
  ASSERT_TRUE(queue.acknowledge(9, 1));
  EXPECT_FALSE(queue.enqueue(line, 9)); // cannot fit even an empty queue
  EXPECT_FALSE(queue.enqueue(nullptr, 3));
  ASSERT_TRUE(queue.enqueue(line, 3));
  EXPECT_EQ(queue.headSequence(), 5U);
  EXPECT_EQ(sequence.next(), 6U);
  EXPECT_EQ(queue.truncatedRecords(), 3U);
  EXPECT_EQ(queue.unavailableRecords(), 0U);
}

TEST(ObserverRecordQueue, LostLastIdentityExhaustsUntilFreshNonceAndDoesNotWrap) {
  observer::QueueSequenceState sequence;
  sequence.advance(UINT32_MAX - 1);
  observer::RecordQueue<8, 4> queue(true, 9, sequence);
  queue.drop(257);
  EXPECT_TRUE(queue.sequenceExhausted());
  const uint8_t line[] = "OK\n";
  EXPECT_FALSE(queue.enqueue(line, 3));
  queue.drop(258);
  EXPECT_TRUE(queue.sequenceExhausted());
  EXPECT_EQ(sequence.next(), 0U);
  EXPECT_FALSE(queue.setBootNonce(9));
  EXPECT_FALSE(queue.setBootNonce(0));
  ASSERT_TRUE(queue.setBootNonce(10));
  ASSERT_TRUE(queue.enqueue(line, 3));
  EXPECT_EQ(queue.headSequence(), 1U);
  EXPECT_EQ(queue.truncatedRecords(), 2U);
  EXPECT_EQ(queue.unavailableRecords(), 1U);
}

TEST(ObserverRecordQueue, LineTruncationUsesIndependentLossReporter) {
  observer::RecordQueue<16, 4> queue(true, 7);
  OverflowReporter truncations;
  observer::LineBuffer<4, 16, observer::RecordQueue<16, 4>, OverflowReporter> line_buffer(queue, truncations);
  const uint8_t line[] = "abcde\n";

  ASSERT_EQ(line_buffer.write(line, sizeof(line) - 1), sizeof(line) - 1);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(truncations.bytes, sizeof(line) - 1);
  EXPECT_EQ(queue.droppedRecords(), 0U);
  EXPECT_EQ(queue.evictedRecords(), 0U);
}

TEST(ObserverRecordQueue, WraparoundEvictsWholeHeadAndKeepsContiguousCompleteRecords) {
  observer::RecordQueue<16, 4> queue(true, 8);
  const uint8_t first[] = "AAAA\n";
  const uint8_t second[] = "BBBB\n";
  const uint8_t third[] = "CCCC\n";
  const uint8_t fourth[] = "DDDD\n";
  const uint8_t fifth[] = "EEEE\n";
  ASSERT_TRUE(queue.enqueue(first, sizeof(first) - 1));
  ASSERT_TRUE(queue.enqueue(second, sizeof(second) - 1));
  ASSERT_TRUE(queue.acknowledge(8, 1));
  ASSERT_TRUE(queue.enqueue(third, sizeof(third) - 1));
  ASSERT_TRUE(queue.enqueue(fourth, sizeof(fourth) - 1));
  ASSERT_TRUE(queue.enqueue(fifth, sizeof(fifth) - 1));
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(second) - 1);
  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, 16));
  EXPECT_EQ(head.sequence, 3U);
  EXPECT_EQ(std::vector<uint8_t>(head.bytes, head.bytes + head.length),
            (std::vector<uint8_t>(third, third + sizeof(third) - 1)));
}

TEST(ObserverRecordQueue, SuccessfulFragmentSendRetainsHeadUntilMatchingAck) {
  observer::RecordQueue<32, 2> queue(true);
  const uint8_t record[] = "RAW: AABB\r\n";
  ASSERT_TRUE(queue.enqueue(record, sizeof(record) - 1));

  observer::RecordFragment sent{};
  ASSERT_TRUE(queue.peek(sent, 4));
  queue.consume(sent.length);

  observer::RecordFragment retry{};
  ASSERT_TRUE(queue.peek(retry, 4));
  EXPECT_EQ(retry.sequence, sent.sequence);
  EXPECT_EQ(retry.offset, sent.length);
  EXPECT_EQ(retry.total_length, sizeof(record) - 1);
  queue.restartHead();
  ASSERT_TRUE(queue.peek(retry, 4));
  EXPECT_EQ(retry.offset, 0U);
  EXPECT_TRUE(queue.acknowledge(retry.boot_nonce, retry.sequence));
  EXPECT_TRUE(queue.empty());
}

TEST(ObserverRecordQueue, EvictingPartiallySentHeadPreservesNextWholeRecord) {
  observer::RecordQueue<32, 2> queue(true);
  const uint8_t first[] = "RAW: AABB\r\n";
  const uint8_t second[] = "RX: CCDD\r\n";
  const uint8_t third[] = "TX: EEFF\r\n";
  ASSERT_TRUE(queue.enqueue(first, sizeof(first) - 1));
  ASSERT_TRUE(queue.enqueue(second, sizeof(second) - 1));

  observer::RecordFragment partial{};
  ASSERT_TRUE(queue.peek(partial, 4));
  queue.consume(partial.length);
  ASSERT_TRUE(queue.enqueue(third, sizeof(third) - 1));

  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, 32));
  EXPECT_EQ(std::vector<uint8_t>(head.bytes, head.bytes + head.length),
            (std::vector<uint8_t>(second, second + sizeof(second) - 1)));
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(first) - 1);
  EXPECT_EQ(queue.evictedRecords(), 1U);
  EXPECT_EQ(queue.evictedBytes(), sizeof(first) - 1);
}

TEST(ObserverRecordQueue, LateNotifyCompletionCannotAdvanceReplacementHeadAfterEviction) {
  observer::RecordQueue<24, 2> queue(true, 19);
  const uint8_t first[] = "AAAAAAAA\n";
  const uint8_t second[] = "BBBBBBBB\n";
  const uint8_t third[] = "CCCCCCCC\n";
  ASSERT_TRUE(queue.enqueue(first, sizeof(first) - 1));
  ASSERT_TRUE(queue.enqueue(second, sizeof(second) - 1));
  observer::RecordFragment sent{};
  ASSERT_TRUE(queue.peek(sent, 4));
  ASSERT_TRUE(queue.enqueue(third, sizeof(third) - 1));  // evicts the in-flight head
  EXPECT_FALSE(queue.consume(sent.boot_nonce, sent.sequence, sent.length));
  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, 4));
  EXPECT_EQ(head.sequence, 2U);
  EXPECT_EQ(head.offset, 0U);
}

TEST(ObserverRecordQueue, RetainsDisconnectedRecordsUntilRebootButStartsNewQueueEmpty) {
  observer::RecordQueue<32, 4> disconnected(true, 0x0123456789abcdefULL);
  const uint8_t record[] = "RX\r\n";
  ASSERT_TRUE(disconnected.enqueue(record, sizeof(record) - 1));
  observer::RecordFragment fragment{};
  ASSERT_TRUE(disconnected.peek(fragment, 4));
  EXPECT_EQ(fragment.sequence, 1U);

  observer::RecordQueue<32, 4> after_reboot(true, 0xfedcba9876543210ULL);
  EXPECT_TRUE(after_reboot.empty());
  EXPECT_EQ(after_reboot.bootNonce(), 0xfedcba9876543210ULL);
  EXPECT_EQ(after_reboot.droppedRecords(), 0U);
  EXPECT_EQ(after_reboot.droppedBytes(), 0U);
}

TEST(ObserverProtocol, EncodesSixteenByteLittleEndianFrameWithFourByteMtu20Payload) {
  const uint8_t record[] = {'R', 'A', 'W', '\r', '\n'};
  uint8_t frame[20]{};
  const auto encoded = observer::encodeFrame(0x0123456789abcdefULL, 0x78563412U,
                                             sizeof(record), 0, record, 4, frame,
                                             sizeof(frame));
  ASSERT_EQ(encoded, 20U);
  EXPECT_EQ(std::vector<uint8_t>(frame, frame + 20),
            (std::vector<uint8_t>{0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01,
                                  0x12, 0x34, 0x56, 0x78, 5, 0, 0, 0,
                                  'R', 'A', 'W', '\r'}));
}

TEST(ObserverProtocol, FormatsCanonicalAckAndRejectsMalformedOrStaleAck) {
  char ack[96]{};
  observer::formatAck(0x0123456789abcdefULL, 1234567890U, ack, sizeof(ack));
  EXPECT_STREQ(ack, "ack 0123456789abcdef 1234567890");
  observer::Ack parsed{};
  EXPECT_TRUE(observer::parseAck(ack, strlen(ack), parsed));
  EXPECT_EQ(parsed.boot_nonce, 0x0123456789abcdefULL);
  EXPECT_EQ(parsed.sequence, 1234567890U);
  EXPECT_TRUE(observer::parseAck("ack 0000000000000000 1", strlen("ack 0000000000000000 1"), parsed));
  observer::RecordQueue<32, 2> zero_nonce_queue(true, 0);
  const uint8_t zero_record[] = "x\n";
  ASSERT_TRUE(zero_nonce_queue.enqueue(zero_record, sizeof(zero_record) - 1));
  EXPECT_TRUE(zero_nonce_queue.acknowledge(parsed.boot_nonce, parsed.sequence));
  EXPECT_FALSE(observer::parseAck("ack 0123456789ABCDEf 1", 22, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 01", 24, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef +1", 24, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 1\r", 23, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 1 ", 23, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 4294967296", 31, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 1", 96, parsed));
  EXPECT_FALSE(observer::parseAck("ack 0123456789abcdef 1", strlen("ack 0123456789abcdef 1") + 1, parsed));
  EXPECT_TRUE(observer::parseAck("ack 0123456789abcdef 4294967295", strlen("ack 0123456789abcdef 4294967295"), parsed));
  EXPECT_EQ(parsed.sequence, UINT32_MAX);
}

TEST(ObserverProtocol, FragmentFramesRemainOrderedAndSequenceIdentifiesEveryFragment) {
  const uint8_t record[] = "ABCDEFGH";
  uint8_t first[20]{};
  uint8_t second[20]{};
  ASSERT_EQ(observer::encodeFrame(7, 1, 8, 0, record, 4, first, sizeof(first)), 20U);
  ASSERT_EQ(observer::encodeFrame(7, 1, 8, 4, record + 4, 4, second, sizeof(second)), 20U);
  EXPECT_EQ(first[8], second[8]);
  EXPECT_EQ(first[12], second[12]);
  EXPECT_EQ(first[14], 0);
  EXPECT_EQ(second[14], 4);
  EXPECT_EQ(std::vector<uint8_t>(first + 16, first + 20), (std::vector<uint8_t>{'A','B','C','D'}));
  EXPECT_EQ(std::vector<uint8_t>(second + 16, second + 20), (std::vector<uint8_t>{'E','F','G','H'}));
}

TEST(ObserverRecordQueue, AckRequiresExactQueueHeadIdentityAndDuplicateCannotEvictNext) {
  observer::RecordQueue<32, 4> queue(true, 0x0123456789abcdefULL);
  const uint8_t first[] = "RAW\n";
  const uint8_t second[] = "RX\n";
  ASSERT_TRUE(queue.enqueue(first, sizeof(first) - 1));
  ASSERT_TRUE(queue.enqueue(second, sizeof(second) - 1));
  EXPECT_FALSE(queue.acknowledge(0, 1));
  EXPECT_FALSE(queue.acknowledge(0x0123456789abcdefULL, 2));
  EXPECT_FALSE(queue.acknowledge(0x0123456789abcdefULL, 0));
  EXPECT_TRUE(queue.acknowledge(0x0123456789abcdefULL, 1));
  EXPECT_FALSE(queue.acknowledge(0x0123456789abcdefULL, 1));
  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, 32));
  EXPECT_EQ(head.sequence, 2U);
}

TEST(ObserverRecordQueue, RecreatedQueuePreservesSequenceAndRejectsDelayedAck) {
  const uint64_t nonce = 0x1122334455667788ULL;
  observer::QueueSequenceState sequence;
  const uint8_t old_line[] = "old\n";
  observer::RecordQueue<32, 4> before_disable(true, nonce, sequence);
  ASSERT_TRUE(before_disable.enqueue(old_line, sizeof(old_line) - 1));
  const uint32_t old_sequence = before_disable.headSequence();

  observer::RecordQueue<32, 4> after_enable(true, nonce, sequence);
  const uint8_t new_line[] = "new\n";
  ASSERT_TRUE(after_enable.enqueue(new_line, sizeof(new_line) - 1));

  EXPECT_NE(after_enable.headSequence(), old_sequence);
  EXPECT_FALSE(after_enable.acknowledge(nonce, old_sequence));
  EXPECT_EQ(after_enable.size(), 1U);
}

TEST(ObserverRecordQueue, SequenceExhaustionStopsAdmissionUntilFreshNonceWithoutRelabelingBacklog) {
  observer::RecordQueue<32, 4> queue(true, 9, UINT32_MAX);
  const uint8_t record[] = "x\n";
  ASSERT_TRUE(queue.enqueue(record, sizeof(record) - 1));
  observer::RecordFragment before{};
  ASSERT_TRUE(queue.peek(before, 32));
  EXPECT_EQ(before.sequence, UINT32_MAX);
  EXPECT_FALSE(queue.enqueue(record, sizeof(record) - 1));
  EXPECT_EQ(queue.unavailableRecords(), 1U);
  EXPECT_EQ(queue.unavailableBytes(), sizeof(record) - 1);
  EXPECT_EQ(queue.truncatedRecords(), 0U);
  EXPECT_EQ(queue.evictedRecords(), 0U);
  queue.setBootNonce(10);  // loop-side fresh entropy, while the old head is retained
  ASSERT_TRUE(queue.enqueue(record, sizeof(record) - 1));
  ASSERT_TRUE(queue.peek(before, 32));
  EXPECT_EQ(before.boot_nonce, 9U);
  EXPECT_TRUE(queue.acknowledge(9, UINT32_MAX));
  observer::RecordFragment after{};
  ASSERT_TRUE(queue.peek(after, 32));
  EXPECT_EQ(after.boot_nonce, 10U);
  EXPECT_EQ(after.sequence, 1U);
  EXPECT_FALSE(queue.acknowledge(9, 1));
}

TEST(ObserverConnectionAdmission, UnapprovedPeerExpiresAndPinnedPeerDoesNot) {
  observer::ConnectionAdmission admission(5000);
  const observer::PeerIdentity pinned{1, {1, 2, 3, 4, 5, 6}};
  const observer::PeerIdentity foreign{1, {6, 5, 4, 3, 2, 1}};
  ASSERT_TRUE(admission.connected(50, 100));
  EXPECT_FALSE(admission.authorize(50, foreign, pinned));
  EXPECT_FALSE(admission.expired(5099));
  uint16_t disconnect_handle = 0;
  EXPECT_FALSE(admission.takeDisconnect(5099, disconnect_handle));
  EXPECT_TRUE(admission.takeDisconnect(5100, disconnect_handle));
  EXPECT_EQ(disconnect_handle, 50);

  ASSERT_TRUE(admission.connected(51, 200));
  EXPECT_TRUE(admission.authorize(51, pinned, pinned));
  EXPECT_FALSE(admission.takeDisconnect(10000, disconnect_handle));
  EXPECT_TRUE(admission.authorized());
}

TEST(ObserverTransportCallbacks, PendingCallbackBlocksQueueReleaseUntilLoopConsumesIt) {
  observer::CallbackEvents events;
  ASSERT_TRUE(events.postDisconnect(61));
  EXPECT_FALSE(events.safeToReleaseQueue(false, 0xFFFF));

  uint16_t disconnected_handle = 0;
  ASSERT_TRUE(events.takeDisconnect(disconnected_handle));
  EXPECT_EQ(disconnected_handle, 61);
  EXPECT_TRUE(events.safeToReleaseQueue(false, 0xFFFF));
}

TEST(ObserverTransportCallbacks, PendingAdmissionAlsoBlocksDisableTimeQueueRelease) {
  observer::CallbackEvents events;
  EXPECT_FALSE(events.safeToReleaseQueue(false, 0xFFFF, true));
  EXPECT_TRUE(events.safeToReleaseQueue(false, 0xFFFF, false));
}

TEST(ObserverTransportCallbacks, DisconnectHandoffLetsLoopRestartDelivery) {
  observer::CallbackEvents events;
  observer::DeliverySchedule schedule(1000, 10);
  schedule.attempted(10);
  schedule.fragmentSent(true);
  ASSERT_TRUE(events.postDisconnect(62));

  uint16_t disconnected_handle = 0;
  ASSERT_TRUE(events.takeDisconnect(disconnected_handle));
  schedule.disconnected();
  EXPECT_EQ(disconnected_handle, 62);
  EXPECT_TRUE(schedule.maySend(11));
  EXPECT_FALSE(schedule.restartWholeRecord());
}

TEST(ObserverTransportCallbacks, DisableWaitsForPendingConnectBeforeQueueRelease) {
  observer::CallbackEvents events;
  ASSERT_TRUE(events.postConnect(63));
  EXPECT_FALSE(events.safeToReleaseQueue(false, 0xFFFF));

  uint16_t handle = 0;
  ASSERT_TRUE(events.takeConnect(handle));
  EXPECT_EQ(handle, 63);
  EXPECT_TRUE(events.safeToReleaseQueue(false, 0xFFFF));
}

TEST(ObserverTransportCallbacks, DisconnectQueuedBeforePendingConnectRequiresLiveLinkCheck) {
  observer::CallbackEvents events;
  observer::ConnectionAdmission admission(5000);
  ASSERT_TRUE(events.postConnect(74));
  ASSERT_TRUE(events.postDisconnect(74));

  uint16_t handle = 0xFFFF;
  ASSERT_TRUE(events.takeDisconnect(handle));
  EXPECT_EQ(handle, 74);
  ASSERT_TRUE(events.takeConnect(handle));
  EXPECT_EQ(handle, 74);
  // Simulate service() checking Bluefruit.Connection(handle)->connected() on the
  // connect path after consuming the out-of-order events.
  const bool accepted = observer::admitsCandidate(
      true, true, observer::hasLiveConnection(true, false), false, true, true, false, {}, {});
  EXPECT_FALSE(accepted);
  if (accepted) admission.connected(handle, 101);
  EXPECT_FALSE(admission.pending());
}

TEST(ObserverConnectionAdmission, PublicStaticAndResolvedAddressIdentityIsKnownBeforeBonding) {
  const observer::PeerIdentity pinned{0, {1, 2, 3, 4, 5, 6}};
  const observer::PeerIdentity foreign{0, {6, 5, 4, 3, 2, 1}};
  EXPECT_TRUE(observer::identityKnownAtConnect(false, 0, 0));
  EXPECT_TRUE(observer::identityKnownAtConnect(false, 1, 0));
  EXPECT_TRUE(observer::identityKnownAtConnect(false, 2, 1));
  EXPECT_FALSE(observer::identityKnownAtConnect(false, 2, 0));
  EXPECT_FALSE(observer::identityKnownAtConnect(false, 3, 0));
  EXPECT_FALSE(observer::admitsCandidate(true, true, true, false, true, true,
      observer::identityKnownAtConnect(false, 0, 0), foreign, pinned));
}

TEST(ObserverConnectionAdmission, PublicStaticForeignIdentityIsRejectedBeforeTakingSlot) {
  const observer::PeerIdentity pinned{0, {1, 2, 3, 4, 5, 6}};
  const observer::PeerIdentity foreign{0, {6, 5, 4, 3, 2, 1}};
  ASSERT_TRUE(observer::identityKnownAtConnect(false, 0, 0));
  EXPECT_FALSE(observer::admitsCandidate(true, true, true, false, true, true, true, foreign, pinned));
}

TEST(ObserverConnectionAdmission, NoPinnedPeerAndNotEnrollingRejectsCandidate) {
  EXPECT_FALSE(observer::admitsCandidate(true, true, true, false, true, false, false, {}, {}));
}

TEST(ObserverConnectionAdmission, UnresolvedCandidateDisconnectHandoffReleasesSlotWithinBound) {
  // An unresolved link holds the physical slot until its timeout-triggered
  // disconnect event is consumed; the pinned peer then takes the slot.
  const uint32_t timeout_ms = 5000;
  const observer::PeerIdentity pinned{1, {1, 2, 3, 4, 5, 6}};
  const observer::PeerIdentity foreign{1, {6, 5, 4, 3, 2, 1}};
  observer::Policy policy(true, pinned, true);
  observer::ConnectionAdmission admission(timeout_ms);
  observer::CallbackEvents events;
  uint16_t active_handle = 0xFFFF;

  ASSERT_TRUE(observer::admitsCandidate(true, true, true, false, true, true, false, {}, pinned));
  ASSERT_TRUE(admission.connected(100, 0));
  active_handle = 100;
  EXPECT_FALSE(observer::admitsCandidate(true, !admission.pending(), true, false, true, true, false, {}, pinned));
  EXPECT_FALSE(admission.authorize(100, foreign, pinned));
  EXPECT_FALSE(admission.authorized());
  EXPECT_FALSE(policy.authorizes(false, foreign, 0));
  EXPECT_FALSE(policy.authorizes(true, foreign, 0));
  EXPECT_FALSE(policy.authorizes(false, pinned, 0));

  uint16_t released = 0;
  EXPECT_FALSE(admission.takeDisconnect(timeout_ms - 1, released));
  ASSERT_TRUE(admission.takeDisconnect(timeout_ms, released));
  EXPECT_EQ(released, 100);
  EXPECT_FALSE(observer::admitsCandidate(true, active_handle == 0xFFFF, true, false, true, true, true, pinned, pinned));

  // The timeout initiates disconnect; only its callback frees the active handle.
  ASSERT_TRUE(events.postDisconnect(released));
  uint16_t disconnected = 0xFFFF;
  ASSERT_TRUE(events.takeDisconnect(disconnected));
  EXPECT_EQ(disconnected, 100);
  if (active_handle == disconnected) active_handle = 0xFFFF;
  admission.disconnected(disconnected);
  EXPECT_EQ(active_handle, 0xFFFF);

  ASSERT_TRUE(observer::admitsCandidate(true, active_handle == 0xFFFF, true, false, true, true, true, pinned, pinned));
  ASSERT_TRUE(admission.connected(101, timeout_ms));
  active_handle = 101;
  EXPECT_TRUE(admission.authorize(101, pinned, pinned));
  EXPECT_TRUE(admission.authorized());
  EXPECT_FALSE(admission.pending());
  EXPECT_EQ(active_handle, 101);
}

TEST(ObserverStatsSnapshot, GattReadChangesOnlyAfterLoopPublishesCounters) {
  observer::StatsSnapshot snapshot;
  observer::RecordQueue<16, 4> queue(true, 9);
  const uint8_t line[] = "123456789\n";
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));

  const observer::StatsValues before = snapshot.read();  // GATT callback read
  EXPECT_EQ(before.evicted_records, 0U);
  const uint32_t loop_evicted = queue.evictedRecords(); // main-loop-owned access
  snapshot.publish(loop_evicted, queue.evictedBytes(), queue.truncatedRecords(),
                   queue.truncatedBytes(), 0);
  const observer::StatsValues after = snapshot.read();
  EXPECT_EQ(after.evicted_records, loop_evicted);
  EXPECT_GT(after.evicted_records, before.evicted_records);
}

TEST(ObserverStatsSnapshot, SnapshotsDoNotTrackLiveQueueUntilPublish) {
  observer::StatsSnapshot snapshot;
  observer::RecordQueue<32, 4> queue(true, 9);
  const uint8_t line[] = "123456789\n";
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));  // 10 bytes
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));  // 20 bytes
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));  // 30 bytes, still fits
  ASSERT_EQ(queue.evictedRecords(), 0U);

  const observer::StatsValues before = snapshot.read();
  EXPECT_EQ(before.evicted_records, 0U);

  // Mutate queue further without publishing; callback still sees snapshot values.
  ASSERT_TRUE(queue.enqueue(line, sizeof(line) - 1));  // overflows 32-byte capacity
  EXPECT_EQ(queue.evictedRecords(), 1U);
  const observer::StatsValues after_mutation = snapshot.read();
  EXPECT_EQ(after_mutation.evicted_records, before.evicted_records);

  const uint32_t evicted = queue.evictedRecords();
  const uint32_t evicted_bytes = queue.evictedBytes();
  const uint32_t truncated = queue.truncatedRecords();
  const uint32_t truncated_bytes = queue.truncatedBytes();
  snapshot.publish(evicted, evicted_bytes, truncated, truncated_bytes, 2);
  const observer::StatsValues published = snapshot.read();
  EXPECT_EQ(published.evicted_records, evicted);
  EXPECT_EQ(published.evicted_bytes, evicted_bytes);
  EXPECT_EQ(published.truncated_records, truncated);
  EXPECT_EQ(published.truncated_bytes, truncated_bytes);
  EXPECT_EQ(published.failed_notifications, 2U);
}

TEST(ObserverStatsSnapshot, GattReadReturnsOnlyPublishedValuesWithoutQueueAccess) {
  observer::StatsSnapshot snapshot;
  const observer::StatsValues initial = snapshot.read();
  EXPECT_EQ(initial.evicted_records, 0U);
  EXPECT_EQ(initial.evicted_bytes, 0U);
  EXPECT_EQ(initial.truncated_records, 0U);
  EXPECT_EQ(initial.truncated_bytes, 0U);
  EXPECT_EQ(initial.failed_notifications, 0U);

  snapshot.publish(3, 120, 4, 500, 2);
  const observer::StatsValues callback_read = snapshot.read();
  EXPECT_EQ(callback_read.evicted_records, 3U);
  EXPECT_EQ(callback_read.evicted_bytes, 120U);
  EXPECT_EQ(callback_read.truncated_records, 4U);
  EXPECT_EQ(callback_read.truncated_bytes, 500U);
  EXPECT_EQ(callback_read.failed_notifications, 2U);
}

TEST(ObserverStatsSnapshot, GattReadFormatsAllPublishedCounters) {
  observer::StatsSnapshot snapshot;
  snapshot.publish(3, 120, 4, 500, 2);
  const observer::StatsValues values = snapshot.read();
  char buffer[96]{};
  const size_t length = observer::formatStats(values, buffer, sizeof(buffer));
  ASSERT_GT(length, 0U);
  EXPECT_EQ(std::string(buffer, length), "evicted=3/120 truncated=4/500 failed=2 unavailable=0/0");
}

TEST(ObserverStatsSnapshot, AdmissionLossIsSeparateAndMaximumCountersFitGattValue) {
  observer::StatsSnapshot snapshot;
  snapshot.publish(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
                   UINT32_MAX, UINT32_MAX);
  const observer::StatsValues values = snapshot.read();
  EXPECT_EQ(values.unavailable_records, UINT32_MAX);
  EXPECT_EQ(values.unavailable_bytes, UINT32_MAX);
  char buffer[128]{};
  const size_t length = observer::formatStats(values, buffer, sizeof(buffer));
  EXPECT_LT(length, sizeof(buffer));
  EXPECT_EQ(std::string(buffer).size(), length);
  EXPECT_NE(std::string(buffer).find("unavailable=4294967295/4294967295"), std::string::npos);
}

TEST(ObserverRecordQueue, ExhaustionSurvivesDisableAndCannotUseZeroOrSameNonce) {
  observer::QueueSequenceState state;
  state.advance(UINT32_MAX);
  observer::RecordQueue<32, 4> recreated(true, 9, state);
  const uint8_t line[] = "x\n";
  EXPECT_FALSE(recreated.enqueue(line, sizeof(line) - 1));
  EXPECT_FALSE(recreated.setBootNonce(0));
  EXPECT_FALSE(recreated.setBootNonce(9));
  ASSERT_TRUE(recreated.setBootNonce(10));
  ASSERT_TRUE(recreated.enqueue(line, sizeof(line) - 1));
  EXPECT_FALSE(recreated.setBootNonce(11)); // not exhausted: no arbitrary relabel
  EXPECT_EQ(state.next(), 2U);
  EXPECT_EQ(recreated.bootNonce(), 10U);
}

TEST(ObserverDelivery, FailedNotifyAndDisconnectRetryWholeHeadUntilPiAck) {
  observer::RecordQueue<32, 4> queue(true, 42);
  const uint8_t record[] = "RAW: AABB\r\n";
  ASSERT_TRUE(queue.enqueue(record, sizeof(record) - 1));
  observer::RecordFragment initial{};
  ASSERT_TRUE(queue.peek(initial, 4));
  queue.restartHead(); // failed notify or disconnected link
  observer::RecordFragment retry{};
  ASSERT_TRUE(queue.peek(retry, 4));
  EXPECT_EQ(retry.sequence, initial.sequence);
  EXPECT_EQ(retry.offset, 0U);
  ASSERT_TRUE(queue.acknowledge(42, retry.sequence));
  EXPECT_TRUE(queue.empty());
}

TEST(ObserverDelivery, RetryCadenceResendsWholeRecordAndResetsOnAckOrDisconnect) {
  observer::DeliverySchedule delivery(1000);
  EXPECT_TRUE(delivery.maySend(10));
  delivery.attempted(10);
  delivery.fragmentSent(false);
  EXPECT_FALSE(delivery.maySend(19));
  EXPECT_TRUE(delivery.maySend(20));
  delivery.attempted(20);
  delivery.fragmentSent(true);
  EXPECT_FALSE(delivery.maySend(1019));
  EXPECT_TRUE(delivery.maySend(1020));
  EXPECT_TRUE(delivery.restartWholeRecord());
  delivery.attempted(1020);
  delivery.fragmentFailed();
  EXPECT_FALSE(delivery.maySend(1029));
  EXPECT_TRUE(delivery.maySend(1030));
  delivery.disconnected();
  EXPECT_TRUE(delivery.maySend(1030));
  delivery.attempted(1030);
  delivery.fragmentSent(true);
  delivery.acknowledged();
  EXPECT_FALSE(delivery.restartWholeRecord());
  EXPECT_TRUE(delivery.maySend(1031));
}

TEST(ObserverPolicy, PreferenceAndEnrollmentFailClosedAndPinOneBondIdentity) {
  observer::Policy policy;
  EXPECT_FALSE(policy.enabled());
  EXPECT_FALSE(policy.enrollmentOpen(0));
  EXPECT_FALSE(policy.authorizes(0, {1, 2, 3, 4, 5, 6}));

  policy.setEnabled(true);
  EXPECT_TRUE(policy.enabled());
  policy.openEnrollment(1000, 30000);
  EXPECT_TRUE(policy.enrollmentOpen(1001));
  const observer::PeerIdentity intended{1, {1, 2, 3, 4, 5, 6}};
  EXPECT_TRUE(policy.enroll(intended, 1001));
  EXPECT_TRUE(policy.authorizes(true, intended, 1002));
  EXPECT_FALSE(policy.authorizes(true, {1, {6, 5, 4, 3, 2, 1}}, 1002));
  EXPECT_FALSE(policy.enrollmentOpen(31000));
  policy.revoke();
  policy.openEnrollment(UINT32_MAX - 1000, 30000);
  EXPECT_TRUE(policy.enrollmentOpen(500));
  EXPECT_FALSE(policy.enrollmentOpen(30000));
  EXPECT_FALSE(policy.enroll({1, {6, 5, 4, 3, 2, 1}}, 31000));

  observer::Policy after_reboot(true, intended, true);
  EXPECT_TRUE(after_reboot.enabled());
  EXPECT_TRUE(after_reboot.authorizes(true, intended));
  after_reboot.revoke();
  EXPECT_FALSE(after_reboot.authorizes(true, intended));
  after_reboot.openEnrollment(40000, 30000);
  const observer::PeerIdentity replacement{0, {6, 5, 4, 3, 2, 1}};
  EXPECT_TRUE(after_reboot.enroll(replacement, 40001));
  EXPECT_TRUE(after_reboot.authorizes(true, replacement, 40002));
}

TEST(ObserverDelivery, RepeatedAckIngressDoesNotReplaceHostCommandSlot) {
  observer::CommandIngress commands;
  observer::AckIngress acknowledgements;
  EXPECT_TRUE(commands.submitCommand("get name", strlen("get name")));
  const char* first_ack = "ack 000000000000002a 1";
  const char* second_ack = "ack 000000000000002a 2";
  EXPECT_TRUE(acknowledgements.submit(first_ack, strlen(first_ack)));
  EXPECT_FALSE(acknowledgements.submit(second_ack, strlen(second_ack)));
  char command[96]{};
  ASSERT_TRUE(commands.takeCommand(command, sizeof(command)));
  EXPECT_STREQ(command, "get name");
  char handled_ack[96]{};
  size_t handled_length = 0;
  ASSERT_TRUE(acknowledgements.take(handled_ack, sizeof(handled_ack), handled_length));
  EXPECT_EQ(std::string(handled_ack, handled_length), first_ack);
  EXPECT_TRUE(acknowledgements.submit(second_ack, strlen(second_ack)));
  ASSERT_TRUE(acknowledgements.take(handled_ack, sizeof(handled_ack), handled_length));
  EXPECT_EQ(std::string(handled_ack, handled_length), second_ack);
}

TEST(ObserverDelivery, HostCommandIngressRejectsEmbeddedNulInsteadOfTruncatingCommand) {
  observer::CommandIngress commands;
  const char smuggled[] = {'g', 'e', 't', ' ', 'n', 'a', 'm', 'e', '\0', 'x'};
  EXPECT_FALSE(commands.submitCommand(smuggled, sizeof(smuggled)));
  char output[96]{};
  EXPECT_FALSE(commands.takeCommand(output, sizeof(output)));
}

TEST(ObserverProtocol, RawEvictionCreatesSequenceGapWithoutMutatingRemainingLines) {
  observer::RecordQueue<48, 4> queue(true, 77);
  const uint8_t raw[] = "RAW: AABB\r\n";
  const uint8_t rx[] = "RX: CCDD\r\n";
  const uint8_t tx[] = "TX: EEFF\r\n";
  ASSERT_TRUE(queue.enqueue(raw, sizeof(raw) - 1));
  ASSERT_TRUE(queue.enqueue(rx, sizeof(rx) - 1));
  ASSERT_TRUE(queue.enqueue(tx, sizeof(tx) - 1));
  observer::RecordFragment first{};
  ASSERT_TRUE(queue.peek(first, 4));
  queue.dropHeadRecord();
  observer::RecordFragment next{};
  ASSERT_TRUE(queue.peek(next, 32));
  EXPECT_EQ(next.sequence, first.sequence + 1);
  EXPECT_EQ(std::vector<uint8_t>(next.bytes, next.bytes + next.length),
            (std::vector<uint8_t>(rx, rx + sizeof(rx) - 1)));
  EXPECT_EQ(queue.droppedRecords(), 1U);
  EXPECT_EQ(queue.droppedBytes(), sizeof(raw) - 1);
}

class MemoryPreferenceStore : public observer::PreferenceStore {
 public:
  bool read(const char* key, uint8_t* output, size_t length) override {
    for (const auto& entry : values) {
      if (entry.key == key && entry.length == length) {
        memcpy(output, entry.value, length);
        return true;
      }
    }
    return false;
  }
  bool write(const char* key, const uint8_t* value, size_t length) override {
    remove(key);
    Entry entry{key, {}, length};
    memcpy(entry.value, value, length);
    values.push_back(entry);
    return true;
  }
  bool remove(const char* key) override {
    values.erase(std::remove_if(values.begin(), values.end(), [&](const Entry& entry) {
      return entry.key == key;
    }), values.end());
    return true;
  }
 private:
  struct Entry { std::string key; uint8_t value[7]{}; size_t length; };
  std::vector<Entry> values;
};

TEST(ObserverPreferences, MissingAndOldDataDefaultOffAndOptInSurvivesReload) {
  MemoryPreferenceStore store;
  observer::Preferences preferences(store);
  bool enabled = true;
  bool has_peer = true;
  observer::PeerIdentity peer{};
  ASSERT_TRUE(preferences.load(enabled, peer, has_peer));
  EXPECT_FALSE(enabled);
  EXPECT_FALSE(has_peer);
  observer::RecordQueue<16, 2> disabled_queue(false);
  const uint8_t record[] = "RAW\r\n";
  ASSERT_TRUE(disabled_queue.enqueue(record, sizeof(record) - 1));
  EXPECT_TRUE(disabled_queue.empty());

  const uint8_t old_version = 99;
  ASSERT_TRUE(store.write("observer_v", &old_version, 1));
  EXPECT_FALSE(preferences.load(enabled, peer, has_peer));
  EXPECT_FALSE(enabled);

  ASSERT_TRUE(preferences.saveEnabled(true));
  observer::Preferences after_reboot(store);
  ASSERT_TRUE(after_reboot.load(enabled, peer, has_peer));
  EXPECT_TRUE(enabled);
  EXPECT_FALSE(has_peer);
}

TEST(ObserverPreferences, PersistedPeerIdentityCanBeRevokedAndReplaced) {
  MemoryPreferenceStore store;
  observer::Preferences preferences(store);
  const observer::PeerIdentity intended{1, {1, 2, 3, 4, 5, 6}};
  ASSERT_TRUE(preferences.savePeer(true, intended));
  observer::PeerIdentity restored{};
  bool enabled = false;
  bool has_peer = false;
  ASSERT_TRUE(preferences.load(enabled, restored, has_peer));
  EXPECT_TRUE(has_peer);
  EXPECT_TRUE(observer::samePeer(intended, restored));
  ASSERT_TRUE(preferences.savePeer(false, {}));
  ASSERT_TRUE(preferences.savePeer(true, {0, {6, 5, 4, 3, 2, 1}}));
  ASSERT_TRUE(preferences.load(enabled, restored, has_peer));
  EXPECT_EQ(restored.type, 0);
  EXPECT_EQ(restored.address[0], 6);
}

class ObserverPrefsStream : public Stream {
 public:
  explicit ObserverPrefsStream(std::string input = {}) : input_(std::move(input)) {}

  int available() override { return static_cast<int>(input_.size() - position_); }
  int read() override { return position_ < input_.size() ? input_[position_++] : -1; }
  int peek() override { return position_ < input_.size() ? input_[position_] : -1; }
  size_t write(uint8_t byte) override {
    if (fail_closing_brace_ && byte == '}') return 0;
    output_.push_back(static_cast<char>(byte));
    return 1;
  }
  size_t print(unsigned char value, int = DEC) override { return printNumber(value); }
  size_t print(int value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned int value, int = DEC) override { return printNumber(value); }
  size_t print(long value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned long value, int = DEC) override { return printNumber(value); }
  size_t print(long long value, int = DEC) override { return printNumber(value); }
  size_t print(unsigned long long value, int = DEC) override { return printNumber(value); }
  size_t print(double value, int precision = 2) override {
    char text[48];
    snprintf(text, sizeof(text), "%.*f", precision, value);
    return Print::print(text);
  }

  void failOnClosingBrace() { fail_closing_brace_ = true; }
  const std::string& output() const { return output_; }

 private:
  template <typename T>
  size_t printNumber(T value) {
    char text[48];
    snprintf(text, sizeof(text), "%lld", static_cast<long long>(value));
    return Print::print(text);
  }

  std::string input_;
  std::string output_;
  size_t position_ = 0;
  bool fail_closing_brace_ = false;
};

TEST(ObserverNodePrefs, MissingUnknownVersionAndEmptyCustomStayDisabled) {
  observer::ObserverNodePrefs missing;
  ObserverPrefsStream empty_custom("{custom:{}}");
  ASSERT_TRUE(missing.loadSerial(empty_custom));
  EXPECT_EQ(missing.observer_version_, 0);
  EXPECT_EQ(missing.observer_enabled, 0);
  EXPECT_EQ(missing.observer_peer_present, 0);
  EXPECT_FALSE(missing.observer_version_ == observer::ObserverNodePrefs::kObserverVersion &&
               missing.observer_enabled == 1);

  observer::ObserverNodePrefs unknown;
  ObserverPrefsStream unknown_version(
      "{observer_v:99,observer_en:1,observer_p_on:1,custom:{}}");
  ASSERT_TRUE(unknown.loadSerial(unknown_version));
  EXPECT_FALSE(unknown.observer_version_ == observer::ObserverNodePrefs::kObserverVersion &&
               unknown.observer_enabled == 1);
}

TEST(ObserverNodePrefs, ObserverAndUpstreamDynamicPrefsRoundTripThroughNodePrefs) {
  observer::ObserverNodePrefs saved;
  NodePrefs* common_prefs = &saved;
  saved.observer_version_ = observer::ObserverNodePrefs::kObserverVersion;
  saved.observer_enabled = 1;
  saved.observer_peer_present = 1;
  saved.observer_peer_type = 1;
  const uint8_t address[] = {1, 2, 3, 4, 5, 6};
  memcpy(saved.observer_peer_address, address, sizeof(address));
  common_prefs->getRadioPrefs()->setFreq(868.3f);
  common_prefs->getRadioPrefs()->setBandwidth(125.0f);
  ASSERT_TRUE(common_prefs->getCustom()->setByKey("site7", "active"));
  ASSERT_TRUE(common_prefs->isDirty());

  ObserverPrefsStream serialized;
  ASSERT_TRUE(common_prefs->saveSerial(serialized));
  common_prefs->clearDirty();
  EXPECT_FALSE(common_prefs->isDirty());

  observer::ObserverNodePrefs reloaded;
  NodePrefs* reloaded_common = &reloaded;
  ObserverPrefsStream input(serialized.output());
  ASSERT_TRUE(reloaded_common->loadSerial(input));
  EXPECT_EQ(reloaded.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(reloaded.observer_enabled, 1);
  EXPECT_EQ(reloaded.observer_peer_present, 1);
  EXPECT_EQ(reloaded.observer_peer_type, 1);
  EXPECT_EQ(0, memcmp(reloaded.observer_peer_address, address, sizeof(address)));
  EXPECT_FLOAT_EQ(868.3f, reloaded_common->getRadioPrefs()->getFreq());
  EXPECT_FLOAT_EQ(125.0f, reloaded_common->getRadioPrefs()->getBandwidth());
  char custom_value[32]{};
  ASSERT_TRUE(reloaded_common->getCustom()->getByKey("site7", custom_value, sizeof(custom_value) - 1));
  EXPECT_STREQ("active", custom_value);
}

TEST(ObserverNodePrefs, FailedNodePrefsSaveRollsBackObserverState) {
  observer::ObserverNodePrefs prefs;
  NodePrefs* common_prefs = &prefs;
  common_prefs->getRadioPrefs()->setFreq(915.0f);
  ASSERT_TRUE(common_prefs->getCustom()->setByKey("board7", "indoor"));
  ASSERT_TRUE(common_prefs->isDirty());
  prefs.observer_version_ = 0;
  prefs.observer_enabled = 0;
  prefs.observer_peer_present = 1;
  prefs.observer_peer_type = 2;
  const uint8_t address[] = {6, 5, 4, 3, 2, 1};
  memcpy(prefs.observer_peer_address, address, sizeof(address));
  const observer::PeerIdentity requested{1, {1, 2, 3, 4, 5, 6}};

  const bool saved = prefs.persistPeer(true, requested, [&] {
    ObserverPrefsStream failed;
    failed.failOnClosingBrace();
    return common_prefs->saveSerial(failed);
  });

  EXPECT_FALSE(saved);
  EXPECT_EQ(prefs.observer_version_, 0);
  EXPECT_EQ(prefs.observer_enabled, 0);
  EXPECT_EQ(prefs.observer_peer_present, 1);
  EXPECT_EQ(prefs.observer_peer_type, 2);
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, address, sizeof(address)));
  EXPECT_TRUE(common_prefs->isDirty());
  EXPECT_FLOAT_EQ(915.0f, common_prefs->getRadioPrefs()->getFreq());
  char custom_value[32]{};
  ASSERT_TRUE(common_prefs->getCustom()->getByKey("board7", custom_value, sizeof(custom_value) - 1));
  EXPECT_STREQ("indoor", custom_value);
}

TEST(ObserverNodePrefs, VersionIsStampedBeforeBasePointerSaveAndEnableRollsBackOnFailure) {
  observer::ObserverNodePrefs prefs;
  NodePrefs* common_prefs = &prefs;
  ObserverPrefsStream saved;

  ASSERT_TRUE(prefs.persistEnabled(true, [&] { return common_prefs->saveSerial(saved); }));
  observer::ObserverNodePrefs reloaded;
  ObserverPrefsStream input(saved.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&reloaded)->loadSerial(input));
  EXPECT_EQ(reloaded.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(reloaded.observer_enabled, 1);

  const bool failed = prefs.persistEnabled(false, [&] {
    ObserverPrefsStream failed_stream;
    failed_stream.failOnClosingBrace();
    return common_prefs->saveSerial(failed_stream);
  });
  EXPECT_FALSE(failed);
  EXPECT_EQ(prefs.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(prefs.observer_enabled, 1);
}

TEST(ObserverNodePrefs, EnablingUnknownSchemaClearsUntrustedPeerBeforeSave) {
  const uint8_t old_address[] = {1, 2, 3, 4, 5, 6};
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream legacy(
      "{observer_v:99,observer_en:0,observer_p_on:1,observer_p_type:1,"
      "observer_p_addr:\"010203040506\",custom:{}}");
  ASSERT_TRUE(prefs.loadSerial(legacy));
  ASSERT_EQ(prefs.observer_version_, 99);
  ASSERT_EQ(prefs.observer_peer_present, 1);
  ASSERT_EQ(0, memcmp(prefs.observer_peer_address, old_address, sizeof(old_address)));

  NodePrefs* common_prefs = &prefs;
  ObserverPrefsStream saved;
  ASSERT_TRUE(prefs.persistEnabled(true, [&] { return common_prefs->saveSerial(saved); }));

  observer::ObserverNodePrefs reloaded;
  ObserverPrefsStream input(saved.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&reloaded)->loadSerial(input));
  EXPECT_EQ(reloaded.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(reloaded.observer_enabled, 1);
  EXPECT_EQ(reloaded.observer_peer_present, 0);
  EXPECT_EQ(reloaded.observer_peer_type, 0);
  const uint8_t cleared_address[sizeof(old_address)]{};
  EXPECT_EQ(0, memcmp(reloaded.observer_peer_address, cleared_address, sizeof(cleared_address)));
}

TEST(ObserverNodePrefs, FailedEnableFromUnknownSchemaRestoresPeerAndUpstreamDirtyPrefs) {
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream legacy(
      "{observer_v:99,observer_en:0,observer_p_on:1,observer_p_type:1,"
      "observer_p_addr:\"010203040506\",custom:{}}");
  ASSERT_TRUE(prefs.loadSerial(legacy));
  NodePrefs* common_prefs = &prefs;
  common_prefs->getRadioPrefs()->setFreq(915.0f);
  ASSERT_TRUE(common_prefs->getCustom()->setByKey("board7", "outdoor"));

  ObserverPrefsStream failed_stream;
  failed_stream.failOnClosingBrace();
  EXPECT_FALSE(prefs.persistEnabled(true, [&] { return common_prefs->saveSerial(failed_stream); }));

  EXPECT_EQ(prefs.observer_version_, 99);
  EXPECT_EQ(prefs.observer_enabled, 0);
  EXPECT_EQ(prefs.observer_peer_present, 1);
  EXPECT_EQ(prefs.observer_peer_type, 1);
  const uint8_t legacy_address[] = {1, 2, 3, 4, 5, 6};
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, legacy_address, sizeof(legacy_address)));
  EXPECT_TRUE(common_prefs->isDirty());
  EXPECT_FLOAT_EQ(915.0f, common_prefs->getRadioPrefs()->getFreq());
  char custom_value[32]{};
  ASSERT_TRUE(common_prefs->getCustom()->getByKey("board7", custom_value, sizeof(custom_value) - 1));
  EXPECT_STREQ("outdoor", custom_value);
}

TEST(ObserverNodePrefs, SuccessfulPeerPersistenceRoundTripsCurrentSchema) {
  observer::ObserverNodePrefs prefs;
  NodePrefs* common_prefs = &prefs;
  const observer::PeerIdentity intended{1, {6, 5, 4, 3, 2, 1}};
  ObserverPrefsStream saved;

  ASSERT_TRUE(prefs.persistPeer(true, intended, [&] { return common_prefs->saveSerial(saved); }));
  observer::ObserverNodePrefs reloaded;
  ObserverPrefsStream input(saved.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&reloaded)->loadSerial(input));
  EXPECT_EQ(reloaded.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(reloaded.observer_peer_present, 1);
  EXPECT_EQ(reloaded.observer_peer_type, intended.type);
  EXPECT_EQ(0, memcmp(reloaded.observer_peer_address, intended.address, sizeof(intended.address)));
}

TEST(ObserverCommand, ReadAllowlistHasExactNamesAndRejectsMutations) {
  const char* allowed[] = {
      "get name", "get public.key", "get prv.key", "get radio", "ver", "board",
      "stats-core", "stats-radio", "stats-packets"};
  for (const char* command : allowed) EXPECT_TRUE(observer::isAllowedReadCommand(command)) << command;

  const char* rejected[] = {
      "get radio extra", "set radio 915 125 7 5", "set tx 20", "erase config",
      "start ota", "observer enable", "get name ", "stats-packets extra"};
  for (const char* command : rejected) EXPECT_FALSE(observer::isAllowedReadCommand(command)) << command;
}

TEST(ObserverPolicy, EnrollmentIsRejectedByDefaultAndSecondPeerCannotReplacePinnedPeer) {
  observer::Policy policy(false);
  const observer::PeerIdentity intended{1, {1, 2, 3, 4, 5, 6}};
  policy.openEnrollment(100, 1000);
  EXPECT_FALSE(policy.enroll(intended, 101));

  policy.setEnabled(true);
  policy.openEnrollment(200, 1000);
  ASSERT_TRUE(policy.enroll(intended, 201));
  policy.openEnrollment(300, 1000);
  EXPECT_FALSE(policy.enroll({1, {6, 5, 4, 3, 2, 1}}, 301));
  policy.closeEnrollment();
  EXPECT_TRUE(policy.authorizes(true, intended, 302));
  EXPECT_FALSE(policy.authorizes(true, {1, {6, 5, 4, 3, 2, 1}}, 302));
}

// Coordinated 544-byte record ceiling. The live P1-Pro trace produced RAW
// records up to 541 bytes (terminator included) and truncated 51 real lines at
// the old 256-byte capture limit; 544 bytes is the smallest coordinated cap
// with a 244-byte notification MTU (228-byte payloads). These tests bind the
// production constants only: the protocol ceiling, the production queue shape,
// the transport LineBuffer guard, and the encode limit mirrored by the host's
// MAX_RECORD_BYTES. No test-only template capacity may fake the guard.

TEST(ObserverProtocolRecordLimit, EncodeFrameAdmitsObservedMaxRecordAndRejectsOneByteOverCeiling) {
  EXPECT_EQ(observer::kMaximumRecordBytes, 544U);
  const uint8_t payload[] = "RAW: AABB";
  uint8_t encoded[observer::kFrameHeaderBytes + sizeof(payload)] = {};
  EXPECT_EQ(observer::encodeFrame(7, 1, 541, 0, payload, sizeof(payload) - 1, encoded, sizeof(encoded)),
            observer::kFrameHeaderBytes + sizeof(payload) - 1);
  EXPECT_EQ(observer::encodeFrame(7, 1, 544, 0, payload, sizeof(payload) - 1, encoded, sizeof(encoded)),
            observer::kFrameHeaderBytes + sizeof(payload) - 1);
  EXPECT_EQ(observer::encodeFrame(7, 1, 545, 0, payload, sizeof(payload) - 1, encoded, sizeof(encoded)), 0U);
}

TEST(ObserverRecordQueue, ProductionQueueShapeHoldsObservedMaxRawAndNewCeilingRecords) {
  observer::RecordQueue<2048, 16> queue(true, 7);  // production Transport queue shape
  const std::string raw_541 = std::string(540, 'A') + '\n';
  const std::string raw_544 = std::string(543, 'B') + '\n';
  ASSERT_EQ(raw_541.size(), 541U);
  ASSERT_EQ(raw_544.size(), 544U);
  ASSERT_TRUE(queue.enqueue(reinterpret_cast<const uint8_t*>(raw_541.data()), raw_541.size()));
  ASSERT_TRUE(queue.enqueue(reinterpret_cast<const uint8_t*>(raw_544.data()), raw_544.size()));
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.truncatedRecords(), 0U);
  EXPECT_EQ(queue.droppedRecords(), 0U);
  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, observer::kMaximumRecordBytes));
  EXPECT_EQ(head.total_length, 541U);
  EXPECT_EQ(head.sequence, 1U);
  ASSERT_TRUE(queue.acknowledge(7, 1));
  ASSERT_TRUE(queue.peek(head, observer::kMaximumRecordBytes));
  EXPECT_EQ(head.total_length, 544U);
}

TEST(ObserverLineBuffer, ProductionRecordLimitCommitsObservedMaxLinesAndDropsWholeOverlongLineAtFullLength) {
  observer::RecordQueue<2048, 16> queue(true, 7);  // production Transport queue shape
  observer::LineBuffer<observer::kMaximumRecordBytes, 2048, decltype(queue)> buffer(queue);
  const std::string accepted_541 = std::string(540, 'a') + '\n';
  const std::string accepted_544 = std::string(543, 'b') + '\n';
  const std::string dropped_545 = std::string(544, 'c') + '\n';
  for (const std::string& line : {accepted_541, accepted_544, dropped_545})
    for (const uint8_t byte : line) buffer.write(byte);
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.truncatedRecords(), 1U);
  EXPECT_EQ(queue.truncatedBytes(), 545U);  // whole line including its newline
  EXPECT_EQ(queue.droppedRecords(), 1U);    // the truncation is the only recorded loss
  EXPECT_EQ(queue.droppedBytes(), 545U);
  EXPECT_EQ(queue.evictedRecords(), 0U);
  observer::RecordFragment head{};
  ASSERT_TRUE(queue.peek(head, observer::kMaximumRecordBytes));
  EXPECT_EQ(head.total_length, 541U);
  ASSERT_TRUE(queue.acknowledge(7, 1));
  ASSERT_TRUE(queue.peek(head, observer::kMaximumRecordBytes));
  EXPECT_EQ(head.total_length, 544U);
}

TEST_F(ObserverProductionLifecycle, ProductionTransportCapturesObservedMaxRawAndDropsWholeOverlongLine) {
  ready(true); connect();
  const std::string accepted_541 = std::string(540, 'a') + '\n';
  const std::string accepted_544 = std::string(543, 'b') + '\n';
  const std::string dropped_545 = std::string(544, 'c') + '\n';
  t.writeLog(reinterpret_cast<const uint8_t*>(accepted_541.data()), accepted_541.size());
  t.writeLog(reinterpret_cast<const uint8_t*>(accepted_544.data()), accepted_544.size());
  t.writeLog(reinterpret_cast<const uint8_t*>(dropped_545.data()), dropped_545.size());
  EXPECT_EQ(observer::TransportTestAccess::queueSize(t), 2U);
  const auto stats = observer::TransportTestAccess::stats(t);
  EXPECT_EQ(stats.truncated_records, 1U);
  EXPECT_EQ(stats.truncated_bytes, 545U);
  EXPECT_EQ(stats.evicted_records, 0U);
  EXPECT_EQ(stats.unavailable_records, 0U);
  EXPECT_EQ(observer::TransportTestAccess::headTotal(t), 541U);
  EXPECT_TRUE(observer::TransportTestAccess::acknowledgeHead(t));
  EXPECT_EQ(observer::TransportTestAccess::headTotal(t), 544U);
}

}  // namespace

#if defined(OBSERVER_REAL_COMMONCLI_TEST)
#define constrain(value, lower, upper) \
  ((value) < (lower) ? (lower) : ((value) > (upper) ? (upper) : (value)))
#define min(left, right) ((left) < (right) ? (left) : (right))
// The native source filter omits these constructors; provide only the default
// storage initializers needed to instantiate the unused CLI dependencies.
namespace mesh {
Identity::Identity() { memset(pub_key, 0, sizeof(pub_key)); }
}
RegionMap::RegionMap(TransportKeyStore& store) : _store(&store) {
  next_id = 1; num_regions = 0;
  default_id = home_id = 0;
  wildcard.id = wildcard.parent = 0;
  wildcard.flags = 0;
  strcpy(wildcard.name, "*");
}
#include "../../src/helpers/CommonCLI.cpp"
#undef min
#undef constrain

class ObserverPrefsTestBoard : public mesh::MainBoard {
 public:
  uint16_t getBattMilliVolts() override { return 4100; }
  const char* getManufacturerName() const override { return "test"; }
  void reboot() override {}
  uint8_t getStartupReason() const override { return 0; }
};

class ObserverPrefsTestRTC : public mesh::RTCClock {
 public:
  uint32_t getCurrentTime() override { return 0; }
  void setCurrentTime(uint32_t) override {}
};

TEST(ObserverNodePrefs, CommonCLIMigratesLegacyBinaryPrefsWithoutEnrollingObserver) {
  FILESYSTEM filesystem;
  std::string legacy(295, '\0');
  const char legacy_name[] = "legacy-node";
  const float frequency = 868.3f;
  const float bandwidth = 125.0f;
  memcpy(&legacy[4], legacy_name, sizeof(legacy_name));
  memcpy(&legacy[72], &frequency, sizeof(frequency));
  legacy[112] = 7;
  legacy[113] = 5;
  memcpy(&legacy[116], &bandwidth, sizeof(bandwidth));
  filesystem.addLegacyPrefs(legacy);

  observer::ObserverNodePrefs prefs;
  ObserverPrefsTestBoard board;
  ObserverPrefsTestRTC rtc;
  SensorManager sensors;
  TransportKeyStore key_store;
  RegionMap regions(key_store);
  ClientACL acl;
  CommonCLI cli(board, rtc, sensors, regions, acl, &prefs, nullptr);

  cli.loadPrefs(&filesystem);

  EXPECT_TRUE(filesystem.hasMigratedPrefs());
  EXPECT_STREQ(legacy_name, prefs.node_name);
  EXPECT_FLOAT_EQ(frequency, prefs.freq);
  EXPECT_EQ(prefs.sf, 7);
  EXPECT_EQ(prefs.cr, 5);
  EXPECT_FLOAT_EQ(bandwidth, prefs.bw);
  EXPECT_EQ(prefs.observer_version_, 0);
  EXPECT_EQ(prefs.observer_enabled, 0);
  EXPECT_EQ(prefs.observer_peer_present, 0);
  EXPECT_NE(std::string::npos, filesystem.migratedPrefs().find("observer_v:0"));

  observer::ObserverNodePrefs reloaded;
  ObserverPrefsStream input(filesystem.migratedPrefs());
  ASSERT_TRUE(reloaded.loadSerial(input));
  EXPECT_EQ(reloaded.observer_version_, 0);
  EXPECT_EQ(reloaded.observer_enabled, 0);
  EXPECT_EQ(reloaded.observer_peer_present, 0);
  EXPECT_STREQ(legacy_name, reloaded.node_name);
  EXPECT_FLOAT_EQ(frequency, reloaded.freq);
  EXPECT_FLOAT_EQ(bandwidth, reloaded.bw);
}
TEST(ObserverNodePrefs, BootStartupFromReloadedEnabledPeerWaitsForEntropyThenAdvertises) {
  Bluefruit = FakeBluefruit{}; Serial = FakeSerial{}; g_mock_millis = 0;
  fake_gatt_calls = fake_gatt_status = fake_notifications = 0;
  saved_enabled = false;

  observer::ObserverNodePrefs persisted;
  NodePrefs* persisted_common = &persisted;
  ObserverPrefsStream enabled_json;
  ASSERT_TRUE(persisted.persistEnabled(true, [&] { return persisted_common->saveSerial(enabled_json); }));

  observer::ObserverNodePrefs enabled;
  ObserverPrefsStream enabled_input(enabled_json.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&enabled)->loadSerial(enabled_input));
  const observer::PeerIdentity intended{1, {1, 2, 3, 4, 5, 6}};
  ObserverPrefsStream peer_json;
  ASSERT_TRUE(enabled.persistPeer(true, intended, [&] {
    return static_cast<NodePrefs*>(&enabled)->saveSerial(peer_json);
  }));

  observer::ObserverNodePrefs reloaded;
  ObserverPrefsStream peer_input(peer_json.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&reloaded)->loadSerial(peer_input));
  ASSERT_EQ(reloaded.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  ASSERT_EQ(reloaded.observer_enabled, 1);
  ASSERT_EQ(reloaded.observer_peer_present, 1);

  observer::Transport transport;
  observer::PeerIdentity peer{};
  peer.type = reloaded.observer_peer_type;
  memcpy(peer.address, reloaded.observer_peer_address, sizeof(peer.address));
  transport.attachPreferences(
      reloaded.observer_version_ == observer::ObserverNodePrefs::kObserverVersion && reloaded.observer_enabled == 1,
      peer,
      reloaded.observer_version_ == observer::ObserverNodePrefs::kObserverVersion && reloaded.observer_peer_present == 1);
  transport.setPreferenceCallbacks(saveEnabled, savePeer);

  ASSERT_TRUE(transport.beginFromPreferences("SenseCAP Observer"));
  EXPECT_EQ(Bluefruit.begins, 1);
  EXPECT_EQ(fake_gatt_calls, 5);
  transport.service();
  EXPECT_FALSE(observer::TransportTestAccess::allocated(transport));
  EXPECT_FALSE(Bluefruit.Advertising.running);

  Bluefruit.entropy = true;
  g_mock_millis += 100;
  transport.service();
  EXPECT_TRUE(observer::TransportTestAccess::allocated(transport));
  EXPECT_EQ(observer::TransportTestAccess::nonce(transport), 42U);
  EXPECT_TRUE(Bluefruit.Advertising.running);
  observer::TransportTestAccess::dispose(transport);
}

TEST(ObserverNodePrefs, BootStartupDoesNotInitializeMissingUnknownOrDisabledPreferences) {
  auto expect_no_start = [](observer::ObserverNodePrefs& prefs) {
    Bluefruit = FakeBluefruit{}; Serial = FakeSerial{}; g_mock_millis = 0;
    fake_gatt_calls = fake_gatt_status = fake_notifications = 0;
    observer::PeerIdentity peer{};
    peer.type = prefs.observer_peer_type;
    memcpy(peer.address, prefs.observer_peer_address, sizeof(peer.address));
    const bool enabled = prefs.observer_version_ == observer::ObserverNodePrefs::kObserverVersion &&
                         prefs.observer_enabled == 1;
    const bool has_peer = prefs.observer_version_ == observer::ObserverNodePrefs::kObserverVersion &&
                          prefs.observer_peer_present == 1;
    observer::Transport transport;
    transport.attachPreferences(enabled, peer, has_peer);
    transport.setPreferenceCallbacks(saveEnabled, savePeer);
    EXPECT_FALSE(transport.beginFromPreferences("SenseCAP Observer"));
    transport.service();
    EXPECT_EQ(Bluefruit.begins, 0);
    EXPECT_EQ(fake_gatt_calls, 0);
    EXPECT_EQ(Bluefruit.entropy_calls, 0);
    EXPECT_FALSE(observer::TransportTestAccess::allocated(transport));
    EXPECT_FALSE(Bluefruit.Advertising.running);
    observer::TransportTestAccess::dispose(transport);
  };

  observer::ObserverNodePrefs missing;
  expect_no_start(missing);

  observer::ObserverNodePrefs unknown;
  ObserverPrefsStream unknown_json(
      "{observer_v:99,observer_en:1,observer_p_on:1,observer_p_type:1,"
      "observer_p_addr:\"010203040506\",custom:{}}");
  ASSERT_TRUE(unknown.loadSerial(unknown_json));
  expect_no_start(unknown);

  observer::ObserverNodePrefs disabled;
  disabled.observer_version_ = observer::ObserverNodePrefs::kObserverVersion;
  disabled.observer_enabled = 0;
  expect_no_start(disabled);
}

#endif

// ---- Prefs written by firmware older than the observer_p_* key rename ----
// That firmware wrote observer_peer_on / observer_peer_type / observer_peer_addr and was
// built with CONFIG_MAX_KEYLEN=19. The format differs from today's only in those three names.
namespace {
std::string replaceAll(std::string text, const std::string& from, const std::string& to) {
  for (size_t at = 0; (at = text.find(from, at)) != std::string::npos; at += to.size()) {
    text.replace(at, from.size(), to);
  }
  return text;
}

std::string toOldFormat(std::string current) {
  current = replaceAll(current, "observer_p_on:", "observer_peer_on:");
  current = replaceAll(current, "observer_p_type:", "observer_peer_type:");
  return replaceAll(current, "observer_p_addr:", "observer_peer_addr:");
}

const uint8_t kLegacyAddress[] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};

// What the previous firmware stored for an enrolled host, plus upstream settings.
std::string previousFirmwareBlob() {
  observer::ObserverNodePrefs prefs;
  prefs.observer_version_ = observer::ObserverNodePrefs::kObserverVersion;
  prefs.observer_enabled = 1;
  prefs.observer_peer_present = 1;
  prefs.observer_peer_type = 1;
  memcpy(prefs.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress));
  NodePrefs* common = &prefs;
  common->getRadioPrefs()->setFreq(869.525f);
  common->getCustom()->setByKey("site7", "active");
  ObserverPrefsStream out;
  common->saveSerial(out);
  return toOldFormat(out.output());
}

void expectUpstreamSettings(NodePrefs* common) {
  EXPECT_FLOAT_EQ(869.525f, common->getRadioPrefs()->getFreq());
  char value[32]{};
  ASSERT_TRUE(common->getCustom()->getByKey("site7", value, sizeof(value) - 1));
  EXPECT_STREQ("active", value);
}
}  // namespace

TEST(ObserverLegacyPrefs, PreviousFirmwareBlobUsesTheOldNames) {
  const std::string blob = previousFirmwareBlob();
  EXPECT_NE(blob.find("observer_peer_on:1"), std::string::npos);
  EXPECT_NE(blob.find("observer_peer_type:1"), std::string::npos);
  EXPECT_NE(blob.find("observer_peer_addr:\"A1B2C3D4E5F6\""), std::string::npos);
  EXPECT_EQ(blob.find("observer_p_"), std::string::npos);
}

#ifdef OBSERVER_LEGACY_PREF_KEYS

TEST(ObserverLegacyPrefs, OldNamesLoadAsTheSamePeerAndKeepOtherSettings) {
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream input(previousFirmwareBlob());
  ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->loadSerial(input));
  EXPECT_EQ(prefs.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  EXPECT_EQ(prefs.observer_enabled, 1);
  EXPECT_EQ(prefs.observer_peer_present, 1);
  EXPECT_EQ(prefs.observer_peer_type, 1);
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress)));
  EXPECT_TRUE(prefs.legacyKeysUsed());
  expectUpstreamSettings(&prefs);
}

TEST(ObserverLegacyPrefs, SaveWritesOnlyNewNamesAndReloadsWithoutLegacyUse) {
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream input(previousFirmwareBlob());
  ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->loadSerial(input));
  ObserverPrefsStream saved;
  ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->saveSerial(saved));
  EXPECT_EQ(saved.output().find("observer_peer"), std::string::npos);
  EXPECT_NE(saved.output().find("observer_p_on:1"), std::string::npos);
  EXPECT_NE(saved.output().find("observer_p_type:1"), std::string::npos);
  EXPECT_NE(saved.output().find("observer_p_addr:\"A1B2C3D4E5F6\""), std::string::npos);

  observer::ObserverNodePrefs again;
  ObserverPrefsStream reread(saved.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&again)->loadSerial(reread));
  EXPECT_FALSE(again.legacyKeysUsed());
  EXPECT_EQ(again.observer_peer_present, 1);
  EXPECT_EQ(0, memcmp(again.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress)));
  expectUpstreamSettings(&again);
}

TEST(ObserverLegacyPrefs, NewNameBlobStillLoadsWithoutLegacyUse) {
  observer::ObserverNodePrefs source;
  source.observer_version_ = observer::ObserverNodePrefs::kObserverVersion;
  source.observer_enabled = 1;
  source.observer_peer_present = 1;
  source.observer_peer_type = 0;
  memcpy(source.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress));
  ObserverPrefsStream out;
  ASSERT_TRUE(static_cast<NodePrefs*>(&source)->saveSerial(out));

  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream input(out.output());
  ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->loadSerial(input));
  EXPECT_FALSE(prefs.legacyKeysUsed());
  EXPECT_EQ(prefs.observer_enabled, 1);
  EXPECT_EQ(prefs.observer_peer_present, 1);
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress)));
}

TEST(ObserverLegacyPrefs, NewNamesWinWhenBothArePresentInEitherOrder) {
  const std::string old_part =
      "observer_peer_on:0,observer_peer_type:2,observer_peer_addr:\"010203040506\"";
  const std::string new_part =
      "observer_p_on:1,observer_p_type:1,observer_p_addr:\"A1B2C3D4E5F6\"";
  for (const std::string& body : {old_part + "," + new_part, new_part + "," + old_part}) {
    observer::ObserverNodePrefs prefs;
    ObserverPrefsStream input("{observer_v:1,observer_en:1," + body + ",custom:{}}");
    ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->loadSerial(input)) << body;
    EXPECT_EQ(prefs.observer_peer_present, 1) << body;
    EXPECT_EQ(prefs.observer_peer_type, 1) << body;
    EXPECT_EQ(0, memcmp(prefs.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress))) << body;
    EXPECT_EQ(prefs.observer_enabled, 1);
    EXPECT_EQ(prefs.observer_version_, observer::ObserverNodePrefs::kObserverVersion);
  }
}

TEST(ObserverLegacyPrefs, PartialOldBlobUsesOldNameOnlyWhereNewIsAbsent) {
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream input(
      "{observer_v:1,observer_en:0,observer_peer_on:1,observer_p_type:1,"
      "observer_peer_addr:\"A1B2C3D4E5F6\",custom:{}}");
  ASSERT_TRUE(static_cast<NodePrefs*>(&prefs)->loadSerial(input));
  EXPECT_EQ(prefs.observer_enabled, 0);
  EXPECT_EQ(prefs.observer_peer_present, 1);
  EXPECT_EQ(prefs.observer_peer_type, 1);
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, kLegacyAddress, sizeof(kLegacyAddress)));
  EXPECT_TRUE(prefs.legacyKeysUsed());
}

#else  // default key limit: no migration compiled in

TEST(ObserverLegacyPrefs, DefaultLimitBuildDoesNotReadOldNames) {
  // Why the migration exists: with the default limit the 16th key character is a parse
  // error, so the load stops at the first old peer name and the enrolled host is lost.
  static_assert(CONFIG_MAX_KEYLEN == 16, "this test documents the default limit");
  observer::ObserverNodePrefs prefs;
  ObserverPrefsStream input(previousFirmwareBlob());
  EXPECT_FALSE(static_cast<NodePrefs*>(&prefs)->loadSerial(input));
  EXPECT_EQ(prefs.observer_peer_present, 0);
  EXPECT_EQ(prefs.observer_peer_type, 0);
  const uint8_t none[6]{};
  EXPECT_EQ(0, memcmp(prefs.observer_peer_address, none, sizeof(none)));
}

#endif


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
