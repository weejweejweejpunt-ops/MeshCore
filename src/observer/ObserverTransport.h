#pragma once

#include "ObserverStreamQueue.h"
#include "ObserverRecordQueue.h"
#include "ObserverLineBuffer.h"
#include "ObserverProtocol.h"
#include "ObserverPolicy.h"
#include "ObserverPreference.h"
#include "ObserverAckIngress.h"
#include "ObserverCommandIngress.h"
#include "ObserverCallbackState.h"
#include "ObserverConnectionAdmission.h"
#include "ObserverDelivery.h"
#include "ObserverStatsSnapshot.h"
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
#include "ObserverBleRuntime.h"
#include "coc/NordicPort.h"
#endif

#if defined(SENSECAP_OBSERVER_TRANSPORT)
#if !defined(OBSERVER_TRANSPORT_PLATFORM_TEST)
#include <Arduino.h>
#include <bluefruit.h>
#endif

namespace observer {

class LogMirror;

class Transport {
 public:
  static constexpr size_t kLogQueueBytes = 2048;
  static constexpr size_t kLogQueueRecords = 16;
  static constexpr size_t kCommandBytes = 96;
  static constexpr uint32_t kRetryIntervalMs = 1000;
  static constexpr uint32_t kFragmentIntervalMs = 10;
  static constexpr uint32_t kEnrollmentWindowMs = 60000;
  static constexpr uint32_t kUnauthenticatedTimeoutMs = 5000;
  static constexpr uint32_t kEntropyRetryMs = 100;

  explicit Transport(bool enabled = false);
  bool begin(const char* name, bool enabled);
  bool beginFromPreferences(const char* name);
  bool initialized() const { return initialized_; }
  void service();
  bool takeCommand(char* destination, size_t capacity);
  bool authorizeCoC(uint16_t connection_handle) const;
  static bool authorizeCoCLink(uint16_t connection_handle);
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  bool replyCoC(const uint8_t* bytes, size_t length);
#endif
  bool enable(bool enabled);
  bool enrollmentOpen() const { return policy_.enrollmentOpen(millis()); }
  bool revokePeer();
  bool beginEnrollment();
  bool closeEnrollment();
  uint32_t droppedRecords() const { return prequeue_unavailable_records_ + (log_queue_ ? log_queue_->droppedRecords() : 0); }
  uint32_t droppedBytes() const { return prequeue_unavailable_bytes_ + (log_queue_ ? log_queue_->droppedBytes() : 0); }
  uint32_t evictedRecords() const { return log_queue_ ? log_queue_->evictedRecords() : 0; }
  uint32_t evictedBytes() const { return log_queue_ ? log_queue_->evictedBytes() : 0; }
  uint32_t truncatedRecords() const { return log_queue_ ? log_queue_->truncatedRecords() : 0; }
  uint32_t truncatedBytes() const { return log_queue_ ? log_queue_->truncatedBytes() : 0; }
  uint32_t failedNotifications() const { return failed_notifications_; }
  void attachPreferences(bool enabled, const PeerIdentity& peer, bool has_peer);
  void setPreferenceCallbacks(bool (*save_enabled)(bool),
                              bool (*save_peer)(bool, const PeerIdentity&));
  size_t writeLog(const uint8_t* bytes, size_t length);

 private:
#if defined(OBSERVER_TRANSPORT_PLATFORM_TEST)
  friend struct TransportTestAccess;
#endif
  static Transport* instance_;
  BLEService service_;
  BLECharacteristic log_characteristic_;
  BLECharacteristic command_characteristic_;
  BLECharacteristic ack_characteristic_;
  BLECharacteristic stats_characteristic_;
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  BLECharacteristic discovery_characteristic_;
#endif
  using Queue = RecordQueue<kLogQueueBytes, kLogQueueRecords>;
  Queue* log_queue_ = nullptr;
  LineBuffer<544, kLogQueueBytes, Queue>* line_buffer_ = nullptr;
  CommandIngress command_ingress_;
  AckIngress ack_ingress_;
  CallbackEvents callback_events_;
  ConnectionAdmission admission_{kUnauthenticatedTimeoutMs};
  char ack_[kCommandBytes]{};
  size_t ack_length_ = 0;
  volatile uint16_t connection_handle_ = BLE_CONN_HANDLE_INVALID;
  volatile bool secured_ = false;
  bool initialized_ = false;
  bool core_ready_ = false;
  bool initialization_failed_ = false;
  bool entropy_attempted_ = false;
  uint32_t entropy_attempt_at_ = 0;
  bool enabled_ = false;
  bool onboarding_ = false;
  volatile bool complete_record_sent_ = false;
  DeliverySchedule delivery_{kRetryIntervalMs, kFragmentIntervalMs};
  volatile bool passkey_pending_ = false;
  volatile uint8_t passkey_state_ = 0;
  volatile uint16_t passkey_allowed_handle_ = BLE_CONN_HANDLE_INVALID;
  uint8_t pending_passkey_[6]{};
  volatile bool pair_complete_pending_ = false;
  uint16_t pair_complete_handle_ = BLE_CONN_HANDLE_INVALID;
  uint8_t pair_complete_status_ = BLE_GAP_SEC_STATUS_SUCCESS;
  uint32_t failed_notifications_ = 0;
  size_t unavailable_line_bytes_ = 0;
  uint32_t prequeue_unavailable_records_ = 0;
  uint32_t prequeue_unavailable_bytes_ = 0;
  QueueSequenceState sequence_state_;
  uint32_t connected_at_ = 0;
  bool disable_pending_ = false;
  unsigned long enrollment_started_ = 0;
  uint8_t nonce_bytes_[8]{};
  size_t nonce_bytes_read_ = 0;
  uint64_t boot_nonce_ = 0;
  bool nonce_ready_ = false;
  Policy policy_;
  bool (*save_enabled_)(bool) = nullptr;
  bool (*save_peer_)(bool, const PeerIdentity&) = nullptr;
  uint16_t enrollment_handle_ = BLE_CONN_HANDLE_INVALID;
  StatsSnapshot stats_snapshot_;
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  coc::NordicPort coc_port_{&Transport::authorizeCoCLink, &Transport::cocStats,
                            &Transport::cocWake, &Transport::cocSchedule};
  coc::Server coc_server_{coc_port_, SENSECAP_OBSERVER_COC_CHANNELS};
  SoftwareTimer coc_timer_;
  bool coc_timer_initialized_ = false;
#endif

  static void onConnect(uint16_t connection_handle);
  static void onDisconnect(uint16_t connection_handle, uint8_t reason);
  static void onSecured(uint16_t connection_handle);
  static void onCommand(uint16_t connection_handle, BLECharacteristic* characteristic, uint8_t* data, uint16_t length);
  static void onAck(uint16_t connection_handle, BLECharacteristic* characteristic, uint8_t* data, uint16_t length);
  static void onStats(uint16_t connection_handle, BLECharacteristic* characteristic, ble_gatts_evt_read_t* request);
  static bool onPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request);
  static void onPairComplete(uint16_t connection_handle, uint8_t auth_status);
#if defined(SENSECAP_OBSERVER_COC_CHANNELS)
  static void onCoCBLEEvent(ble_evt_t* event);
  static void onDiscovery(uint16_t connection_handle, BLECharacteristic* characteristic,
                          ble_gatts_evt_read_t* request);
  static size_t cocStats(char* bytes, size_t capacity);
  static void cocWake();
  static void cocSchedule(uint32_t delay_ms, bool enabled);
  static void cocTimerCallback(TimerHandle_t timer);
#endif
  bool authorizedAndSecured(uint16_t connection_handle) const;
  bool refreshNonce(bool rotate = false);
  bool persistPeer(bool has_peer, const PeerIdentity& peer);
  bool processAck(const char* bytes, size_t length);
  bool allocateQueue();
  bool failInitialization();
  void serviceAdvertising();
  void releaseQueue();
  void updateStatsSnapshot();
  void setConnectionHandle(uint16_t handle);
  uint16_t connectionHandle() const;
};

extern Transport transport;

class LogMirror : public Stream {
 public:
  int available() override { return Serial.available(); }
  int read() override { return Serial.read(); }
  int peek() override { return Serial.peek(); }
  void flush() override {}
  size_t write(uint8_t byte) override;
  size_t write(const uint8_t* bytes, size_t length) override;
};

extern LogMirror log_mirror;
inline Stream& logOutput() { return log_mirror; }

}  // namespace observer

#endif
