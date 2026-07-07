#include "functions/diag/uds.h"

#include <string.h>

#include "functions/can/can.h"
#include "functions/config/config.h"
#include "functions/core/state.h"
#include "functions/storage/filelog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const uint8_t k_sid_read_data_by_identifier = 0x22;
static const uint8_t k_sid_read_dtc_information = 0x19;
static const uint8_t k_sid_clear_diagnostic_information = 0x14;
static const uint8_t k_sid_negative_response = 0x7F;
static const uint8_t k_nrc_response_pending = 0x78;
static const uint8_t k_sid_kwp_start_diagnostic_session = 0x10;
static const uint8_t k_sid_kwp_read_data_by_local_identifier = 0x21;
static const uint8_t k_kwp_vag_diagnostic_session = 0x89;

static const uint8_t k_tp20_logical_haldex = 0x0A;
static const uint32_t k_tp20_setup_request_id = 0x200;
static const uint32_t k_tp20_setup_response_id = 0x200 + k_tp20_logical_haldex;
static const uint32_t k_tp20_requested_ecu_tx_id = 0x300;
static const uint8_t k_tp20_app_kwp = 0x01;
static const uint8_t k_tp20_setup_request = 0xC0;
static const uint8_t k_tp20_setup_positive = 0xD0;
static const uint8_t k_tp20_params_request = 0xA0;
static const uint8_t k_tp20_params_response = 0xA1;
static const uint8_t k_tp20_disconnect = 0xA8;

static const diag_uds_route_t k_haldex_routes[] = {
  {"allwheel-70f", 0x70F, 0x779, false},
  {"allwheel-71d", 0x71D, 0x787, false},
  {"allwheel-29bit", 0x1C40000F, 0x1C42000F, true},
};

static const diag_uds_route_t k_kwp_tp20_haldex_route = {"allwheel-kwp-tp20", k_tp20_setup_request_id,
                                                         k_tp20_setup_response_id, false};

static QueueHandle_t g_rx_queue = nullptr;
static QueueHandle_t g_tp20_rx_queue = nullptr;
static SemaphoreHandle_t g_diag_mutex = nullptr;
static volatile bool g_active = false;
static volatile uint32_t g_active_response_id = 0;
static volatile bool g_active_extended = false;
static volatile bool g_tp20_active = false;
static volatile uint32_t g_tp20_response_id = 0;
static int g_selected_route_index = -1;
static diag_uds_result_t g_last_result = {};
static uint32_t g_last_result_ms = 0;
static const char* g_last_transport = "uds_can";

static String byteHex(uint8_t value) {
  String out;
  if (value < 0x10) {
    out += "0";
  }
  out += String(value, HEX);
  out.toUpperCase();
  return out;
}

static String wordHex(uint32_t value, bool extended) {
  String out = "0x";
  String hex = String(value, HEX);
  hex.toUpperCase();
  const uint8_t width = extended ? 8 : 3;
  while (hex.length() < width) {
    hex = "0" + hex;
  }
  out += hex;
  return out;
}

static String bytesHex(const uint8_t* data, uint16_t len) {
  String out;
  out.reserve((size_t)len * 3);
  for (uint16_t i = 0; i < len; i++) {
    if (i > 0) {
      out += " ";
    }
    out += byteHex(data[i]);
  }
  return out;
}

static void clearResult(diag_uds_result_t& result) {
  memset(&result, 0, sizeof(result));
  result.status = "idle";
  result.message = "";
}

static void failResult(diag_uds_result_t& result, const char* status, const char* message) {
  result.ok = false;
  result.status = status;
  result.message = message;
}

static void storeLastResult(const diag_uds_result_t& result) {
  g_last_result = result;
  g_last_result_ms = millis();
}

static void recordLastResult(const diag_uds_result_t& result) {
  g_last_transport = "uds_can";
  storeLastResult(result);
}

static void recordLastResult(const diag_uds_result_t& result, const char* transport) {
  g_last_transport = transport ? transport : "uds_can";
  storeLastResult(result);
}

static bool ensureInit() {
  if (!g_rx_queue) {
    g_rx_queue = xQueueCreate(16, sizeof(twai_message_t));
  }
  if (!g_tp20_rx_queue) {
    g_tp20_rx_queue = xQueueCreate(16, sizeof(twai_message_t));
  }
  if (!g_diag_mutex) {
    g_diag_mutex = xSemaphoreCreateMutex();
  }
  return g_rx_queue && g_tp20_rx_queue && g_diag_mutex;
}

void diagUdsInit() {
  (void)ensureInit();
}

bool diagUdsObserveHaldexFrame(const twai_message_t& frame) {
  const uint32_t id = frame.identifier & 0x1FFFFFFF;

  if (g_active) {
    if (id == g_active_response_id && ((bool)frame.extd) == g_active_extended) {
      if (g_rx_queue) {
        (void)xQueueSend(g_rx_queue, &frame, 0);
      }
      return true;
    }
  }

  if (g_tp20_active && !frame.extd) {
    if (id == k_tp20_setup_response_id || (g_tp20_response_id != 0 && id == g_tp20_response_id) ||
        id == k_tp20_requested_ecu_tx_id) {
      if (g_tp20_rx_queue) {
        (void)xQueueSend(g_tp20_rx_queue, &frame, 0);
      }
      return true;
    }
  }

  return false;
}

static void clearRxQueue() {
  if (!g_rx_queue) {
    return;
  }
  twai_message_t discarded = {};
  while (xQueueReceive(g_rx_queue, &discarded, 0) == pdTRUE) {
  }
}

static void clearTp20RxQueue() {
  if (!g_tp20_rx_queue) {
    return;
  }
  twai_message_t discarded = {};
  while (xQueueReceive(g_tp20_rx_queue, &discarded, 0) == pdTRUE) {
  }
}

static bool sendSingleFrame(const diag_uds_route_t& route, const uint8_t* payload, uint8_t len) {
  if (len == 0 || len > 7) {
    return false;
  }

  twai_message_t msg = {};
  msg.identifier = route.requestId;
  msg.extd = route.extended ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  msg.data[0] = len & 0x0F;
  for (uint8_t i = 0; i < len; i++) {
    msg.data[i + 1] = payload[i];
  }
  return haldex_can_send(msg, pdMS_TO_TICKS(20), true);
}

static bool sendFlowControl(const diag_uds_route_t& route) {
  twai_message_t msg = {};
  msg.identifier = route.requestId;
  msg.extd = route.extended ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  msg.data[0] = 0x30; // Continue To Send
  msg.data[1] = 0x00; // Block size: unlimited
  msg.data[2] = 0x00; // STmin: no extra separation requested
  return haldex_can_send(msg, pdMS_TO_TICKS(20), true);
}

static bool responseLooksRelevant(const uint8_t* payload, uint16_t len, uint8_t request_sid) {
  if (len == 0) {
    return false;
  }
  if (payload[0] == (uint8_t)(request_sid + 0x40)) {
    return true;
  }
  return len >= 3 && payload[0] == k_sid_negative_response && payload[1] == request_sid;
}

static bool finishResponse(uint8_t request_sid, diag_uds_result_t& result) {
  if (!responseLooksRelevant(result.payload, result.payloadLen, request_sid)) {
    failResult(result, "unexpected_response", "Unexpected UDS response");
    return false;
  }

  result.responseServiceId = result.payload[0];
  if (result.payload[0] == k_sid_negative_response) {
    result.negative = true;
    result.negativeServiceId = result.payloadLen >= 2 ? result.payload[1] : 0;
    result.nrc = result.payloadLen >= 3 ? result.payload[2] : 0;
    result.status = "negative_response";
    result.message = diagUdsNrcName(result.nrc);
    result.ok = false;
    return false;
  }

  result.ok = true;
  result.status = "positive";
  result.message = "Positive response";
  return true;
}

static bool copyPayload(diag_uds_result_t& result, const uint8_t* payload, uint16_t len) {
  if (len > OH_UDS_MAX_PAYLOAD) {
    failResult(result, "payload_too_large", "UDS response exceeded local buffer");
    return false;
  }
  memcpy(result.payload, payload, len);
  result.payloadLen = len;
  return true;
}

static void encodeTp20CanId(uint32_t id, bool valid, uint8_t& low, uint8_t& validity_prefix) {
  low = (uint8_t)(id & 0xFF);
  validity_prefix = (uint8_t)((id >> 8) & 0x0F);
  if (!valid) {
    validity_prefix |= 0x10;
  }
}

static uint32_t decodeTp20CanId(uint8_t low, uint8_t validity_prefix, bool& valid) {
  valid = (validity_prefix & 0xF0) == 0;
  return (((uint32_t)validity_prefix & 0x0F) << 8) | low;
}

static bool sendTp20Frame(uint32_t can_id, const uint8_t* data, uint8_t len) {
  if (len == 0 || len > 8) {
    return false;
  }

  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  for (uint8_t i = 0; i < len; i++) {
    msg.data[i] = data[i];
  }
  return haldex_can_send(msg, pdMS_TO_TICKS(20), true);
}

static bool receiveTp20Frame(uint32_t expected_id, twai_message_t& out, uint32_t started_ms, uint32_t timeout_ms) {
  while ((millis() - started_ms) < timeout_ms) {
    const uint32_t elapsed = millis() - started_ms;
    const uint32_t remaining = elapsed < timeout_ms ? timeout_ms - elapsed : 0;

    twai_message_t frame = {};
    if (xQueueReceive(g_tp20_rx_queue, &frame, pdMS_TO_TICKS(remaining > 0 ? remaining : 1)) != pdTRUE) {
      continue;
    }

    const uint32_t id = frame.identifier & 0x1FFFFFFF;
    if (!frame.extd && id == expected_id) {
      out = frame;
      return true;
    }
  }
  return false;
}

static bool finishKwpResponse(uint8_t request_sid, uint8_t positive_sid, diag_uds_result_t& result) {
  if (result.payloadLen == 0) {
    failResult(result, "empty_response", "Empty KWP response");
    return false;
  }

  result.responseServiceId = result.payload[0];
  if (result.payloadLen >= 3 && result.payload[0] == k_sid_negative_response && result.payload[1] == request_sid) {
    result.negative = true;
    result.negativeServiceId = result.payload[1];
    result.nrc = result.payload[2];
    result.status = "negative_response";
    result.message = diagUdsNrcName(result.nrc);
    result.ok = false;
    return false;
  }

  if (result.payload[0] != positive_sid) {
    failResult(result, "unexpected_response", "Unexpected KWP response");
    return false;
  }

  result.ok = true;
  result.status = "positive";
  result.message = "Positive response";
  return true;
}

static bool sendTp20Ack(uint32_t tester_tx_id, uint8_t rx_seq) {
  const uint8_t ack[] = {(uint8_t)(0xB0 | ((rx_seq + 1) & 0x0F))};
  return sendTp20Frame(tester_tx_id, ack, sizeof(ack));
}

static bool receiveKwpPayloadTp20(uint32_t tester_tx_id, uint32_t ecu_tx_id, uint8_t request_sid,
                                  uint8_t positive_sid, diag_uds_result_t& result, uint32_t started_ms,
                                  uint32_t timeout_ms) {
  uint16_t total_len = 0;
  uint16_t offset = 0;
  bool saw_first_frame = false;

  while ((millis() - started_ms) < timeout_ms) {
    twai_message_t frame = {};
    if (!receiveTp20Frame(ecu_tx_id, frame, started_ms, timeout_ms)) {
      result.timeout = true;
      failResult(result, "timeout", "No KWP/TP20 response from Haldex module");
      return false;
    }

    if (frame.data_length_code == 0) {
      continue;
    }

    const uint8_t op = frame.data[0] >> 4;
    const uint8_t seq = frame.data[0] & 0x0F;

    if (op == 0xB || op == 0x9) {
      continue;
    }
    if (op > 0x3) {
      continue;
    }

    if (!saw_first_frame) {
      if (frame.data_length_code < 3) {
        failResult(result, "bad_tp20_frame", "Invalid first KWP/TP20 data frame");
        return false;
      }
      total_len = ((uint16_t)frame.data[1] << 8) | frame.data[2];
      if (total_len == 0 || total_len > OH_UDS_MAX_PAYLOAD) {
        failResult(result, "payload_too_large", "KWP/TP20 response exceeded local buffer");
        return false;
      }
      for (uint8_t i = 3; i < frame.data_length_code && offset < total_len; i++) {
        result.payload[offset++] = frame.data[i];
      }
      saw_first_frame = true;
    } else {
      for (uint8_t i = 1; i < frame.data_length_code && offset < total_len; i++) {
        result.payload[offset++] = frame.data[i];
      }
    }

    const bool sender_waits_for_ack = (op == 0x0 || op == 0x1);
    if (sender_waits_for_ack && !sendTp20Ack(tester_tx_id, seq)) {
      failResult(result, "ack_failed", "Failed to acknowledge KWP/TP20 response");
      return false;
    }

    if (saw_first_frame && offset >= total_len) {
      result.payloadLen = total_len;
      if (result.payloadLen >= 3 && result.payload[0] == k_sid_negative_response &&
          result.payload[1] == request_sid && result.payload[2] == k_nrc_response_pending) {
        result.pendingSeen = true;
        offset = 0;
        total_len = 0;
        saw_first_frame = false;
        continue;
      }
      return finishKwpResponse(request_sid, positive_sid, result);
    }
  }

  result.timeout = true;
  failResult(result, "timeout", "Timed out waiting for complete KWP/TP20 response");
  return false;
}

static bool sendKwpRequestTp20(uint32_t tester_tx_id, uint32_t ecu_tx_id, const uint8_t* request, uint8_t request_len,
                               uint8_t positive_sid, uint8_t& tx_seq, diag_uds_result_t& result,
                               uint32_t started_ms, uint32_t timeout_ms) {
  if (request_len == 0 || request_len > 5) {
    failResult(result, "request_too_long", "KWP/TP20 helper only supports single-frame requests");
    return false;
  }

  clearResult(result);
  result.route = &k_kwp_tp20_haldex_route;
  result.requestServiceId = request[0];

  uint8_t frame[8] = {};
  frame[0] = (uint8_t)(0x10 | (tx_seq & 0x0F));
  frame[1] = 0x00;
  frame[2] = request_len;
  for (uint8_t i = 0; i < request_len; i++) {
    frame[i + 3] = request[i];
  }

  if (!sendTp20Frame(tester_tx_id, frame, (uint8_t)(request_len + 3))) {
    failResult(result, "send_failed", "Failed to send KWP/TP20 request");
    return false;
  }

  const uint8_t expected_ack_seq = (tx_seq + 1) & 0x0F;
  tx_seq = expected_ack_seq;
  bool ack_seen = false;
  bool response_started = false;
  while ((millis() - started_ms) < timeout_ms) {
    twai_message_t ack = {};
    if (!receiveTp20Frame(ecu_tx_id, ack, started_ms, timeout_ms)) {
      break;
    }
    if (ack.data_length_code == 0) {
      continue;
    }
    const uint8_t op = ack.data[0] >> 4;
    const uint8_t seq = ack.data[0] & 0x0F;
    if (op == 0xB && seq == expected_ack_seq) {
      ack_seen = true;
      break;
    }
    if (op <= 0x3) {
      (void)xQueueSendToFront(g_tp20_rx_queue, &ack, 0);
      response_started = true;
      break;
    }
  }

  if (!ack_seen && !response_started) {
    failResult(result, "ack_timeout", "No KWP/TP20 ACK from Haldex module");
    result.timeout = true;
    return false;
  }

  return receiveKwpPayloadTp20(tester_tx_id, ecu_tx_id, request[0], positive_sid, result, started_ms, timeout_ms);
}

static bool openTp20Channel(uint32_t& tester_tx_id, uint32_t& ecu_tx_id, diag_uds_result_t& result,
                            uint32_t started_ms, uint32_t timeout_ms) {
  uint8_t rx_low = 0;
  uint8_t rx_vp = 0;
  uint8_t tx_low = 0;
  uint8_t tx_vp = 0;
  encodeTp20CanId(0, false, rx_low, rx_vp);
  encodeTp20CanId(k_tp20_requested_ecu_tx_id, true, tx_low, tx_vp);

  const uint8_t setup[] = {
    k_tp20_logical_haldex,
    k_tp20_setup_request,
    rx_low,
    rx_vp,
    tx_low,
    tx_vp,
    k_tp20_app_kwp,
  };

  if (!sendTp20Frame(k_tp20_setup_request_id, setup, sizeof(setup))) {
    failResult(result, "setup_send_failed", "Failed to send VW TP2.0 channel setup");
    return false;
  }

  twai_message_t response = {};
  if (!receiveTp20Frame(k_tp20_setup_response_id, response, started_ms, timeout_ms)) {
    result.timeout = true;
    failResult(result, "setup_timeout", "No VW TP2.0 setup response from Haldex module");
    return false;
  }

  if (response.data_length_code < 7) {
    failResult(result, "bad_setup_response", "Invalid VW TP2.0 setup response");
    return false;
  }
  if (response.data[1] >= 0xD6 && response.data[1] <= 0xD8) {
    failResult(result, "setup_negative", "VW TP2.0 channel setup rejected");
    return false;
  }
  if (response.data[1] != k_tp20_setup_positive || response.data[0] != 0x00 || response.data[6] != k_tp20_app_kwp) {
    failResult(result, "bad_setup_response", "Unexpected VW TP2.0 setup response");
    return false;
  }

  bool ecu_tx_valid = false;
  bool tester_tx_valid = false;
  ecu_tx_id = decodeTp20CanId(response.data[2], response.data[3], ecu_tx_valid);
  tester_tx_id = decodeTp20CanId(response.data[4], response.data[5], tester_tx_valid);
  if (!ecu_tx_valid || !tester_tx_valid || ecu_tx_id == 0 || tester_tx_id == 0) {
    failResult(result, "bad_setup_response", "VW TP2.0 setup returned invalid CAN IDs");
    return false;
  }
  g_tp20_response_id = ecu_tx_id;

  const uint8_t params[] = {
    k_tp20_params_request,
    0x0F,
    0x8A,
    0xFF,
    0x32,
    0xFF,
  };
  if (!sendTp20Frame(tester_tx_id, params, sizeof(params))) {
    failResult(result, "params_send_failed", "Failed to send VW TP2.0 channel parameters");
    return false;
  }

  twai_message_t params_response = {};
  if (!receiveTp20Frame(ecu_tx_id, params_response, started_ms, timeout_ms)) {
    result.timeout = true;
    failResult(result, "params_timeout", "No VW TP2.0 parameter response from Haldex module");
    return false;
  }
  if (params_response.data_length_code < 1 || params_response.data[0] != k_tp20_params_response) {
    failResult(result, "bad_params_response", "Unexpected VW TP2.0 parameter response");
    return false;
  }

  return true;
}

static void disconnectTp20Channel(uint32_t tester_tx_id, uint32_t ecu_tx_id) {
  if (tester_tx_id == 0) {
    return;
  }

  const uint8_t disconnect[] = {k_tp20_disconnect};
  (void)sendTp20Frame(tester_tx_id, disconnect, sizeof(disconnect));

  const uint32_t started_ms = millis();
  twai_message_t response = {};
  (void)receiveTp20Frame(ecu_tx_id, response, started_ms, 50);
}

static bool readSingleFrame(const twai_message_t& frame, uint8_t request_sid, diag_uds_result_t& result,
                            bool& response_pending) {
  const uint8_t len = frame.data[0] & 0x0F;
  if (len == 0 || len > 7 || frame.data_length_code < (uint8_t)(len + 1)) {
    failResult(result, "bad_single_frame", "Invalid ISO-TP single frame");
    return false;
  }

  if (!copyPayload(result, &frame.data[1], len)) {
    return false;
  }

  if (result.payloadLen >= 3 && result.payload[0] == k_sid_negative_response &&
      result.payload[1] == request_sid && result.payload[2] == k_nrc_response_pending) {
    result.pendingSeen = true;
    response_pending = true;
    return false;
  }

  response_pending = false;
  return finishResponse(request_sid, result);
}

static bool readFirstFrame(const diag_uds_route_t& route, const twai_message_t& frame, uint8_t request_sid,
                           diag_uds_result_t& result, uint32_t started_ms, uint32_t timeout_ms) {
  const uint16_t total_len = (uint16_t)(((frame.data[0] & 0x0F) << 8) | frame.data[1]);
  if (total_len == 0 || total_len > OH_UDS_MAX_PAYLOAD) {
    failResult(result, "payload_too_large", "UDS multi-frame response exceeded local buffer");
    return false;
  }

  uint16_t offset = 0;
  for (uint8_t i = 2; i < frame.data_length_code && offset < total_len; i++) {
    result.payload[offset++] = frame.data[i];
  }
  result.payloadLen = offset;

  if (!sendFlowControl(route)) {
    failResult(result, "flow_control_failed", "Failed to send ISO-TP flow control");
    return false;
  }

  uint8_t expected_seq = 1;
  while (result.payloadLen < total_len && (millis() - started_ms) < timeout_ms) {
    const uint32_t elapsed = millis() - started_ms;
    const uint32_t remaining = elapsed < timeout_ms ? timeout_ms - elapsed : 0;
    twai_message_t cf = {};
    if (xQueueReceive(g_rx_queue, &cf, pdMS_TO_TICKS(remaining > 0 ? remaining : 1)) != pdTRUE) {
      continue;
    }

    if ((cf.data[0] & 0xF0) != 0x20) {
      continue;
    }

    const uint8_t seq = cf.data[0] & 0x0F;
    if (seq != expected_seq) {
      failResult(result, "bad_sequence", "ISO-TP consecutive frame sequence mismatch");
      return false;
    }
    expected_seq = (expected_seq + 1) & 0x0F;

    for (uint8_t i = 1; i < cf.data_length_code && result.payloadLen < total_len; i++) {
      result.payload[result.payloadLen++] = cf.data[i];
    }
  }

  if (result.payloadLen < total_len) {
    result.timeout = true;
    failResult(result, "timeout", "Timed out waiting for ISO-TP consecutive frames");
    return false;
  }

  return finishResponse(request_sid, result);
}

static bool receiveResponse(const diag_uds_route_t& route, uint8_t request_sid, diag_uds_result_t& result,
                            uint32_t timeout_ms) {
  const uint32_t started_ms = millis();
  while ((millis() - started_ms) < timeout_ms) {
    const uint32_t elapsed = millis() - started_ms;
    const uint32_t remaining = elapsed < timeout_ms ? timeout_ms - elapsed : 0;

    twai_message_t frame = {};
    if (xQueueReceive(g_rx_queue, &frame, pdMS_TO_TICKS(remaining > 0 ? remaining : 1)) != pdTRUE) {
      continue;
    }

    const uint8_t pci = frame.data[0] & 0xF0;
    if (pci == 0x00) {
      bool response_pending = false;
      const bool ok = readSingleFrame(frame, request_sid, result, response_pending);
      if (response_pending) {
        continue;
      }
      result.elapsedMs = millis() - started_ms;
      return ok;
    }
    if (pci == 0x10) {
      const bool ok = readFirstFrame(route, frame, request_sid, result, started_ms, timeout_ms);
      result.elapsedMs = millis() - started_ms;
      return ok;
    }
  }

  result.timeout = true;
  result.elapsedMs = millis() - started_ms;
  failResult(result, "timeout", "No UDS response from Haldex module");
  return false;
}

static bool sendRequestOnRoute(const diag_uds_route_t& route, const uint8_t* request, uint8_t request_len,
                               diag_uds_result_t& result, uint32_t timeout_ms) {
  clearResult(result);
  result.route = &route;
  result.requestServiceId = request_len > 0 ? request[0] : 0;

  if (!ensureInit()) {
    failResult(result, "not_initialized", "UDS diagnostics failed to initialize");
    recordLastResult(result);
    return false;
  }
  if (!can_ready || !can1_ready) {
    failResult(result, "can_not_ready", "Haldex CAN is not ready");
    recordLastResult(result);
    return false;
  }
  if (request_len == 0 || request_len > 7) {
    failResult(result, "request_too_long", "Only single-frame UDS requests are enabled in this build");
    recordLastResult(result);
    return false;
  }

  if (xSemaphoreTake(g_diag_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    result.busy = true;
    failResult(result, "busy", "UDS diagnostic request already active");
    recordLastResult(result);
    return false;
  }

  clearRxQueue();
  g_active_response_id = route.responseId;
  g_active_extended = route.extended;
  g_active = true;

  bool ok = false;
  if (!sendSingleFrame(route, request, request_len)) {
    failResult(result, "send_failed", "Failed to send UDS request");
  } else {
    ok = receiveResponse(route, request[0], result, timeout_ms);
  }

  g_active = false;
  xSemaphoreGive(g_diag_mutex);

  recordLastResult(result);
  if (!ok && result.negative) {
    LOG_WARN("uds", "negative response route=%s sid=0x%02X nrc=0x%02X %s", route.name, result.requestServiceId,
             result.nrc, diagUdsNrcName(result.nrc));
  }
  return ok;
}

static bool isPositiveDidResponse(const diag_uds_result_t& result, uint16_t did) {
  return result.ok && result.payloadLen >= 3 && result.payload[0] == (k_sid_read_data_by_identifier + 0x40) &&
         result.payload[1] == (uint8_t)(did >> 8) && result.payload[2] == (uint8_t)(did & 0xFF);
}

static bool readDidOnRoute(const diag_uds_route_t& route, uint16_t did, diag_uds_result_t& out, uint32_t timeout_ms) {
  const uint8_t req[] = {
    k_sid_read_data_by_identifier,
    (uint8_t)(did >> 8),
    (uint8_t)(did & 0xFF),
  };
  return sendRequestOnRoute(route, req, sizeof(req), out, timeout_ms);
}

bool diagUdsProbeHaldex(diag_uds_result_t& out, uint32_t timeout_ms) {
  clearResult(out);
  if (!ensureInit()) {
    failResult(out, "not_initialized", "UDS diagnostics failed to initialize");
    recordLastResult(out);
    return false;
  }

  const int selected_first = g_selected_route_index;
  if (selected_first >= 0 && selected_first < (int)(sizeof(k_haldex_routes) / sizeof(k_haldex_routes[0]))) {
    if (readDidOnRoute(k_haldex_routes[selected_first], 0xF19E, out, timeout_ms) &&
        isPositiveDidResponse(out, 0xF19E)) {
      return true;
    }
  }

  for (uint8_t i = 0; i < sizeof(k_haldex_routes) / sizeof(k_haldex_routes[0]); i++) {
    if ((int)i == selected_first) {
      continue;
    }
    if (readDidOnRoute(k_haldex_routes[i], 0xF19E, out, timeout_ms) && isPositiveDidResponse(out, 0xF19E)) {
      g_selected_route_index = i;
      LOG_INFO("uds", "selected Haldex UDS route=%s req=%s resp=%s", k_haldex_routes[i].name,
               wordHex(k_haldex_routes[i].requestId, k_haldex_routes[i].extended).c_str(),
               wordHex(k_haldex_routes[i].responseId, k_haldex_routes[i].extended).c_str());
      return true;
    }
  }

  g_selected_route_index = -1;
  if (out.status == nullptr || strcmp(out.status, "idle") == 0) {
    failResult(out, "no_route", "No Haldex UDS route responded");
    recordLastResult(out);
  }
  return false;
}

bool diagUdsReadDataByIdentifier(uint16_t did, diag_uds_result_t& out, uint32_t timeout_ms) {
  if (!diagUdsHasSelectedRoute()) {
    diag_uds_result_t probe = {};
    if (!diagUdsProbeHaldex(probe, timeout_ms)) {
      out = probe;
      return false;
    }
  }

  const diag_uds_route_t* route = diagUdsSelectedRoute();
  if (!route) {
    clearResult(out);
    failResult(out, "no_route", "No Haldex UDS route selected");
    recordLastResult(out);
    return false;
  }

  bool ok = readDidOnRoute(*route, did, out, timeout_ms);
  if (!ok && out.timeout) {
    g_selected_route_index = -1;
  }
  return ok;
}

bool diagUdsReadDtcByStatus(uint8_t status_mask, diag_uds_result_t& out, uint32_t timeout_ms) {
  if (!diagUdsHasSelectedRoute()) {
    diag_uds_result_t probe = {};
    if (!diagUdsProbeHaldex(probe, timeout_ms)) {
      out = probe;
      return false;
    }
  }

  const diag_uds_route_t* route = diagUdsSelectedRoute();
  if (!route) {
    clearResult(out);
    failResult(out, "no_route", "No Haldex UDS route selected");
    recordLastResult(out);
    return false;
  }

  const uint8_t req[] = {
    k_sid_read_dtc_information,
    0x02,
    status_mask,
  };
  const bool ok = sendRequestOnRoute(*route, req, sizeof(req), out, timeout_ms);
  if (!ok && out.timeout) {
    g_selected_route_index = -1;
  }
  return ok;
}

bool diagUdsClearDtc(uint32_t group_of_dtc, diag_uds_result_t& out, uint32_t timeout_ms) {
  if (!diagUdsHasSelectedRoute()) {
    diag_uds_result_t probe = {};
    if (!diagUdsProbeHaldex(probe, 900)) {
      out = probe;
      return false;
    }
  }

  const diag_uds_route_t* route = diagUdsSelectedRoute();
  if (!route) {
    clearResult(out);
    failResult(out, "no_route", "No Haldex UDS route selected");
    recordLastResult(out);
    return false;
  }

  group_of_dtc &= 0xFFFFFFUL;
  const uint8_t req[] = {
    k_sid_clear_diagnostic_information,
    (uint8_t)((group_of_dtc >> 16) & 0xFF),
    (uint8_t)((group_of_dtc >> 8) & 0xFF),
    (uint8_t)(group_of_dtc & 0xFF),
  };
  const bool ok = sendRequestOnRoute(*route, req, sizeof(req), out, timeout_ms);
  if (!ok && out.timeout) {
    g_selected_route_index = -1;
  }
  return ok;
}

static bool runKwpTp20Request(const uint8_t* request, uint8_t request_len, uint8_t positive_sid,
                              diag_uds_result_t& out, uint32_t timeout_ms) {
  clearResult(out);
  out.route = &k_kwp_tp20_haldex_route;

  if (!ensureInit()) {
    failResult(out, "not_initialized", "KWP/TP20 diagnostics failed to initialize");
    recordLastResult(out, "kwp_tp20");
    return false;
  }
  if (!can_ready || !can1_ready) {
    failResult(out, "can_not_ready", "Haldex CAN is not ready");
    recordLastResult(out, "kwp_tp20");
    return false;
  }

  if (xSemaphoreTake(g_diag_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    out.busy = true;
    failResult(out, "busy", "Diagnostic request already active");
    recordLastResult(out, "kwp_tp20");
    return false;
  }

  const uint32_t started_ms = millis();
  clearRxQueue();
  clearTp20RxQueue();
  g_tp20_response_id = 0;
  g_tp20_active = true;

  uint32_t tester_tx_id = 0;
  uint32_t ecu_tx_id = 0;
  bool channel_open = false;
  bool ok = false;

  if (openTp20Channel(tester_tx_id, ecu_tx_id, out, started_ms, timeout_ms)) {
    channel_open = true;
    uint8_t tx_seq = 0;
    const uint8_t session_req[] = {k_sid_kwp_start_diagnostic_session, k_kwp_vag_diagnostic_session};
    if (sendKwpRequestTp20(tester_tx_id, ecu_tx_id, session_req, sizeof(session_req),
                           (uint8_t)(k_sid_kwp_start_diagnostic_session + 0x40), tx_seq, out, started_ms,
                           timeout_ms)) {
      ok = sendKwpRequestTp20(tester_tx_id, ecu_tx_id, request, request_len, positive_sid, tx_seq, out, started_ms,
                              timeout_ms);
    }
  }

  if (channel_open) {
    disconnectTp20Channel(tester_tx_id, ecu_tx_id);
  }

  g_tp20_active = false;
  g_tp20_response_id = 0;
  out.elapsedMs = millis() - started_ms;
  xSemaphoreGive(g_diag_mutex);

  recordLastResult(out, "kwp_tp20");
  if (!ok && out.negative) {
    LOG_WARN("kwp", "negative response route=%s sid=0x%02X nrc=0x%02X %s", k_kwp_tp20_haldex_route.name,
             out.requestServiceId, out.nrc, diagUdsNrcName(out.nrc));
  }
  return ok;
}

bool diagKwpTp20ReadLocalIdentifier(uint8_t local_id, diag_uds_result_t& out, uint32_t timeout_ms) {
  const uint8_t request[] = {
    k_sid_kwp_read_data_by_local_identifier,
    local_id,
  };
  return runKwpTp20Request(request, sizeof(request), (uint8_t)(k_sid_kwp_read_data_by_local_identifier + 0x40),
                           out, timeout_ms);
}

bool diagKwpTp20ClearDtc(uint32_t group_of_dtc, diag_uds_result_t& out, uint32_t timeout_ms) {
  group_of_dtc &= 0xFFFFFFUL;
  const uint8_t request[] = {
    k_sid_clear_diagnostic_information,
    (uint8_t)((group_of_dtc >> 16) & 0xFF),
    (uint8_t)((group_of_dtc >> 8) & 0xFF),
    (uint8_t)(group_of_dtc & 0xFF),
  };
  return runKwpTp20Request(request, sizeof(request), (uint8_t)(k_sid_clear_diagnostic_information + 0x40), out,
                           timeout_ms);
}

bool diagUdsHasSelectedRoute() {
  return g_selected_route_index >= 0 &&
         g_selected_route_index < (int)(sizeof(k_haldex_routes) / sizeof(k_haldex_routes[0]));
}

const diag_uds_route_t* diagUdsSelectedRoute() {
  if (!diagUdsHasSelectedRoute()) {
    return nullptr;
  }
  return &k_haldex_routes[g_selected_route_index];
}

const char* diagUdsNrcName(uint8_t nrc) {
  switch (nrc) {
  case 0x10:
    return "generalReject";
  case 0x11:
    return "serviceNotSupported";
  case 0x12:
    return "subFunctionNotSupported";
  case 0x13:
    return "incorrectMessageLengthOrInvalidFormat";
  case 0x21:
    return "busyRepeatRequest";
  case 0x22:
    return "conditionsNotCorrect";
  case 0x24:
    return "requestSequenceError";
  case 0x31:
    return "requestOutOfRange";
  case 0x33:
    return "securityAccessDenied";
  case 0x35:
    return "invalidKey";
  case 0x36:
    return "exceedNumberOfAttempts";
  case 0x37:
    return "requiredTimeDelayNotExpired";
  case 0x78:
    return "requestCorrectlyReceivedResponsePending";
  default:
    break;
  }
  return "unknownNrc";
}

static void writeRouteJson(JsonObject out, const diag_uds_route_t* route) {
  if (!route) {
    out["selected"] = false;
    return;
  }
  out["selected"] = true;
  out["name"] = route->name;
  out["requestCanId"] = wordHex(route->requestId, route->extended);
  out["responseCanId"] = wordHex(route->responseId, route->extended);
  out["extended"] = route->extended;
}

void diagUdsWriteStatusJson(JsonObject out) {
  out["supported"] = true;
  out["transport"] = g_tp20_active ? "kwp_tp20" : g_last_transport;
  out["kwpTp20Supported"] = true;
  out["target"] = "haldex";
  out["active"] = (bool)(g_active || g_tp20_active);
  out["udsActive"] = (bool)g_active;
  out["kwpTp20Active"] = (bool)g_tp20_active;
  JsonObject route = out["route"].to<JsonObject>();
  const bool last_was_tp20 = strcmp(g_last_transport, "kwp_tp20") == 0;
  writeRouteJson(route, (g_tp20_active || last_was_tp20) ? &k_kwp_tp20_haldex_route : diagUdsSelectedRoute());

  JsonObject last = out["last"].to<JsonObject>();
  if (g_last_result_ms == 0) {
    last["available"] = false;
  } else {
    last["available"] = true;
    last["ageMs"] = millis() - g_last_result_ms;
    diagUdsWriteResultJson(last, g_last_result);
  }
}

void diagUdsWriteResultJson(JsonObject out, const diag_uds_result_t& result) {
  out["ok"] = result.ok;
  out["status"] = result.status ? result.status : "";
  out["message"] = result.message ? result.message : "";
  out["timeout"] = result.timeout;
  out["busy"] = result.busy;
  out["negative"] = result.negative;
  out["pendingSeen"] = result.pendingSeen;
  out["elapsedMs"] = result.elapsedMs;
  out["requestServiceId"] = byteHex(result.requestServiceId);
  out["responseServiceId"] = byteHex(result.responseServiceId);
  out["responseHex"] = bytesHex(result.payload, result.payloadLen);
  out["responseLength"] = result.payloadLen;
  if (result.negative) {
    out["negativeServiceId"] = byteHex(result.negativeServiceId);
    out["nrc"] = byteHex(result.nrc);
    out["nrcName"] = diagUdsNrcName(result.nrc);
  }
  JsonObject route = out["route"].to<JsonObject>();
  writeRouteJson(route, result.route);
}
