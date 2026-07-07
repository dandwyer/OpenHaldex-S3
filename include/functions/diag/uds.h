#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/twai.h>

#define OH_UDS_MAX_PAYLOAD 256

struct diag_uds_route_t {
  const char* name;
  uint32_t requestId;
  uint32_t responseId;
  bool extended;
};

struct diag_uds_result_t {
  bool ok;
  bool timeout;
  bool busy;
  bool negative;
  bool pendingSeen;
  uint8_t requestServiceId;
  uint8_t responseServiceId;
  uint8_t negativeServiceId;
  uint8_t nrc;
  uint16_t payloadLen;
  uint8_t payload[OH_UDS_MAX_PAYLOAD];
  uint32_t elapsedMs;
  const char* status;
  const char* message;
  const diag_uds_route_t* route;
};

void diagUdsInit();

// Called from the Haldex receive loop. Returns true when a frame belongs to an
// internal diagnostic transaction and should not be rebroadcast to chassis CAN.
bool diagUdsObserveHaldexFrame(const twai_message_t& frame);

bool diagUdsProbeHaldex(diag_uds_result_t& out, uint32_t timeout_ms = 900);
bool diagUdsReadDataByIdentifier(uint16_t did, diag_uds_result_t& out, uint32_t timeout_ms = 1200);
bool diagUdsReadDtcByStatus(uint8_t status_mask, diag_uds_result_t& out, uint32_t timeout_ms = 2500);
bool diagUdsClearDtc(uint32_t group_of_dtc, diag_uds_result_t& out, uint32_t timeout_ms = 3000);
bool diagKwpTp20ReadLocalIdentifier(uint8_t local_id, diag_uds_result_t& out, uint32_t timeout_ms = 5000);
bool diagKwpTp20ClearDtc(uint32_t group_of_dtc, diag_uds_result_t& out, uint32_t timeout_ms = 5000);

bool diagUdsHasSelectedRoute();
const diag_uds_route_t* diagUdsSelectedRoute();
const char* diagUdsNrcName(uint8_t nrc);
void diagUdsWriteStatusJson(JsonObject out);
void diagUdsWriteResultJson(JsonObject out, const diag_uds_result_t& result);
