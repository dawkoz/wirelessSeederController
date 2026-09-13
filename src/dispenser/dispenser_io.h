#pragma once

#include <Arduino.h>

#include "dispenser_logic.h"

// ---------------------------------------------------------------------------
// The dispenser's hardware and packet inbox. This is the only dispenser code
// that touches pins; no ESP-NOW init or send lives here. The bench test
// (src/dispenser_bench) reuses this file unchanged and feeds it simulated
// packets.
// ---------------------------------------------------------------------------

void     motorBegin();                          // DIR forward, LEDC attached, duty 0
void     motorApply(const DispenserOutputs &out);
void     encoderBegin();                        // INPUT_PULLUP + RISING interrupt on ENCODER_A_PIN
void     encoderEnd();                          // detach it (bench fault injection only)
uint32_t encoderEdges();
void     dispenserHandlePacket(const uint8_t *data, int len);   // the whole body of the receive callback
void     dispenserReadInputs(uint32_t nowMs, DispenserInputs &in);
void     dispenserResetInbox();                 // forget every received packet (bench only)
uint8_t  dispenserLinkFlags();
bool     dispenserControlTick(DispenserLogic &logic, uint32_t nowMs, DispenserOutputs &out);
