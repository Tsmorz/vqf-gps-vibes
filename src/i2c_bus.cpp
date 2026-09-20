#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_power.h"
#include "config.h"

namespace {

// Per-bus state. `generation` is bumped by Start() itself rather than by its
// callers, so the invariant holds no matter which path restarts a bus: any
// restart invalidates every device handle on it, and the drivers watching the
// counter will re-initialise. An earlier version bumped it in one path only,
// and a second restart path silently broke the IMU for good.
struct BusState {
    TwoWire* wire;
    int sda_pin;
    int scl_pin;
    uint32_t clock_hz;
    const char* name;
    uint32_t generation;
    bool started;
    uint32_t last_transfer_ok_ms;
    uint32_t last_restart_ms;
    int consecutive_restarts;
};

BusState buses[] = {
    {&Wire, I2C_IMU_SDA_PIN, I2C_IMU_SCL_PIN, I2C_IMU_CLOCK_HZ, "imu", 0, false, 0, 0, 0},
    {&Wire1, I2C_AUX_SDA_PIN, I2C_AUX_SCL_PIN, I2C_AUX_CLOCK_HZ, "aux", 0, false, 0, 0, 0},
};

BusState& State(I2cBus bus) {
    return buses[static_cast<int>(bus)];
}

// Consecutive low readings required before SDA is believed to be stuck.
// Sampled over a few hundred microseconds so a transfer that happens to be
// mid-byte is not mistaken for a wedge.
constexpr int kWedgeSamples = 5;

// How long every device on a bus may go without a single successful transfer
// before that peripheral is presumed wedged. Comfortably longer than the
// slowest retry interval, so an absent sensor alone can never trip it.
constexpr uint32_t kBusDeadMs = 3000;

void Start(BusState& state) {
    // The first call is initialisation, not a restart -- nothing holds a
    // handle yet, so it must not count as a new generation.
    if (state.started) {
        state.generation++;
    }
    state.started = true;

    state.wire->begin(state.sda_pin, state.scl_pin);
    state.wire->setClock(state.clock_hz);
    // Without a timeout a single stuck slave blocks the estimator task
    // forever; 50 ms is far longer than any legitimate transfer here.
    state.wire->setTimeOut(50);
}

}  // namespace

void I2cBeginAll() {
    for (BusState& state : buses) {
        Start(state);
    }
}

TwoWire& I2cWire(I2cBus bus) {
    return *State(bus).wire;
}

void I2cRestart(I2cBus bus) {
    BusState& state = State(bus);
    state.wire->end();
    Start(state);
}

bool I2cBusIsWedged(I2cBus bus) {
    const BusState& state = State(bus);
    // digitalRead() reports the pad's input level regardless of which
    // peripheral owns the mux, so nothing here disturbs a working bus.
    for (int sample = 0; sample < kWedgeSamples; sample++) {
        if (digitalRead(state.sda_pin) == HIGH) {
            return false;
        }
        delayMicroseconds(50);
    }
    return true;
}

bool I2cRecover(I2cBus bus) {
    BusState& state = State(bus);
    state.wire->end();

    // Manually clock out up to nine bits -- enough for a slave to finish any
    // byte plus its ACK -- stopping as soon as it releases SDA.
    pinMode(state.scl_pin, OUTPUT_OPEN_DRAIN);
    pinMode(state.sda_pin, INPUT_PULLUP);
    for (int pulse = 0; pulse < 9 && digitalRead(state.sda_pin) == LOW; pulse++) {
        digitalWrite(state.scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite(state.scl_pin, HIGH);
        delayMicroseconds(5);
    }

    // Generate a STOP condition: SDA released while SCL is high.
    pinMode(state.sda_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(state.sda_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(state.scl_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(state.sda_pin, HIGH);
    delayMicroseconds(5);

    const bool sda_released = digitalRead(state.sda_pin) == HIGH;
    Start(state);  // bumps the generation, forcing every driver to re-initialise
    return sda_released;
}

uint32_t I2cBusGeneration(I2cBus bus) {
    return State(bus).generation;
}

bool I2cDeviceResponds(I2cBus bus, uint8_t address) {
    TwoWire& wire = I2cWire(bus);
    wire.beginTransmission(address);
    return wire.endTransmission() == 0;
}

void I2cNoteTransferOk(I2cBus bus) {
    BusState& state = State(bus);
    state.last_transfer_ok_ms = millis();
    state.consecutive_restarts = 0;
}

void I2cServiceWatchdog(I2cBus bus) {
    BusState& state = State(bus);
    const uint32_t now = millis();
    if (state.last_transfer_ok_ms == 0) {
        state.last_transfer_ok_ms = now;  // start the clock at first use
        return;
    }
    if (now - state.last_transfer_ok_ms < kBusDeadMs) {
        return;
    }
    // Leave a gap between attempts so a genuinely dead bus does not restart
    // in a tight loop.
    if (now - state.last_restart_ms < kBusDeadMs) {
        return;
    }
    state.last_restart_ms = now;
    state.last_transfer_ok_ms = now;

    state.consecutive_restarts++;

    if (I2cBusIsWedged(bus)) {
        Serial.printf("[i2c] %s bus: SDA stuck low -- clocking it free\n", state.name);
        I2cRecover(bus);
        return;
    }

    // Escalate if restarting the peripheral has already failed to bring the
    // bus back. The aux sensors sit on the switchable LDO2 rail, so they can
    // be power-cycled -- which resets a device that has stopped listening to
    // its own bus, something no amount of clocking will fix. The IMU is on the
    // always-on rail and has no such option, so it just keeps retrying.
    if (bus == I2cBus::kAux && state.consecutive_restarts >= 2) {
        Serial.printf("[i2c] %s bus: still dead after a restart -- cycling its power\n",
                      state.name);
        state.wire->end();
        BoardPowerCycleAux();
        Start(state);  // bumps the generation, forcing both aux drivers to re-init
        return;
    }

    Serial.printf("[i2c] %s bus: nothing has answered for 3 s -- restarting it\n", state.name);
    I2cRestart(bus);
}
