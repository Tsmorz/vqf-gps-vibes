#pragma once

// The FeatherS3's BOOT button (GPIO 0), used as the board's only physical input.
//
//   short press  start a magnetometer sweep; press again to finish it
//   long press   deep sleep; press again to wake (the board reboots)
//
// GPIO 0 is a strapping pin: held low at reset it selects the ROM bootloader.
// That is harmless here because it is only sampled at power-on/reset, and it
// is read as a plain input with the board's external pull-up afterwards.

enum class ButtonEvent {
    kNone,
    kShortPress,
    kLongPress,
};

void ButtonBegin();

// Debounces and classifies the button. Call often from loop(); returns each
// event once. A long press fires as soon as the threshold passes, not on
// release, so the user gets feedback while still holding.
ButtonEvent ButtonPoll();

// Blocks until the button is released, then enters deep sleep with the button
// armed as the wake source. Does not return.
[[noreturn]] void ButtonSleepUntilPressed();
