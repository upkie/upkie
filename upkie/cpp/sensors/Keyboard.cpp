// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#include "upkie/cpp/sensors/Keyboard.h"

namespace upkie {
Keyboard::Keyboard() {
  termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  setbuf(stdin, NULL);

  key_pressed_ = false;
  key_code_ = Key::UNKNOWN;

  last_key_poll_time_ = system_clock::now() - milliseconds(kPollingIntervalMS);
}

Keyboard::~Keyboard() {}

bool Keyboard::read_event() {
  ssize_t bytes_available = 0;
  ioctl(STDIN_FILENO, FIONREAD, &bytes_available);

  if (bytes_available) {
    ssize_t bytes_to_read =
        bytes_available < kMaxKeyBytes ? bytes_available : kMaxKeyBytes;
    ssize_t bytes_read = ::read(STDIN_FILENO, &buf_, bytes_to_read);

    if (bytes_read < bytes_available) {
      // Skip the remaining bytes
      do {
        unsigned char garbage[bytes_available - bytes_read];
        bytes_read =
            ::read(STDIN_FILENO, &garbage, bytes_available - bytes_read);
        ioctl(STDIN_FILENO, FIONREAD, &bytes_available);
      } while (bytes_available);
    }
    return 1;
  }
  return 0;
}

Key Keyboard::map_char_to_key(unsigned char* buf) {
  // Check for 3-byte characters first (i.e. arrows)
  if (!memcmp(buf_, DOWN_BYTES, kMaxKeyBytes)) {
    return Key::DOWN;
  }
  if (!memcmp(buf_, UP_BYTES, kMaxKeyBytes)) {
    return Key::UP;
  }
  if (!memcmp(buf_, LEFT_BYTES, kMaxKeyBytes)) {
    return Key::LEFT;
  }
  if (!memcmp(buf_, RIGHT_BYTES, kMaxKeyBytes)) {
    return Key::RIGHT;
  }

  // If the first byte corresponds to a lowercase ASCII alphabetic
  if (is_lowercase_alpha(buf[0])) {
    buf[0] -= 32;  // Map to uppercase equivalent
  }

  // We treat any printable ASCII as a single key code
  if (is_printable_ascii(buf[0])) {
    switch (buf[0]) {
      case 87:  // 0x57
        return Key::W;
      case 65:  // 0x41
        return Key::A;
      case 83:  // 0x53
        return Key::S;
      case 68:  // 0x44
        return Key::D;
      case 88:  // 0x58
        return Key::X;
    }
  }
  return Key::UNKNOWN;
}

void Keyboard::write(Dictionary& observation) {
  // Check elapsed time since last key polling
  auto elapsed = system_clock::now() - last_key_poll_time_;
  auto elapsed_ms = duration_cast<milliseconds>(elapsed).count();

  // Poll for key press if enough time has elapsed or if no key is pressed
  if (elapsed_ms >= kPollingIntervalMS || !key_pressed_) {
    key_pressed_ = read_event();

    if (key_pressed_) {
      key_code_ = map_char_to_key(buf_);
    } else {
      key_code_ = Key::NONE;
    }

    last_key_poll_time_ = system_clock::now();
  }

  auto& output = observation(prefix());
  output("key_pressed") = key_pressed_;
  output("up") = key_code_ == Key::UP;
  output("down") = key_code_ == Key::DOWN;
  output("left") = key_code_ == Key::LEFT;
  output("right") = key_code_ == Key::RIGHT;
  output("w") = key_code_ == Key::W;
  output("a") = key_code_ == Key::A;
  output("s") = key_code_ == Key::S;
  output("d") = key_code_ == Key::D;
  output("x") = key_code_ == Key::X;
  output("unknown") = key_code_ == Key::UNKNOWN;
}

}  // namespace upkie
