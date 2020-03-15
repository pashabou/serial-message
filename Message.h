#pragma once

#if not defined (__AVR__) || (__avr__)
#include "Serial.h"
#else
typedef HardwareSerial SerialInterface;
#endif

#include "SerialBuffer.h"

/**
 * Message abstraction for Serial I/O
 */
template <unsigned int Size>
struct Message {
private:
  SerialBuffer<Size> buffer;

public:
  template<unsigned int N>
  Message(SerialInterface si = Serial, int to = 100, char (&str)[N] = "msg")
  : buffer(SerialBuffer<Size>(si, to, str)) {}

  /** 
   * Get data from message and put directly into argument addresses
   * - Returns success of read operation
   * - If unsuccessful, the data is unmodified
   */
  template <typename ...Data>
  inline bool read(Data&... data) {
    constexpr auto size = (0 + ... + sizeof(Data));
    static_assert(Size == size, "Incorrect data size");
    if (!buffer.receive()) return false;
    (buffer >> ... >> data);
    buffer.reset();
    return true;
  }

  /** Write objects directly into message and Serial output */
  template <typename ...Data>
  inline void write(Data... data) {
    constexpr auto size = (0 + ... + sizeof(Data));
    static_assert(Size == size, "Incorrect data size");
    (buffer << ... << data);
    buffer.emit();
  }

  /** Write array directly into message and Serial output */
  template<typename T, unsigned int N>
  inline void write(const T (&obj)[N]) {
    static_assert(N * sizeof(T) == Size, "Incorrect array size");
    for (auto c: obj) buffer << c;
    buffer.emit();
  }
};
