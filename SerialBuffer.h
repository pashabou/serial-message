#if not defined MESSAGE_START_SEQUENCE
#define MESSAGE_START_SEQUENCE "msg"
#endif

#if not defined MESSAGE_TIMEOUT_MILLIS
#define MESSAGE_TIMEOUT_MILLIS 100
#endif

#if not defined (__AVR__) || (__avr__)
#define TIMEOUT(condition)                                                \
    using namespace std::chrono;                                          \
    auto loopStart = high_resolution_clock::now();                        \
    while ((condition) &&                                                 \
      duration_cast<milliseconds>(high_resolution_clock::now() -          \
      loopStart).count() < MESSAGE_TIMEOUT_MILLIS)
#else
#define TIMEOUT(condition)                                                \
    auto loopStart = millis();                                            \
    while ((condition) && (millis() - loopStart) < MESSAGE_TIMEOUT_MILLIS)
#endif

#include "CRC32.h"

/**
 * Buffer for serial I/O
 */
template <unsigned int Size>
struct SerialBuffer {
private:
  char buffer[Size]{};
  unsigned int offset = 0;
  static constexpr char StartSeq[] = MESSAGE_START_SEQUENCE;

public:
  SerialBuffer() {}

  /** Prepare buffer for reuse */
  inline void reset() { offset = 0; }

  /** Receive buffer from serial input
   *  Fails after MESSAGE_TIMEOUT_MILLIS milliseconds
   *    OR if computed hash doesn't match message hash
   */
  inline bool receive() {
    uint32_t hash;
    char start = 0;
    int startIdx = 0;
    {TIMEOUT(startIdx < sizeof(StartSeq)) {
      while (Serial.available() && startIdx < sizeof(StartSeq)) {
        Serial.readBytes(&start, 1);
        if (start == StartSeq[startIdx])
          startIdx++;
        else if (startIdx != 0)
          startIdx = 0;
      }
    }}
    if (startIdx < sizeof(StartSeq)) return false;
    {TIMEOUT(Serial.available() < Size + sizeof(hash));}
    if (Serial.available() >= Size + sizeof(hash)) {
      Serial.readBytes(buffer, Size);
      Serial.readBytes((char*)&hash, sizeof(hash));
    } else return false;
    CRC32<Size> computedHash(buffer);
    return hash == computedHash.crc;
  }

  /** Emit buffer to serial output */
  inline void emit() {
    char size = Size;
    CRC32<Size> hash(buffer);
    Serial.write((char*)StartSeq, sizeof(StartSeq));
    Serial.write(buffer, Size);
    Serial.write((char*)&hash.crc, sizeof(hash.crc));
    Serial.flush();
    reset();
  }

  /** Get object from input buffer */
  template <typename T, unsigned int N>
  friend SerialBuffer<N>& operator>>(SerialBuffer<N>& in, const T& obj);

  /** Serialize object into buffer */
  template <typename T, unsigned int N>
  friend SerialBuffer<N>& operator<<(SerialBuffer<N>& out, const T& obj);
};

template <typename T, unsigned int N>
inline SerialBuffer<N>& operator>>(SerialBuffer<N>& in, const T& obj) {
  for (int i = 0; i < sizeof(T) && in.offset < N; i++)
    ((char*)&obj)[i] = in.buffer[in.offset++];
  return in;
}

template <typename T, unsigned int N>
inline SerialBuffer<N>& operator<<(SerialBuffer<N>& out, const T& obj) {
  for (int i = 0; i < sizeof(T) && out.offset < N; i++)
    out.buffer[out.offset++] = ((char*)&obj)[i];
  return out;
}
