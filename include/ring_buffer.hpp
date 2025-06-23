#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <stdint.h>
#include <cstddef>

constexpr size_t BUFFER_SIZE = 32;

class RingBuffer
{
private:
  uint16_t buffer[BUFFER_SIZE];
  volatile int head = 0;
  volatile int tail = 0;
  volatile int count = 0;

public:
  inline bool push(uint16_t event_data)
  {
    uint32_t status = save_and_disable_interrupts();

    if (count >= BUFFER_SIZE)
    {
      restore_interrupts(status);
      return false;
    }

    buffer[head] = event_data;
    head = (head + 1) % BUFFER_SIZE;
    ++count;

    restore_interrupts(status);
    return true;
  }

  inline bool pop(uint16_t &out)
  {
    uint32_t status = save_and_disable_interrupts();

    if (count == 0)
    {
      restore_interrupts(status);
      return false;
    }

    out = buffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    --count;

    restore_interrupts(status);
    return true;
  }

  inline bool is_empty() const
  {
    return count == 0;
  }

  inline bool is_full() const
  {
    return count == BUFFER_SIZE;
  }

  inline size_t size() const
  {
    return count;
  }

  inline void clear()
  {
    uint32_t status = save_and_disable_interrupts();
    head = tail = count = 0;
    restore_interrupts(status);
  }

  inline uint8_t utilization_percent() const
  {
    return (count * 100) / BUFFER_SIZE;
  }
};

#endif

// EOF
