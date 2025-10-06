#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <stdint.h>
#include <cstddef>
#include <hardware/sync.h>

// Lock-free (IRQ-safe) ring buffer for 16-bit event words.
// Single-producer/single-consumer usage with short critical sections
// guarded by save_and_disable_interrupts()/restore_interrupts().

constexpr size_t BUFFER_SIZE = 256;

// Fixed-size circular queue storing 16-bit packets
class RingBuffer
{
private:
  alignas(16) uint16_t buffer[BUFFER_SIZE];
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t count = 0;

public:
  // Enqueue an item; returns false if buffer is full
  inline bool push(uint16_t event_data) noexcept
  {
    uint32_t status = save_and_disable_interrupts();

    if (count >= BUFFER_SIZE)
    {
      restore_interrupts(status);
      return false;
    }

    buffer[head] = event_data;
    head = (head + 1) & (BUFFER_SIZE - 1);
    ++count;

    restore_interrupts(status);
    return true;
  }

  // Dequeue an item; returns false if buffer is empty
  inline bool pop(uint16_t &out) noexcept
  {
    uint32_t status = save_and_disable_interrupts();

    if (count == 0)
    {
      restore_interrupts(status);
      return false;
    }

    out = buffer[tail];
    tail = (tail + 1) & (BUFFER_SIZE - 1);
    --count;

    restore_interrupts(status);
    return true;
  }

  // State helpers (non-mutating)
  inline bool is_empty() const noexcept { return count == 0; }
  inline bool is_full() const noexcept { return count == BUFFER_SIZE; }
  inline size_t size() const noexcept { return count; }
  inline uint8_t utilization_percent() const noexcept { return (count * 100) / BUFFER_SIZE; }

  // Clear buffer indices and count atomically
  inline void clear() noexcept
  {
    uint32_t status = save_and_disable_interrupts();
    head = tail = count = 0;
    restore_interrupts(status);
  }
};

#endif

// EOF
