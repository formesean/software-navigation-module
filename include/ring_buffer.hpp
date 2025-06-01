#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <stdint.h>
#include <cstddef>

#include "packet.hpp"

constexpr size_t BUFFER_SIZE = 32;

class RingBuffer
{
private:
  Packet buffer[BUFFER_SIZE];
  int head = 0;
  int tail = 0;
  int count = 0;

public:
  bool push(const Packet &p)
  {
    if (count >= BUFFER_SIZE)
      return false;

    buffer[head] = p;
    head = (head + 1) % BUFFER_SIZE;
    ++count;
    return true;
  }

  bool pop(Packet &out)
  {
    if (count == 0)
      return false;

    out = buffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    --count;
    return true;
  }

  bool is_empty() const
  {
    return count == 0;
  }

  bool is_full() const
  {
    return count == BUFFER_SIZE;
  }

  size_t size() const
  {
    return count;
  }

  void clear()
  {
    head = tail = count = 0;
  }
};

#endif

// EOF
