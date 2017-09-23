#include "Arduino.h"
#include "fifo.h"

fifo::fifo(int n) : size_(n)
{
  array_ = new int[size_];
}

int fifo::length() const
{
  return length_;
}

void fifo::append(int x)
{
  present_index_ = present_index_ < (size_ - 1) ? ++present_index_ : 0;
  array_[present_index_] = x;
  length_ = min(length_ + 1, size_);
  zero_index_ = present_index_ + 1 < length_ ? present_index_ + 1 : 0;
}

int fifo::operator[](int i) const
{
  int index = ((zero_index_ + i) % size_);
  return array_[index];
}

