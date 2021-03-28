#pragma once

#include <cstdint>

template <typename type_t, size_t default_size>
struct small_vector_t {

  small_vector_t()
    : head(0)
  {
  }

  size_t size() const {
    return head;
  }

  bool empty() const {
    return head == 0;
  }

  void push_back(const type_t &t) {
    assert(head < data.size());
    data[head++] = t;
  }

  type_t &front() {
    assert(!empty());
    return data[0];
  }

  type_t &back() {
    assert(!empty());
    return data[head - 1];
  }

  type_t *begin() const {
    return &data[0];
  }

  type_t *end() const {
    return &data[head];
  }

  void clear() {
    head = 0;
  }

  void pop() {
    assert(!empty());
    --head;
  }

protected:
  size_t head;
  std::array<type_t, default_size> data;
};
