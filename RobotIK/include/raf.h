#ifndef RAF_H
#define RAF_H

#include <memory>

namespace robotik {

template<typename T>
class RollingAverage {
public:
    RollingAverage(size_t _size, T init_value = {}) {
      size = _size;
      history = (T *) calloc(size, sizeof(T));
      for (size_t i = 0; i < size; i++)
        new(&history[i]) T(init_value);
      p = 0;
      acc = init_value * size;
    }

    RollingAverage(const RollingAverage &copy) : size(copy.size), p(copy.p), acc(copy.acc) {
      history = (T *) calloc(size, sizeof(T));
      for (size_t i = 0; i < size; i++)
        // placement new via T's copy constructor
        new(&history[i]) T(copy.history[i]);
    }

    RollingAverage &operator=(const RollingAverage &copy) {
      if (&copy == this)
        return *this;
      // deallocate old history
      for (size_t i = 0; i < size; i++)
        ::operator delete(&history[i]);
      free(history);
      // copy
      size = copy.size;
      p = copy.p;
      acc = copy.acc;
      history = (T *) calloc(size, sizeof(T));
      for (size_t i = 0; i < size; i++)
        history[i] = copy.history[i];
      return *this;
    }

    ~RollingAverage() {
      for (size_t i = 0; i < size; i++)
        ::operator delete(&history[i]);
      free(history);
    }

    void clear(T init_value) {
      for (size_t i = 0; i < size; i++)
        history[i] = init_value;
      acc = init_value * size;
      p = 0;
    }

    inline void clear() { clear(0); }

    void add(T value) {
      if (++p >= size)
        p = 0;
      acc -= history[p];
      history[p] = value;
      acc += value;
    }

    T average() const {
      return (size > 0) ? acc / size : T();
    }

    T recent() const {
      return history[p];
    }

    void minmax(T &min_out, T &max_out) {
      if (size <= 0) return;
      min_out = max_out = history[0];
      for (size_t i = 1; i < size; i++) {
        if (min_out > history[i])
          min_out = history[i];
        if (max_out < history[i])
          max_out = history[i];
      }
    }

private:
    T *history;
    size_t size;
    T acc;
    size_t p;
};

} // ns:robotik

#endif

