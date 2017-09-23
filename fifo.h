#ifndef FIFO_H
#define FIFO_H

class fifo {
    int* array_;
    int size_;
    int zero_index_ = -1;
    int present_index_ = -1;
    int length_ = 0;
  public:
    void append(int x);
    int length() const;
    int operator[](int i) const;
    fifo(int n);
    ~fifo() {
      delete [] array_;
    }
};

#endif

