//
// Created by me on 6/30/24.
//

#ifndef DILDONICA_BLEMIDI_CIRCULARBUFFER_H
#define DILDONICA_BLEMIDI_CIRCULARBUFFER_H

template <typename T, size_t CAPACITY>
class CircularQueue {
private:
    T buffer[CAPACITY];
    size_t front;
    size_t rear;
    size_t size;

public:
    CircularQueue() : front(0), rear(0), size(0) {}

    void enqueue(const T& item) {
        buffer[rear] = item;
        rear = (rear + 1) % CAPACITY;
       if (is_full()) {
           return;
       }
        size++;
    }

    T dequeue() {
        if (is_empty()) {
            while(1){};
        }
        T item = buffer[front];
        front = (front + 1) % CAPACITY;
        size--;
        return item;
    }

    T peek() const {
        if (is_empty()) {
            while(1){};
        }
        return buffer[front];
    }

    bool is_empty() const {
        return size == 0;
    }

    bool is_full() const {
        return size == CAPACITY;
    }

    size_t get_size() const {
        return size;
    }
};

#endif //DILDONICA_BLEMIDI_CIRCULARBUFFER_H
