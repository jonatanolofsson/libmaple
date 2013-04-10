#ifndef DMA_MESSAGE_H_
#define DMA_MESSAGE_H_

struct dma_message {
    volatile bool transmitting;
    void* arg;
    volatile unsigned char* data;
    int length;
    typedef void(*dma_msg_callback)(dma_message*, dma_irq_cause);
    dma_msg_callback callback;

    dma_message() : transmitting(false) {}

    bool ready() {
        return !transmitting;
    }

    void wait() {
        while(!ready())
            ;
    }
    void claim() {
        transmitting = true;
    }
    void ret() {
        transmitting = false;
    }
    dma_message& operator()(volatile unsigned char* const d, const int l, dma_msg_callback cb = NULL, void* a = NULL) {
        data = d;
        length = l;
        callback = cb;
        arg = a;
        return *this;
    }
};


template<typename T, void(T::*FN)(dma_message* msg, dma_irq_cause cause)>
inline void dmaMemberCallback(dma_message* msg, dma_irq_cause cause) {
    (((T*)msg->arg)->*FN)(msg, cause);
}
#endif
