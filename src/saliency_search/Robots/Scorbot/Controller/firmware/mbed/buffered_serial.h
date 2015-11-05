/*
 * buffered_serial.h
 *
 * Defines a variable size buffer for received serial data
 *
 * Original code by: Richard Sewell (11/17/09)
 * Changes by: Nick Young (9/26/10)
 *
 */

#ifndef BUFFERED_SERIAL_H
#define BUFFERED_SERIAL_H

/**
 * Serial wrapper providing a variable sized RX buffer
 *
 * Used to store serial data received in a circular buffer in cases
 * where the data stream is larger than the hardware buffer (16 bytes)
 * and the data is not immediately processed. On RXIRQ interrupt
 * (i.e., on receipt of data), new data is retreived from the hardware
 * serial buffer and stored in a circular buffer.  If the circular
 * buffer becomes full, no additional data will be added until some
 * of the current data has been removed.
 *
 */
class BufferedSerial : public Serial
{
public:
  BufferedSerial(PinName tx, PinName rx, uint8_t bufferSize);
  virtual ~BufferedSerial();

  bool readable();
  uint8_t availableBytes();
  uint8_t peek();
  uint8_t getc();
  long readLong();
  void readBytes(uint8_t *bytes, size_t requested);
  void flushBuffer();
  void writeLong(long data);
  
  /**
  * Set a callback function when buffer fills
  *
  * Set a member function on a particular object to be
  * called once the buffer becomes full.  While this 
  * this callback function is running, no additional
  * data will be received (even if the callback function
  * immediately empties the buffer).  If this becomes an
  * issue, we can add a Timeout to call the callback
  * 1us later, which would allow us to receive data again.
  * 
  * @param T* object
  *  The object on whom the callback should be made
  * @param void (T::*member)(void)
  *  The member which should be called back
  */ 
  template<typename T> void setFullCallback(T * object, void (T::*member)(void))
  {
    callback_ = FunctionPointer(object, member);
    has_callback_ = true;
  }

private:
  void handleInterrupt();

  /// Circular buffer of bytes
  uint8_t *buffer_;
  /// Index of the first byte of data
  uint8_t content_start_;
  /// Index of the first free byte (index after the last byte of data)
  uint8_t content_end_;
  /// Number of bytes in the buffer
  uint8_t buffer_size_;
  /// Number of bytes filled in the data buffer (ready to be read)
  uint8_t available_bytes_;
  /// Callback method handle
  FunctionPointer callback_;
  bool has_callback_;
};

#endif /* BUFFERED_SERIAL_H */