/*
 * buffered_serial.h
 *
 * Implements a variable size buffer for received serial data
 *
 * Author: Nick Young (9/26/10)
 * Portions of code by: Richard Sewel (11/17/09)
 *
 */

#include "mbed.h"
#include "buffered_serial.h"

/**
 * Constructs a BufferedSerial object
 *
 * Creates a BufferedSerial with default settings of 9600 8N1.
 * Valid (TX/RX) pairs are {(USBTX/USBRX), (9/10), (13/14), (28/27)}.
 * Valid baud rates are {110, 300, 600, 1200, 2400, 4800, 9600, 14400,
 * 19200, 38400, 57600, 115200, 230400, 460800, 921600}.
 * Maximum buffer size is 256 bytes.
 *
 * @param PinName tx
 *   The transmit out (TXO) pin for this serial port
 * @param PinName rx
 *   The receive in (RXI) pin for this serial port
 * @param uint8_t buffer_size
 *   The desired size of the circular buffer
 */
BufferedSerial::BufferedSerial(PinName tx, PinName rx, uint8_t buffer_size) :
  Serial(tx,  rx),
  has_callback_(false)
{
  buffer_size_ = 0;
  content_start_ = 0;
  content_end_ = 0;
  available_bytes_ = 0;
    
  attach(this, &BufferedSerial::handleInterrupt);
    
  buffer_ = (uint8_t *) malloc(buffer_size + 1);
  if(buffer_ != NULL)
  {
    buffer_size_ = buffer_size + 1;
  }
}

BufferedSerial::~BufferedSerial()
{
  if(buffer_)
    free(buffer_);
}

/**
 * Checks if there are bytes ready to be read
 *
 * @return bool
 *  true    if there are bytes ready to be read
 *  false   otherwise
 */
bool BufferedSerial::readable()
{
  return (content_start_ != content_end_);
}

/**
 * Checks how many bytes are ready to be read
 *
 * @return uint8_t
 *  The number of bytes ready to be read
 */
uint8_t BufferedSerial::availableBytes()
{
  return available_bytes_;
}

/**
 * Reads the next available byte without removing it from the buffer
 *
 * Reads a single byte from the front of the circular buffer (without
 * removing it from the buffer and returns its value.  This method
 * will block until a single byte is available.
 *
 * @return uint8_t
 *  The frontmost byte in the circular receive buffer
 */
uint8_t BufferedSerial::peek()
{
  //block until a byte is available
  while(content_start_ == content_end_) { wait_us(1); }
    
  return buffer_[content_start_];
}

/**
 * Reads the next available received byte
 *
 * Removes a single byte from the front of the circular buffer and
 * returns its value.  This method will block until a single byte
 * is available.
 *
 * @return uint8_t
 *  The frontmost byte in the circular receive buffer
 */
uint8_t BufferedSerial::getc()
{
  //block until a byte is available
  while(content_start_ == content_end_) { wait_us(1); }

  //read the byte and remove it from the buffer
  uint8_t result = buffer_[content_start_++];
  content_start_ =  content_start_ % buffer_size_;
  available_bytes_--;
   
  return result;
}

/**
 * Reads the next four bytes into a long data type
 * 
 * Reads the next four bytes and stores them into a signed long data
 * type.  This method will block until four bytes are available. The
 * first byte is considered to be the MSB, and the last byte is
 * considered to be the LSB.
 *
 * @return long
 *  The front most four bytes in the buffer as a long
 */
long BufferedSerial::readLong()
{
  long result = (getc() << 24) |
                (getc() << 16) |
                (getc() << 8 ) |
                (getc()      );
  return result;
}

/**
 * Reads requested_bytes bytes into a provided buffer
 *
 * Reads a specified number of bytes into a caller-supplied buffer.
 * This method will block until requested_bytes bytes are available.
 * The reads are performed byte-wise and will remove the byte from
 * the circular buffer as it is added to the destination buffer.
 *
 * @param uint8_t* bytes
 *   The pre-allocated destination buffer
 * @param size_t requested_bytes
 *   The number of bytes to move into the buffer
 */
void BufferedSerial::readBytes(uint8_t *bytes, size_t requested_bytes)
{
    for (size_t index = 0; index < requested_bytes; index++)
      bytes[index] = getc();        
}

/**
 * Flushes the receive buffer
 */
void BufferedSerial::flushBuffer()
{
  content_end_ = content_start_;
  available_bytes_ = 0;
}

void BufferedSerial::writeLong(long data)
{
  putc((data >> 24) & 0xFF);
  putc((data >> 16) & 0xFF);
  putc((data >>  8) & 0xFF);
  putc((data      ) & 0xFF);
}

/**
 * Serial receive interrupt handler
 *
 * On receipt of data, this method is run to empty
 * the hardware buffer and fill the circular buffer
 * with the received data.  If the circular buffer is
 * full, incoming data will be dropped.
 */
void BufferedSerial::handleInterrupt()
{
    while(Serial::readable())
    {
        if (content_start_ == (content_end_ + 1) % buffer_size_)
            Serial::getc();
        else
        {
            buffer_[content_end_++] = Serial::getc();
            content_end_ = content_end_ % buffer_size_;
            available_bytes_++;
        }
        //if the buffer is full, then callback the object
        if (has_callback_ && (content_start_ == (content_end_ + 1) % buffer_size_) )
            callback_.call();
    }
}