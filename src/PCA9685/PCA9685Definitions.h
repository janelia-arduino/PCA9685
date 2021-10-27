// ----------------------------------------------------------------------------
// PCA9685.h
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef PCA9685_DEFINITIONS_H
#define PCA9685_DEFINITIONS_H


template<typename T>
void PCA9685::write(DeviceAddress device_address,
  uint8_t register_address,
  T data)
{
  int byte_count = sizeof(data);
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  uint8_t write_byte;
  for (int byte_n=0; byte_n<byte_count; ++byte_n)
  {
    write_byte = (data >> (BITS_PER_BYTE * byte_n)) & BYTE_MAX;
    wire_ptr_->write(write_byte);
  }
  wire_ptr_->endTransmission();
}

template<typename T>
void PCA9685::read(DeviceIndex device_index,
  uint8_t register_address,
  T & data)
{
  int byte_count = sizeof(data);
  int device_address = device_addresses_[device_index];
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->endTransmission();

  wire_ptr_->requestFrom(device_address,byte_count);
  data = 0;
  for (int byte_n=0; byte_n<byte_count; ++byte_n)
  {
    data |= (wire_ptr_->read()) << (BITS_PER_BYTE * byte_n);
  }
}

#endif
