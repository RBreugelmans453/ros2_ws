#ifndef WAVESHARE_REAL_ARDUINO_COMMS_HPP
#define WAVESHARE_REAL_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    std::cout << "Serial connected" << std::endl;
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
    std::cout << "Received: " << val_1 << std::endl;
  }

  void read_imu_values(float& acceX, float& acceY, float& acceZ, float& gyroX, float& gyroY, float& gyroZ, float& magX, float& magY, float& magZ, float& imuR, float& imuP, float& imuY)
  {
    std::string response = send_msg("c\r");
    // Trim leading and trailing whitespace and commas
    size_t start = response.find_first_not_of(" ,");
    size_t end = response.find_last_not_of(" ,");
    response = response.substr(start, end - start + 1);

    std::stringstream ss(response);
    std::string token;
    std::string imu_check;
    int tokenCount = 0;
    std::cout << "received: " << response << std::endl;
    while (std::getline(ss, token, ',')) {
      // Skip empty tokens
      if (token.empty()) continue;
      tokenCount++;
      
      switch (tokenCount) {
          case 1: imu_check = token;
          if (imu_check != "imu") {
            return;
          }
          break;
          case 2: acceX = std::stof(token); break;
          case 3: acceY = std::stof(token); break;
          case 4: acceZ = std::stof(token); break;
          case 5: gyroX = std::stof(token); break;
          case 6: gyroY = std::stof(token); break;
          case 7: gyroZ = std::stof(token); break;
          case 8: magX = std::stof(token); break;
          case 9: magY = std::stof(token); break;
          case 10: magZ = std::stof(token); break;
          case 11: imuR = std::stof(token); break;
          case 12: imuP = std::stof(token); break;
          case 13: imuY = std::stof(token); break;
      }
      //std::cout << "Received: " << imu_check << std::endl;
    }    
  }



  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    int val_tbs_1;
    int val_tbs_2;
    
    //check whether the values indicate turning, to increase the motor speed
    if (val_1 == val_2) {
      val_tbs_1 = val_1;
      val_tbs_2 = val_2;
    }

    //opposite values mean turning
    else if (val_1 == -val_2 || val_2 == -val_1) {
      val_tbs_1 = val_1 * 15;
      val_tbs_2 = val_2 * 15;
    }
    ss << "m " << val_tbs_1 << " " << val_tbs_2 << "\r";
    send_msg(ss.str());
    //std::cout << "Sent: " << ss.str() << std::endl;
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};



#endif