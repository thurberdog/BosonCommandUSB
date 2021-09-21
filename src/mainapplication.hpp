#ifndef MAINAPPLICATION_HPP
#define MAINAPPLICATION_HPP
#define SERIAL_TIMEOUT 500
// Define COLOR CODES
#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLU "\x1B[34m"
#define MAG "\x1B[35m"
#define CYN "\x1B[36m"
#define WHT "\x1B[37m"
#define RESET "\x1B[0m"
// Define error codes

//#define NoError 0x00

#define R_CAM_DSPCH_BAD_CMD_ID 0x0161
#define R_CAM_DSPCH_BAD_PAYLOAD_STATUS 0x0162
#define R_CAM_PKG_UNSPECIFIED_FAILURE 0x0170
#define R_CAM_PKG_INSUFFICIENT_BYTES 0x017D
#define R_CAM_PKG_EXCESS_BYTES 0x017E
#define R_CAM_PKG_BUFFER_OVERFLOW 0x017F
#define FLIR_RANGE_ERROR 0x0203
#define BYTES_READ 1
#define COLARX 100
#define COLATX 10
#include "usb.hpp"
#include <QByteArray>
#include <QCoreApplication>
#include <QDebug>
#include <QObject>
#include <QTimer>
#include <cstring>
#include <fcntl.h>
#include <math.h> // For POW function
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

class MainApplication : public QObject {
  Q_OBJECT
public:
  explicit MainApplication(QObject *parent = nullptr);
  QTimer *timer;
  USB *usb;
  unsigned char NoError = 0x00;
  // Serial Port variables
  char puerto_str[30];  // We give up to 30 chars to put the path
  char baudios_str[10]; // Max size is 921600 ... so we give margin
  int baudios;
  // Boson Function variables
  long commandID = -1;          // Mandatory to receive as input
  char commandID_str[10];       // CommandID is 4 bytes. We give margin as well
  unsigned short num_bytes = 0; // Number of DATA to send
  unsigned char tx_buffer[768]; // Max Boson Data before bit stuffing
  // Auxiliary variables
  int ret;
  char aux_data_str[2];
  char aux_cad[20];
  int aux_data;
  // GLOBAL declaration to have access to them
  // from the other functions. Not the best practise, but simple and fast.

  // ........ Boson Package (Before Bit Stuffing) ................
  unsigned char aux_boson_package[768]; // Max package without bit stuffing 768
  unsigned short aux_boson_package_len = 0; // To storage real size of package
  unsigned char sequence = 0; // This number increases in every SENT commnand.

  // ........ Boson Package (After Bit Stuffing) .............
  unsigned char
      boson_stuffed_package[1544]; // Max package with bit stuffing 1544
  unsigned short boson_stuffed_package_len =
      0; // To storage real size of package

  // ........ Received BOSON requested DATA ..................................
  unsigned char BosonData[768];       // Buffer with DATA from Boson
  unsigned short BosonData_count = 0; // Number of data received

  void print_help();
  void readSerialNumber();
  unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer);
  void Boson_Print_FSLP(unsigned char *data, unsigned short datalen);
  int Receive_Serial_package();
  int flush_buffer_tx(int fd);
  int flush_buffer_rx(int fd);
  int64_t GetTickCount();
  int rxbyte_waiting(int fd);
  int txbyte_waiting(int fd);
  unsigned char read_byte(int fd);
  void get_fpa_temperature();
  void Boson_BitStuffing();
  int Send_Serial_package(unsigned char *package, short package_len);
  int Boson_Check_Received_Frame(unsigned char *pkg);
  void Boson_BitUnstuffing();
  void Boson_Status_Error_Codes(int code);
  long buildBosonPacket(QByteArray cmd);
  int sendBosonPacket();
  int receiveBosonPacket();
  long hex_to_long(char *str);
  int toInt(unsigned char mybyte);
  void enableTlinerOutput();
  void Boson_Build_FSLP(unsigned long function);
public slots:

  void tempturePoll();
  void handleSigTerm();
signals:
};

#endif // MAINAPPLICATION_HPP
