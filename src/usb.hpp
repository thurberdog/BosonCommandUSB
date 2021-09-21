#ifndef USB_HPP
#define USB_HPP
#define BYTES_READ 1
#define COLARX 100
#define COLATX 10
#define NOPARITY 0
#define ODDPARITY 1
#define EVENPARITY 2
#define MARKPARITY 3
#define SPACEPARITY 4
#define ONESTOPBIT 0
#define ONE5STOPBITS 1
#define TWOSTOPBITS 2
#include <QDebug>
#include <QObject>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

typedef struct PortSettings {
  char port[13] = "/dev/ttyACM0";
  int baudrate;
  int databits;
  int parity;
  int stopbits;
} PortSettingsType;

class USB : public QObject {
  Q_OBJECT
public:
  explicit USB(QObject *parent = nullptr);
  int HANDLE;
  PortSettingsType puerto_serie_conf;
  int serial;
  int open_port(PortSettingsType ps, int *handle);
  int close_port(int handle);
  int setup_serial(int fdes, int baud, int databits, int stopbits, int parity);
  PortSettingsType str2ps(char *str1, char *str2);
  int send_buffer(int fd, unsigned char *tx_array, short bytes_to_send);
  int send_byte(int fd, unsigned char car);
  int rxbyte_waiting(int fd);
  int txbyte_waiting(int fd);
  unsigned char read_byte_time(int fd, int plazo, int *timeout);
  unsigned char read_byte(int fd);
  int send_break(int fd);
signals:
};

#endif // USB_HPP
