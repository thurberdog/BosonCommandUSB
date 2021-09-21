#include "usb.hpp"

USB::USB(QObject *parent) : QObject(parent) {
  // Serial Port variables
  char puerto_str[30]; // We give up to 30 chars to put the path
  int baudios;
  // Auxiliary variables
  int ret;
  char aux_cad[20];
  // Default BOSON values
  strcpy(puerto_str, "/dev/ttyACM0");
  baudios = 921600;
  qDebug(aux_cad, "%i,8,n,1", baudios);
  qDebug() << __FUNCTION__ << __LINE__ << "aux_cad " << aux_cad << "baudios "
           << baudios;
  // open serial port
  puerto_serie_conf = str2ps(puerto_str, aux_cad);
  ret = open_port(puerto_serie_conf, &serial);
  qDebug() << __FUNCTION__ << __LINE__ << "open serial port return code  "
           << ret;
}

// Opens serial port
// Input  : PortSettingType
// Output : handle to the device
// Returns 0 on success
// Returns -1 if not capable of opening the serial port handler
// Returns -2 if serial port settings cannot be applied
/**
 * @brief USB::open_port
 * @param ps
 * @param handle
 * @return
 */
int USB::open_port(PortSettingsType ps, int *handle) {
  //    ps.port = "/dev/ttyACM0";
  if ((*handle = open(ps.port, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
    qDebug() << __FUNCTION__ << __LINE__ << "cannot open serial port "
             << ps.port;

    return -1; // cannot open serial port
  } else {
    if (setup_serial(*handle, ps.baudrate, ps.databits, ps.stopbits,
                     ps.parity) != 0) {
      close(*handle);
      return -2; // cannot apply settings
    } else {
      return 0; // Serial port opened and configured correctly
    }
  }
}

// Close the serial port handle
// Returns 0 on success
// Returns -1 on error , and errno is updated.
int USB::close_port(int handle) { return close(handle); }

// Set the serial port configuration ( baudrate, databits, stopbits, parity)
// Returns  0 if OK
// Returns -1 if cannot get serial port configuration
// Returns -2 if baudrate not supported
// Returns -3 if selected baudrate didn't work
// Returns -4 if databits selection failed
// Returns -5 if parity selection failed
// Returns -6 if stopbits selection failed
// Returns -7 if cannot update new options
/**
 * @brief USB::setup_serial
 * @param fdes
 * @param baud
 * @param databits
 * @param stopbits
 * @param parity
 * @return
 */
int USB::setup_serial(int fdes, int baud, int databits, int stopbits,
                      int parity) {
  int n;
  struct termios options;

  // Get the current options
  if (tcgetattr(fdes, &options) != 0) {
    // error getting the serial port configuration options
    return -1;
  }

  // Set the baud rate
  switch (baud) {
  case 2400:
    n = cfsetospeed(&options, B2400);
    n += cfsetispeed(&options, B2400);
    break;
  case 4800:
    n = cfsetospeed(&options, B4800);
    n += cfsetispeed(&options, B4800);
    break;
  case 9600:
    n = cfsetospeed(&options, B9600);
    n += cfsetispeed(&options, B9600);
    break;
  case 19200:
    n = cfsetospeed(&options, B19200);
    n += cfsetispeed(&options, B19200);
    break;
  case 38400:
    n = cfsetospeed(&options, B38400);
    n += cfsetispeed(&options, B38400);
    break;
  case 57600:
    n = cfsetospeed(&options, B57600);
    n += cfsetispeed(&options, B57600);
    break;
  case 115200:
    n = cfsetospeed(&options, B115200);
    n += cfsetispeed(&options, B115200);
    break;
  case 230400:
    n = cfsetospeed(&options, B230400);
    n += cfsetispeed(&options, B230400);
    break;
  case 921600:
    n = cfsetospeed(&options, B921600);
    n += cfsetispeed(&options, B921600);
    break;

  default:
    // not supported baudrate
    return -2;
  }
  // If n != 0 then Baud Rate selection didn't work
  if (n != 0) {
    return -3; // Error settig the baud rate
  }

  // Set the data size
  options.c_cflag &= ~CSIZE;
  switch (databits) {
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    // Not supported data size
    return -4;
  }

  // Set up parity
  switch (parity) {
  case NOPARITY:
    options.c_cflag &= ~PARENB; // Clear parity enable
    options.c_iflag &= ~INPCK;  // Enable parity checking
    break;
  case ODDPARITY:
    options.c_cflag |= (PARODD | PARENB); // Enable odd parity
    options.c_iflag |= INPCK;             // Disnable parity checking
    break;
  case EVENPARITY:
    options.c_cflag |= PARENB;  // Enable parity
    options.c_cflag &= ~PARODD; // Turn odd off => even
    options.c_iflag |= INPCK;   // Disnable parity checking
    break;
  default:
    // Unsupported parity
    return -5;
  }
  // Set up stop bits
  switch (stopbits) {
  case ONESTOPBIT:
    options.c_cflag &= ~CSTOPB;
    break;
  case TWOSTOPBITS:
    options.c_cflag |= CSTOPB;
    break;
  default:
    // "Unsupported stop bits
    return -6;
  }

  // Set input parity option
  if (parity != NOPARITY)
    options.c_iflag |= INPCK;

  // Deal with hardware or software flow control
  options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
  // options.c_iflag |= (IXANY); // xon/xoff flow control
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // xon/xoff flow control

  // Output processing
  options.c_oflag &= ~OPOST; // No output processing
  options.c_oflag &= ~ONLCR; // Don't convert linefeeds

  // Input processing
  options.c_iflag |= IGNBRK;  // Ignore break conditions
  options.c_iflag &= ~IUCLC;  //  Don't map upper to lower case
  options.c_iflag &= ~BRKINT; // Ignore break signals
  options.c_iflag &= ~INLCR;  // Map NL to CR
  options.c_iflag &= ~ICRNL;  // Map CR to NL

  // Miscellaneous stuff
  options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local
  // Linux seems to have problem with the following ?
  //	options.c_cflag |= (IXON | IXOFF); // Software flow control
  options.c_lflag = 0;      // no local flags
  options.c_cflag |= HUPCL; // Drop DTR on close

  // Setup non blocking, return on 1 character
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;

  // Clear the line
  tcflush(fdes, TCIFLUSH);

  // Update the options and do it NOW
  if (tcsetattr(fdes, TCSANOW, &options) != 0) {
    return -7;
  }

  return 0; // Serial port configured correctly
}

// Send buffer of  bytes
// Input: serial port handler, pointer to first byte of buffer , number of bytes
// to send Returns 0 on success Returns -1 on error
/**
 * @brief USB::send_buffer
 * @param fd
 * @param tx_array
 * @param bytes_to_send
 * @return
 */
int USB::send_buffer(int fd, unsigned char *tx_array, short bytes_to_send) {
  if (write(fd, tx_array, bytes_to_send) == -1) {
    qDebug() << __LINE__ << __FUNCTION__ << " Failed to send buffer ";
    return -1; // Error
  }
  QByteArray data = QByteArray::fromRawData(reinterpret_cast<char *>(tx_array),
                                            bytes_to_send);

  qDebug() << __LINE__ << __FUNCTION__ << ">>>"
           << " Frame sent " << data.toHex() << " " << bytes_to_send
           << " bytes";
  return 0;
}

// Send one byte
// Input: serial port handler, byte to send
// Returns 0 on success
// Returns -1 on error
int USB::send_byte(int fd, unsigned char car) {
  if (write(fd, &car, 1) == -1) {
    return -1;
  }
  return 0;
}

// Check if there is a byte or not waiting to be read (RX)
// Returns 'n' > 0 as numbers of bytes to read
// Returns 0 is there is no byte waiting
int USB::rxbyte_waiting(int fd) {
  int n;
  if (ioctl(fd, TIOCINQ, &n) == 0) {
    return n;
  }
  return 0;
}

// Check if there is a byte or not waiting to be sent (TX)
// Returns 'n' > 0 as numbers of bytes to send
// Returns 0 is there is no byte waiting
int USB::txbyte_waiting(int fd) {
  int n = 0;
  if (ioctl(fd, TIOCOUTQ, &n) == 0) {
    return n;
  }
  return 0;
}
// Read a byte within a period of time
// Returns byte read
// Returns -1 if timeout happened.
// timeout = 0 if no timeout, timeout = 1 if timeout
unsigned char USB::read_byte_time(int fd, int plazo, int *timeout) {
  fd_set leer;
  struct timeval tout;
  int n;
  unsigned char c;

  tout.tv_sec = 0;
  tout.tv_usec = plazo;

  FD_ZERO(&leer);
  FD_SET(fd, &leer);

  n = select(fd + 2, &leer, NULL, NULL, &tout);
  if (n == 0) {
    *timeout = 1;
    return -1;
  }
  *timeout = 0;

  read(fd, &c, 1);
  return c;
}

// Read a byte. Blocking call. waits until byte is received
// Returns byte read
unsigned char USB::read_byte(int fd) {
  unsigned char c;

  while (!rxbyte_waiting(fd))
    ;
  read(fd, &c, 1);
  return c;
}

// Sends break signal
// Returns -1 if error
// Returns 0 if OK
int USB::send_break(int fd) {
  if (tcsendbreak(fd, 1) == -1) {
    return -1;
  }
  return 0;
}

// Define serial port settings
// str1 -> serial port name
// str2 -> serial port setting baud,databits, parity, stop bits
// output PS structure
PortSettingsType USB::str2ps(char *str1, char *str2) {

  PortSettingsType ps;
  char parity;
  char stopbits[4];

  // Default values (just in case)
  ps.baudrate = 9600;
  ps.databits = 8;
  ps.parity = NOPARITY;
  ps.stopbits = ONESTOPBIT;

  qDebug(ps.port, "%s", str1);
  if (sscanf(str2, "%d,%d,%c,%s", &ps.baudrate, &ps.databits, &parity,
             stopbits) == 4) {
    switch (parity) {
    case 'e':
      ps.parity = EVENPARITY;
      break;
    case 'o':
      ps.parity = ODDPARITY;
      break;
    case 'm':
      ps.parity = MARKPARITY;
      break;
    case 's':
      ps.parity = SPACEPARITY;
      break;
    case 'n':
      ps.parity = NOPARITY;
      break;
    }
    if (!strcmp(stopbits, "1")) {
      ps.stopbits = ONESTOPBIT;
    } else if (!strcmp(stopbits, "1.5")) {
      ps.stopbits = ONE5STOPBITS;
    } else if (!strcmp(stopbits, "2")) {
      ps.stopbits = TWOSTOPBITS;
    }
  }
  return ps;
}
