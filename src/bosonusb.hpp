#ifndef BOSONUSB_HPP
#define BOSONUSB_HPP
#include <chrono>   // std::chrono::seconds
#include <iostream> // std::cout, std::endl
#include <thread>   // std::this_thread::sleep_for

#include <QDebug>
#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

class BosonUSB : public QObject {
  Q_OBJECT
  int sendPacket(QByteArray &data, QByteArray &crc);

public:
  explicit BosonUSB(QObject *parent = nullptr);
  QSerialPort *bosonCommandUSB;
signals:
private slots:
  void onReadyRead();

  void onSendDataTimeout();

  void onErrorOccurred(QSerialPort::SerialPortError error);

private:
  qint64 result;
  bool flush;
  unsigned char sequence = 0; // This number increases in every SENT commnand.
  // FLR_ROI   region of interest
  uint16_t rowStart;
  uint16_t rowStop;
  uint16_t columnStart;
  uint16_t columnStop;

  QTimer *sendTimer;
  int getFPAtemperature();
  int getSerialNumber();
  int flatFieldCorrection();
  unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer);
  qint64 sendBosonCommand(QByteArray bosonCommand);
  int enableRadiometry();
  int agcSetROI(uint16_t rowStart, uint16_t rowStop, uint16_t columnStart,
                uint16_t columnStop);
  int roicGetFPATemp();
  int roicGetFPATempTable();
  int roicGetFPATempValue();
  int bosonGetMyriadTemp();
  int captureFrames(int frameCount);
  int captureSingleFrameWithSrc();
};

#endif // BOSONUSB_HPP
