#ifndef BOSONUSB_HPP
#define BOSONUSB_HPP

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
  QTimer *sendTimer;
  int getFPAtemperature();
  int getSerialNumber();
  int flatFieldCorrection();
  unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer);
  qint64 sendBosonCommand(QByteArray bosonCommand);
};

#endif // BOSONUSB_HPP
