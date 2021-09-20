#ifndef BOSONUSB_HPP
#define BOSONUSB_HPP

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>

class BosonUSB : public QObject
{
    Q_OBJECT
        int sendPacket(QByteArray &data, QByteArray &crc);
public:
                 explicit BosonUSB(QObject *parent = nullptr);
            QSerialPort * bosonCommandUSB;
signals:
private slots:
                 void onReadyRead();
private:
    unsigned char sequence=0;               // This number increases in every SENT commnand.

    int getFPAtemperature();
    int getSerialNumber();
    int flatFieldCorrection();
    unsigned short CalcBlockCRC16(unsigned int bufferlen, unsigned char *buffer);
};

#endif // BOSONUSB_HPP
