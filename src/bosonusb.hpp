#ifndef BOSONUSB_HPP
#define BOSONUSB_HPP

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>

class BosonUSB : public QObject
{
    Q_OBJECT
        public:
                 explicit BosonUSB(QObject *parent = nullptr);
            QSerialPort * bosonCommandUSB;
        signals:

};

#endif // BOSONUSB_HPP
