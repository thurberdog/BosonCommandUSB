#include "bosonusb.hpp"

BosonUSB::BosonUSB(QObject *parent) : QObject(parent)
{
    bosonCommandUSB = new QSerialPort(this);
    bosonCommandUSB->setPortName("/dev/ttyACM0");
    bosonCommandUSB->setBaudRate(QSerialPort::Baud115200);
    bosonCommandUSB->setDataBits(QSerialPort::Data8);
    bosonCommandUSB->setParity(QSerialPort::NoParity);
    bosonCommandUSB->setStopBits(QSerialPort::OneStop);
    bosonCommandUSB->setFlowControl(QSerialPort::NoFlowControl);

    if (bosonCommandUSB->open(QIODevice::ReadWrite))
    {
        //Connected
        qDebug() << __LINE__ << __FUNCTION__ << "Boson Command channel open. " ;
    }
    else
    {
        //Open error
        qDebug() << __LINE__ << __FUNCTION__ << "Could not open Boson Command channel. "<< bosonCommandUSB->errorString();
    }
}
