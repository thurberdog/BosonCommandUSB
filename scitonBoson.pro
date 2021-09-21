QT += quick multimedia multimediawidgets charts qml serialport

CONFIG += c++11
VERSION = 0.1.8

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
#LIBS += -lusb-1.0

SOURCES += \
        src/main.cpp \
        src/mainapplication.cpp \
        src/usb.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Default rules for deployment.
target.path = /data/bin
INSTALLS += target

HEADERS += \
    src/mainapplication.hpp \
    src/usb.hpp
