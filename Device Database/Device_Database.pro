# -------------------------------------------------
# Project created by QtCreator 2009-04-22T11:17:17
# -------------------------------------------------
TARGET = Device_Database
TEMPLATE = app
SOURCES += main.cpp \
    DeviceWindow.cpp \
    DeviceXmlLoader.cpp \
    "../PC Software/Bootload/Device.cpp"
HEADERS += DeviceWindow.h \
    DeviceXmlLoader.h \
    "../PC Software/Bootload/Device.h"
win32 { 
    RC_FILE = windows.rc
    QMAKE_CXXFLAGS_RELEASE = -Os
}
FORMS += DeviceWindow.ui
RESOURCES += resources.qrc
OTHER_FILES += devices.sql \
    windows.rc
