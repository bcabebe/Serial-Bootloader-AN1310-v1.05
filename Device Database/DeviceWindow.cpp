/************************************************************************
* Copyright (c) 2009-2011,  Microchip Technology Inc.
*
* Microchip licenses this software to you solely for use with Microchip
* products.  The software is owned by Microchip and its licensors, and
* is protected under applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED "AS IS."  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT
* NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL
* MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
* CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR
* EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY
* OR SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED
* TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION,
* OR OTHER SIMILAR COSTS.
*
* To the fullest extent allowed by law, Microchip and its licensors
* liability shall not exceed the amount of fees, if any, that you
* have paid directly to Microchip to use this software.
*
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*
* Author        Date        Comment
*************************************************************************
* E. Schlunder  2009/04/29  Import "Essential Device Characteristics"
*                           from XML PIC files and convert into SQL
*                           scripts.
************************************************************************/

#include <QXmlStreamReader>
#include <QFile>

#include <QTime>
#include <QTextStream>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>

#include "DeviceWindow.h"
#include "ui_DeviceWindow.h"
#include "DeviceXmlLoader.h"

#define APPLICATION "Device Database"
DeviceWindow::DeviceWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::DeviceWindowClass)
{
    ui->setupUi(this);

    QSettings settings;
    settings.beginGroup("DeviceDatabase");
    resize(settings.value("size", QSize(604, 548)).toSize());
    move(settings.value("pos", QPoint(100, 100)).toPoint());
    fileName = settings.value("fileName", "../PIC18F8722.PIC").toString();
    settings.endGroup();
}

DeviceWindow::~DeviceWindow()
{
    QSettings settings;

    settings.beginGroup("DeviceDatabase");
    settings.setValue("size", size());
    settings.setValue("pos", pos());
    settings.setValue("fileName", fileName);
    settings.endGroup();

    delete ui;
}

#define COMMENT_COLUMN 23
void DeviceWindow::on_action_Open_PIC_Definition_triggered()
{
    QString msg, decimal;
    QTextStream stream(&msg);
    QString incMsg;
    QTextStream incStream(&incMsg);
    int i, j, k, x;
    int width;

    stream.setNumberFlags(QTextStream::UppercaseDigits);
    incStream.setNumberFlags(QTextStream::UppercaseDigits);

    QStringList files = QFileDialog::getOpenFileNames(
                             this,
                             "Select one or more PIC definition files to open",
                             fileName,
                             "PIC definitions (*.pic);;XML files (*.xml);;All files (*.*)");
    if(files.isEmpty())
    {
        return;
    }

    if(ui->SqlText->toPlainText().size() == 0)
    {
        msg.clear();
        stream << "drop table DEVICES;\n";
        stream << "drop table CONFIGWORDS;\n";
        stream << "drop table CONFIGSETTINGS;\n";
        stream << "drop table CONFIGFIELDS;\n";
        stream << "\n";
        stream << "create table DEVICES (\n";
        stream << "     DEVICEROWID integer primary key autoincrement,\n";
        stream << "     DEVICEID integer not null,\n";
        stream << "     FAMILYID integer not null,\n";
        stream << "     PARTNAME text not null, \n";
        stream << "     BYTESPERWORDFLASH integer,\n";
        stream << "     WRITEFLASHBLOCKSIZE integer not null,\n";
        stream << "     ERASEFLASHBLOCKSIZE integer not null,\n";
        stream << "     STARTFLASH integer,\n";
        stream << "     ENDFLASH integer,\n";
        stream << "     STARTEE integer, \n";
        stream << "     ENDEE integer, \n";
        stream << "     STARTUSER integer,\n";
        stream << "     ENDUSER integer, \n";
        stream << "     STARTCONFIG integer,\n";
        stream << "     ENDCONFIG integer,\n";
        stream << "     STARTDEVICEID integer,\n";
        stream << "     ENDDEVICEID integer,\n";
        stream << "     DEVICEIDMASK integer,\n";
        stream << "     STARTGPR integer,\n";
        stream << "     ENDGPR integer\n";
        stream << ");\n";
        stream << "\n";
        stream << "create table CONFIGWORDS (\n";
        stream << "     ROWID integer primary key autoincrement,\n";
        stream << "     DEVICEROWID integer not null,\n";
        stream << "     CONFIGNAME text not null,\n";
        stream << "     ADDRESS integer not null,\n";
        stream << "     DEFAULTVALUE integer,\n";
        stream << "     IMPLEMENTEDBITS integer\n";
        stream << ");\n";
        stream << "\n";
        stream << "create table CONFIGFIELDS (\n";
        stream << "     ROWID integer primary key autoincrement,\n";
        stream << "     CONFIGWORDID integer not null,\n";
        stream << "     DEVICEROWID integer not null,\n";
        stream << "     FIELDCNAME text not null,\n";
        stream << "     DESCRIPTION text not null\n";
        stream << ");\n";
        stream << "\n";
        stream << "create table CONFIGSETTINGS (\n";
        stream << "     CONFIGFIELDID integer not null,\n";
        stream << "     DEVICEROWID integer not null,\n";
        stream << "     SETTINGCNAME text not null,\n";
        stream << "     DESCRIPTION text not null,\n";
        stream << "     BITMASK integer not null,\n";
        stream << "     BITVALUE integer not null\n";
        stream << ");\n";

        ui->SqlText->append(msg);
    }

    ui->SqlText->append("pragma read_uncommitted = 1;\n");
    ui->SqlText->append("begin transaction;\n");

    QApplication::setOverrideCursor(Qt::BusyCursor);

    Device device;
    DeviceXmlLoader loader;
    loader.device = &device;
    for(i = 0; i < files.count(); i++)
    {
        device.setUnknown();
        device.configWords.clear();

        fileName = files[i];
        QFileInfo fi(fileName);
        QString name = fi.fileName();

        msg.clear();
        stream.setIntegerBase(10);
        stream << (i+1) << ". " << name << " - " << APPLICATION;
        setWindowTitle(msg);
        loader.read(fileName);

        msg.clear();
        if(device.eraseBlockSizeFLASH == 0)
        {
            stream << "-- " << device.name << " does not support self-programming.";
            ui->SqlText->append(msg);

            msg.clear();
            stream << "; " << device.name << " does not support self-programming.";
            ui->IncText->append(msg);

            continue;
        }

        if(device.family == Device::PIC18 || device.family == Device::PIC16 || device.family == Device::PIC17)
        {
            decimal = ".";
        }
        else
        {
            decimal = "";
        }

        // ----------------------------------------------------------------------------------------------
        stream << "insert into DEVICES values (" << endl;
        stream << "     null,\n";

        width = msg.length();
        stream << "     " << device.id << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Device ID");

        stream << "     " << device.family << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Family ID");

        stream << "     '" << device.name << "',";
        comment(stream, width, COMMENT_COLUMN, "-- Part Name");

        stream << "     " << device.bytesPerWordFLASH << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Bytes Per Word (FLASH)");

        stream << "     " << device.writeBlockSizeFLASH << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Write FLASH Block Size");

        stream << "     " << device.eraseBlockSizeFLASH << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Erase FLASH Page Size");

        stream.setIntegerBase(10);
        stream << "     " << device.startFLASH << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of FLASH");

        stream << "     " << device.endFLASH << ",";
        comment(stream, width, COMMENT_COLUMN, "-- End of FLASH");

        stream << "     " << device.startEEPROM << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of EEPROM");

        stream << "     " << device.endEEPROM << ",";
        comment(stream, width, COMMENT_COLUMN, "-- End of EEPROM");

        stream << "     " << device.startUser << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of User ID");

        stream << "     " << device.endUser << ",";
        comment(stream, width, COMMENT_COLUMN, "-- End of User ID");

        stream << "     " << device.startConfig << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of Config Bits");
        stream << "     " << device.endConfig << ",";
        comment(stream, width, COMMENT_COLUMN, "-- End of Config Bits");

        stream << "     " << device.startDeviceId << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of Device Id");
        stream << "     " << device.endDeviceId << ",";
        comment(stream, width, COMMENT_COLUMN, "-- End of Device Id");
        stream << "     " << device.deviceIdMask << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Device Id Mask");

        stream << "     " << device.startGPR << ",";
        comment(stream, width, COMMENT_COLUMN, "-- Start of GPR");

        stream << "     " << device.endGPR;
        comment(stream, width, COMMENT_COLUMN, "-- End of GPR");

        stream << ");" << endl;

        for(j = 0; j < device.configWords.count(); j++)
        {
            Device::ConfigWord& config = device.configWords[j];
            stream << "insert into CONFIGWORDS values (" << endl;
            stream.setIntegerBase(10);
            stream << "     NULL,\n";
            stream << "     (select max(ROWID) from DEVICES),\n";
            width = msg.length();
            stream << "     '" << config.name << "',";
            comment(stream, width, COMMENT_COLUMN, "-- Config Name");
            stream << "     " << config.address << ",";
            comment(stream, width, COMMENT_COLUMN, "-- Address");
            stream << "     " << config.defaultValue << ",";
            comment(stream, width, COMMENT_COLUMN, "-- Default Value");
            stream << "     " << config.implementedBits;
            comment(stream, width, COMMENT_COLUMN, "-- Implemented Bits");
            stream << ");" << endl;

            for(k = 0; k < config.fields.count(); k++)
            {
                Device::ConfigField& field = config.fields[k];
                if(field.hidden)
                {
                    continue;
                }

                stream << "insert into CONFIGFIELDS values (" << endl;
                stream.setIntegerBase(10);
                stream << "     NULL,\n";
                stream << "     (select max(ROWID) from CONFIGWORDS),\n";
                stream << "     (select max(ROWID) from DEVICES),\n";
                width = msg.length();
                stream << "     '" << field.cname << "',";
                comment(stream, width, COMMENT_COLUMN, "-- C Name");
                stream << "     '" << escape(field.description) << "'";
                comment(stream, width, COMMENT_COLUMN, "-- Description");
                stream << ");\n" << endl;

                for(x = 0; x < field.settings.count(); x++)
                {
                    Device::ConfigFieldSetting& setting = field.settings[x];
                    stream << "insert into CONFIGSETTINGS values (" << endl;
                    stream.setIntegerBase(10);
                    stream << "     (select max(ROWID) from CONFIGFIELDS),\n";
                    stream << "     (select max(ROWID) from DEVICES),\n";
                    width = msg.length();
                    stream << "     '" << setting.cname << "',";
                    comment(stream, width, COMMENT_COLUMN, "-- C Name");
                    stream << "     '" << escape(setting.description) << "',";
                    comment(stream, width, COMMENT_COLUMN, "-- Description");
                    stream.setIntegerBase(10);
                    stream << "     " << setting.bitMask << ",";
                    comment(stream, width, COMMENT_COLUMN, "-- Bit Mask");
                    stream << "     " << setting.bitValue << "";
                    comment(stream, width, COMMENT_COLUMN, "-- Bit Value");
                    stream << ");\n" << endl;
                }
            }
        }
        ui->SqlText->append(msg);

        // ----------------------------------------------------------------------------------------------
        msg.clear();
        incMsg.clear();
        if(device.family == Device::PIC24)
        {
            stream << "#ifdef __" << device.name << "__" << endl;
            incStream << ".ifdef __" << device.name.mid(3) << endl;
        }
        else if(device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
        {
            stream << "#ifdef __" << device.name << "__" << endl;
            incStream << ".ifdef __" << device.name.mid(5) << endl;
        }
        else if(device.family == Device::PIC32)
        {
            stream << "#ifdef __" << device.name.mid(3) << "__" << endl;
        }
        else
        {
            stream << "#ifdef __" << device.name.mid(3) << endl;
        }

        if(device.family == Device::PIC16)
        {
            // there doesn't appear to be a generic 16F include, so let's make devices.inc do it.
            stream << "     #include p" << device.name.mid(3) << ".inc\n" << endl;
        }
        stream.setIntegerBase(10);
        incStream.setIntegerBase(10);
        if(device.hasConfigAsFlash())
        {
            stream << "     #define CONFIG_AS_FLASH" << endl;
            incStream << "     .equ CONFIG_AS_FLASH, 1" << endl;
        }

        stream << "     #define DEVICEID                " << decimal << device.id << endl;
        stream << "     #define WRITE_FLASH_BLOCKSIZE   " << decimal << device.writeBlockSizeFLASH << endl;
        stream << "     #define ERASE_FLASH_BLOCKSIZE   " << decimal << device.eraseBlockSizeFLASH << endl;

        incStream << "     .equ DEVICEID, " << device.id << endl;
        incStream << "     .equ WRITE_FLASH_BLOCKSIZE, " << device.writeBlockSizeFLASH << endl;
        incStream << "     .equ ERASE_FLASH_BLOCKSIZE, " << device.eraseBlockSizeFLASH << endl;

        stream.setIntegerBase(16);
        incStream.setIntegerBase(16);
        stream << "     #define END_FLASH               0x" << device.endFLASH << endl;
        incStream << "     .equ END_FLASH, 0x" << device.endFLASH << endl;
        if(device.family == Device::PIC24 || device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
        {
            stream << "     #define START_GPR               0x" << device.startGPR << endl;
            incStream << "     .equ START_GPR, 0x" << device.startGPR << endl;
        }
        stream << "     #define END_GPR                 0x" << device.endGPR << endl;
        incStream << "     .equ END_GPR, 0x" << device.endGPR << endl;

        if(device.family == Device::PIC24 || device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
        {
            stream << "     #define START_CONFIG            0x" << device.startConfig << endl;
            incStream << "     .equ START_CONFIG, 0x" << device.startConfig << endl;
            stream << "     #define END_CONFIG              0x" << device.endConfig << endl;
            incStream << "     .equ END_CONFIG, 0x" << device.endConfig << endl;
        }

        stream << "#endif" << endl;
        incStream << ".endif" << endl;
        if(device.family == Device::PIC24 || device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
        {
            ui->HText->append(msg);
            ui->IncText->append(incMsg);
        }
        else if(device.family == Device::PIC32)
        {
            ui->HText->append(msg);
        }
        else
        {
            ui->IncText->append(msg);
        }

        // ----------------------------------------------------------------------------------------------
        if(device.family == Device::PIC24 || device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
        {
            GenerateVectors(device, false);
            GenerateVectors(device, true);      // alternative vectors
        }

        // ----------------------------------------------------------------------------------------------
        msg.clear();
        QApplication::processEvents();
    }

    ui->SqlText->append("commit transaction;\n");

    QApplication::restoreOverrideCursor();
}

void DeviceWindow::GenerateVectors(Device& device, bool alternate)
{
    QString msg;
    QTextStream stream(&msg);
    unsigned int vectors = 0;

    msg.clear();
    if(device.family == Device::dsPIC30 || device.family == Device::dsPIC33)
    {
        stream << ".ifdef __" << device.name.mid(5) << endl;
    }
    else
    {
        stream << ".ifdef __" << device.name.mid(3) << endl;
    }

    if(alternate)
    {
        stream << "   .macro AltVectors" << endl;
    }
    else
    {
        stream << "   .macro Vectors" << endl;
    }

    foreach(Device::Trap trap, device.Traps)
    {
        vectors++;
        if(alternate)
        {
            stream << "     .global __Alt" << trap.name << endl;
            stream << "     __Alt" << trap.name << ":" << endl;
        }
        else
        {
            stream << "     .global __" << trap.name << endl;
            stream << "     __" << trap.name << ":" << endl;
        }
        stream << "          .space 2*2" << endl;
    }

    foreach(Device::IRQ irq, device.IRQs)
    {
        vectors++;
        if(alternate)
        {
            stream << "     .global __Alt" << irq.name << endl;
            stream << "     __Alt" << irq.name << ":" << endl;
        }
        else
        {
            stream << "     .global __" << irq.name << endl;
            stream << "     __" << irq.name << ":" << endl;
        }
        stream << "          .space 2*2" << endl;
    }

    // Now add the reserved interrupts
    if(alternate == false)
    {
        stream << "     .global __DefaultInterrupt" << endl;
        stream << "     __DefaultInterrupt:" << endl;
    }
    unsigned int interruptNumber = 0;
    foreach(Device::IRQ irq, device.IRQs)
    {
        while(interruptNumber < irq.number)
        {
            if(alternate)
            {
                stream << "     .global __AltInterrupt" << interruptNumber << endl;
                stream << "     __AltInterrupt" << interruptNumber << ":" << endl;
            }
            else
            {
                stream << "     .global __Interrupt" << interruptNumber << endl;
                stream << "     __Interrupt" << interruptNumber << ":" << endl;
            }

            interruptNumber++;
        }

        interruptNumber++;
    }

    while(interruptNumber < 118)
    {
        if(alternate)
        {
            stream << "     .global __AltInterrupt" << interruptNumber << endl;
            stream << "     __AltInterrupt" << interruptNumber << ":" << endl;
        }
        else
        {
            stream << "     .global __Interrupt" << interruptNumber << endl;
            stream << "     __Interrupt" << interruptNumber << ":" << endl;
        }

        interruptNumber++;
    }

    stream << "          .space 2*2" << endl;   // default vector for all unexpected interrupts
    vectors++;

    stream << "   .endm" << endl << endl;


    if(alternate)
    {
        stream << "     .equ ALTVECTORS, " << vectors << endl;
    }
    else
    {
        stream << "     .equ VECTORS, " << vectors << endl;
    }

    stream << ".endif" << endl;

    if(alternate)
    {
        ui->altVectorsText->append(msg);
    }
    else
    {
        ui->vectorsText->append(msg);
    }
}

void DeviceWindow::comment(QTextStream& stream, int& width, int column, QString newComment)
{
    int currentWidth = stream.string()->length();
    int pad = column - (currentWidth - width);
    while(pad > 0)
    {
        stream << " ";
        pad--;
    }
    stream << newComment;
    stream << endl;

    width = stream.string()->length();
}

void DeviceWindow::on_actionExit_triggered()
{
    QApplication::quit();
}

void DeviceWindow::on_actionClear_All_triggered()
{
    ui->SqlText->clear();
    ui->IncText->clear();
    ui->HText->clear();
    ui->vectorsText->clear();
    ui->altVectorsText->clear();
}

QString DeviceWindow::escape(const QString& text)
{
    QString result(text);

    result.replace("'", "''");
    return result;
}
