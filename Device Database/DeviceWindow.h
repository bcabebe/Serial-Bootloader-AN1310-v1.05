/************************************************************************
* Copyright (c) 2009,  Microchip Technology Inc.
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
*/

#ifndef DEVICEWINDOW_H
#define DEVICEWINDOW_H

#include <QtGui/QMainWindow>
#include <QString>
#include <QTextStream>
#include "../PC Software/Bootload/Device.h"

namespace Ui
{
    class DeviceWindowClass;
}

class DeviceWindow : public QMainWindow
{
    Q_OBJECT

public:
    DeviceWindow(QWidget *parent = 0);
    ~DeviceWindow();

protected:
    QString fileName;

private:
    Ui::DeviceWindowClass *ui;
    void comment(QTextStream& stream, int& width, int column, QString newComment);
    QString escape(const QString& text);
    void GenerateVectors(Device& device, bool alternate = false);

private slots:
    void on_actionClear_All_triggered();
    void on_actionExit_triggered();
    void on_action_Open_PIC_Definition_triggered();
};

#endif // DEVICEWINDOW_H
