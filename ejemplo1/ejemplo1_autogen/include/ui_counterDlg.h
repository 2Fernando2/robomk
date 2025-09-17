/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QPushButton *button;
    QLCDNumber *lcdNumber;
    QSlider *horizontalSlider;
    QLCDNumber *lcdNumber_2;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName("Counter");
        Counter->resize(400, 300);
        button = new QPushButton(Counter);
        button->setObjectName("button");
        button->setGeometry(QRect(80, 210, 251, 71));
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName("lcdNumber");
        lcdNumber->setGeometry(QRect(50, 40, 301, 91));
        horizontalSlider = new QSlider(Counter);
        horizontalSlider->setObjectName("horizontalSlider");
        horizontalSlider->setGeometry(QRect(40, 150, 321, 20));
        horizontalSlider->setMinimum(20);
        horizontalSlider->setMaximum(1000);
        horizontalSlider->setOrientation(Qt::Horizontal);
        lcdNumber_2 = new QLCDNumber(Counter);
        lcdNumber_2->setObjectName("lcdNumber_2");
        lcdNumber_2->setGeometry(QRect(170, 180, 64, 23));
        lcdNumber_2->setProperty("intValue", QVariant(20));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QCoreApplication::translate("Counter", "Counter", nullptr));
        button->setText(QCoreApplication::translate("Counter", "STOP", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
