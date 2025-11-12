/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_17;
    QFrame *frame;
    QFrame *frame_room;
    QHBoxLayout *horizontalLayout_9;
    QHBoxLayout *horizontalLayout_18;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_7;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLCDNumber *lcdNumber_x;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLCDNumber *lcdNumber_y;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QLCDNumber *lcdNumber_angle;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLCDNumber *lcdNumber_adv;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLCDNumber *lcdNumber_rot;
    QHBoxLayout *horizontalLayout;
    QLabel *label_state_name;
    QLabel *label_state;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_stop;
    QFrame *frame_plot_error;
    QLabel *label_img;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName("guiDlg");
        guiDlg->resize(1224, 773);
        verticalLayout_5 = new QVBoxLayout(guiDlg);
        verticalLayout_5->setObjectName("verticalLayout_5");
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName("horizontalLayout_17");
        frame = new QFrame(guiDlg);
        frame->setObjectName("frame");
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(500, 300));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        horizontalLayout_17->addWidget(frame);

        frame_room = new QFrame(guiDlg);
        frame_room->setObjectName("frame_room");
        sizePolicy.setHeightForWidth(frame_room->sizePolicy().hasHeightForWidth());
        frame_room->setSizePolicy(sizePolicy);
        frame_room->setMinimumSize(QSize(500, 300));
        frame_room->setFrameShape(QFrame::StyledPanel);
        frame_room->setFrameShadow(QFrame::Raised);

        horizontalLayout_17->addWidget(frame_room);


        verticalLayout_2->addLayout(horizontalLayout_17);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName("horizontalLayout_9");
        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setObjectName("horizontalLayout_18");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        label = new QLabel(guiDlg);
        label->setObjectName("label");

        horizontalLayout_2->addWidget(label);

        lcdNumber_x = new QLCDNumber(guiDlg);
        lcdNumber_x->setObjectName("lcdNumber_x");
        QFont font;
        font.setBold(true);
        lcdNumber_x->setFont(font);

        horizontalLayout_2->addWidget(lcdNumber_x);


        horizontalLayout_7->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        label_2 = new QLabel(guiDlg);
        label_2->setObjectName("label_2");

        horizontalLayout_3->addWidget(label_2);

        lcdNumber_y = new QLCDNumber(guiDlg);
        lcdNumber_y->setObjectName("lcdNumber_y");
        lcdNumber_y->setFont(font);

        horizontalLayout_3->addWidget(lcdNumber_y);


        horizontalLayout_7->addLayout(horizontalLayout_3);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        label_3 = new QLabel(guiDlg);
        label_3->setObjectName("label_3");

        horizontalLayout_6->addWidget(label_3);

        lcdNumber_angle = new QLCDNumber(guiDlg);
        lcdNumber_angle->setObjectName("lcdNumber_angle");
        lcdNumber_angle->setFont(font);

        horizontalLayout_6->addWidget(lcdNumber_angle);


        horizontalLayout_7->addLayout(horizontalLayout_6);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        label_4 = new QLabel(guiDlg);
        label_4->setObjectName("label_4");

        horizontalLayout_4->addWidget(label_4);

        lcdNumber_adv = new QLCDNumber(guiDlg);
        lcdNumber_adv->setObjectName("lcdNumber_adv");

        horizontalLayout_4->addWidget(lcdNumber_adv);


        horizontalLayout_8->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        label_5 = new QLabel(guiDlg);
        label_5->setObjectName("label_5");

        horizontalLayout_5->addWidget(label_5);

        lcdNumber_rot = new QLCDNumber(guiDlg);
        lcdNumber_rot->setObjectName("lcdNumber_rot");

        horizontalLayout_5->addWidget(lcdNumber_rot);


        horizontalLayout_8->addLayout(horizontalLayout_5);


        verticalLayout->addLayout(horizontalLayout_8);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        label_state_name = new QLabel(guiDlg);
        label_state_name->setObjectName("label_state_name");

        horizontalLayout->addWidget(label_state_name);

        label_state = new QLabel(guiDlg);
        label_state->setObjectName("label_state");

        horizontalLayout->addWidget(label_state);


        verticalLayout->addLayout(horizontalLayout);


        horizontalLayout_18->addLayout(verticalLayout);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_18->addItem(horizontalSpacer);

        pushButton_stop = new QPushButton(guiDlg);
        pushButton_stop->setObjectName("pushButton_stop");
        pushButton_stop->setCheckable(true);

        horizontalLayout_18->addWidget(pushButton_stop);


        horizontalLayout_9->addLayout(horizontalLayout_18);

        frame_plot_error = new QFrame(guiDlg);
        frame_plot_error->setObjectName("frame_plot_error");
        frame_plot_error->setMinimumSize(QSize(500, 0));
        frame_plot_error->setFrameShape(QFrame::StyledPanel);
        frame_plot_error->setFrameShadow(QFrame::Raised);

        horizontalLayout_9->addWidget(frame_plot_error);


        verticalLayout_2->addLayout(horizontalLayout_9);

        label_img = new QLabel(guiDlg);
        label_img->setObjectName("label_img");
        label_img->setMinimumSize(QSize(300, 100));

        verticalLayout_2->addWidget(label_img);


        verticalLayout_5->addLayout(verticalLayout_2);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QCoreApplication::translate("guiDlg", "Localiser", nullptr));
        label->setText(QCoreApplication::translate("guiDlg", "X", nullptr));
        label_2->setText(QCoreApplication::translate("guiDlg", "Y", nullptr));
        label_3->setText(QCoreApplication::translate("guiDlg", "angle", nullptr));
        label_4->setText(QCoreApplication::translate("guiDlg", "adv", nullptr));
        label_5->setText(QCoreApplication::translate("guiDlg", "rot", nullptr));
        label_state_name->setText(QCoreApplication::translate("guiDlg", "state:", nullptr));
        label_state->setText(QCoreApplication::translate("guiDlg", "TextLabel", nullptr));
        pushButton_stop->setText(QCoreApplication::translate("guiDlg", "Stop", nullptr));
        label_img->setText(QCoreApplication::translate("guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
