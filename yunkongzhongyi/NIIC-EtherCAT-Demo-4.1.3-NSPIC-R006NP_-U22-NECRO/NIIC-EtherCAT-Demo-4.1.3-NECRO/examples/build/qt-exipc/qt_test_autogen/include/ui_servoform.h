/********************************************************************************
** Form generated from reading UI file 'servoform.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SERVOFORM_H
#define UI_SERVOFORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ServoForm
{
public:
    QGridLayout *gridLayout_5;
    QSplitter *splitter;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QLabel *label_6;
    QLineEdit *lineEditCurPos;
    QLabel *label_9;
    QLabel *label;
    QLineEdit *lineEditCurError;
    QLabel *label_5;
    QLineEdit *lineEditCurVel;
    QLabel *label_4;
    QLabel *label_2;
    QLineEdit *lineEditCurMode;
    QLineEdit *lineEditCurStatus;
    QLabel *label_10;
    QLineEdit *lineEditVID;
    QLineEdit *lineEditPID;
    QSpacerItem *verticalSpacer_2;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout_3;
    QSlider *hsPos;
    QLabel *label_8;
    QLabel *hsPosDisp;
    QGridLayout *gridLayout_8;
    QPushButton *testStartStop;
    QGridLayout *gridLayout_4;
    QPushButton *pushButtonStartStop;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *ServoForm)
    {
        if (ServoForm->objectName().isEmpty())
            ServoForm->setObjectName(QString::fromUtf8("ServoForm"));
        ServoForm->resize(737, 434);
        gridLayout_5 = new QGridLayout(ServoForm);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        splitter = new QSplitter(ServoForm);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        groupBox = new QGroupBox(splitter);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 6, 0, 1, 1);

        lineEditCurPos = new QLineEdit(groupBox);
        lineEditCurPos->setObjectName(QString::fromUtf8("lineEditCurPos"));
        lineEditCurPos->setReadOnly(true);

        gridLayout->addWidget(lineEditCurPos, 2, 1, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout->addWidget(label_9, 1, 0, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        lineEditCurError = new QLineEdit(groupBox);
        lineEditCurError->setObjectName(QString::fromUtf8("lineEditCurError"));
        lineEditCurError->setReadOnly(true);

        gridLayout->addWidget(lineEditCurError, 6, 1, 1, 1);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 5, 0, 1, 1);

        lineEditCurVel = new QLineEdit(groupBox);
        lineEditCurVel->setObjectName(QString::fromUtf8("lineEditCurVel"));
        lineEditCurVel->setReadOnly(true);

        gridLayout->addWidget(lineEditCurVel, 3, 1, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 4, 0, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 3, 0, 1, 1);

        lineEditCurMode = new QLineEdit(groupBox);
        lineEditCurMode->setObjectName(QString::fromUtf8("lineEditCurMode"));
        lineEditCurMode->setReadOnly(true);

        gridLayout->addWidget(lineEditCurMode, 4, 1, 1, 1);

        lineEditCurStatus = new QLineEdit(groupBox);
        lineEditCurStatus->setObjectName(QString::fromUtf8("lineEditCurStatus"));
        lineEditCurStatus->setReadOnly(true);

        gridLayout->addWidget(lineEditCurStatus, 5, 1, 1, 1);

        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 0, 0, 1, 1);

        lineEditVID = new QLineEdit(groupBox);
        lineEditVID->setObjectName(QString::fromUtf8("lineEditVID"));
        lineEditVID->setReadOnly(true);

        gridLayout->addWidget(lineEditVID, 0, 1, 1, 1);

        lineEditPID = new QLineEdit(groupBox);
        lineEditPID->setObjectName(QString::fromUtf8("lineEditPID"));
        lineEditPID->setReadOnly(true);

        gridLayout->addWidget(lineEditPID, 1, 1, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 181, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_2->addItem(verticalSpacer_2, 1, 0, 1, 1);

        splitter->addWidget(groupBox);
        groupBox_2 = new QGroupBox(splitter);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout = new QVBoxLayout(groupBox_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        hsPos = new QSlider(groupBox_2);
        hsPos->setObjectName(QString::fromUtf8("hsPos"));
        hsPos->setMinimum(-99);
        hsPos->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(hsPos, 1, 1, 1, 1);

        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_3->addWidget(label_8, 1, 0, 1, 1);

        hsPosDisp = new QLabel(groupBox_2);
        hsPosDisp->setObjectName(QString::fromUtf8("hsPosDisp"));

        gridLayout_3->addWidget(hsPosDisp, 1, 2, 1, 1);


        verticalLayout->addLayout(gridLayout_3);

        gridLayout_8 = new QGridLayout();
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        testStartStop = new QPushButton(groupBox_2);
        testStartStop->setObjectName(QString::fromUtf8("testStartStop"));

        gridLayout_8->addWidget(testStartStop, 1, 0, 1, 1);


        verticalLayout->addLayout(gridLayout_8);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        pushButtonStartStop = new QPushButton(groupBox_2);
        pushButtonStartStop->setObjectName(QString::fromUtf8("pushButtonStartStop"));

        gridLayout_4->addWidget(pushButtonStartStop, 0, 0, 1, 1);


        verticalLayout->addLayout(gridLayout_4);

        verticalSpacer = new QSpacerItem(20, 272, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        splitter->addWidget(groupBox_2);

        gridLayout_5->addWidget(splitter, 0, 0, 1, 1);


        retranslateUi(ServoForm);

        QMetaObject::connectSlotsByName(ServoForm);
    } // setupUi

    void retranslateUi(QWidget *ServoForm)
    {
        ServoForm->setWindowTitle(QCoreApplication::translate("ServoForm", "Form", nullptr));
        groupBox->setTitle(QCoreApplication::translate("ServoForm", "\347\212\266\346\200\201\346\230\276\347\244\272", nullptr));
        label_6->setText(QCoreApplication::translate("ServoForm", "\351\224\231\350\257\257\347\240\201:", nullptr));
        label_9->setText(QCoreApplication::translate("ServoForm", "\344\272\247\345\223\201ID:", nullptr));
        label->setText(QCoreApplication::translate("ServoForm", "\345\275\223\345\211\215\344\275\215\347\275\256:", nullptr));
        label_5->setText(QCoreApplication::translate("ServoForm", "\347\212\266\346\200\201\345\255\227:", nullptr));
        label_4->setText(QCoreApplication::translate("ServoForm", "\345\275\223\345\211\215\346\250\241\345\274\217:", nullptr));
        label_2->setText(QCoreApplication::translate("ServoForm", "\345\275\223\345\211\215\351\200\237\345\272\246:", nullptr));
        label_10->setText(QCoreApplication::translate("ServoForm", "\345\216\202\345\225\206ID:", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("ServoForm", "\347\224\250\346\210\267\346\216\247\345\210\266", nullptr));
        label_8->setText(QCoreApplication::translate("ServoForm", "\350\256\276\347\275\256\344\275\215\347\275\256:", nullptr));
        hsPosDisp->setText(QCoreApplication::translate("ServoForm", "0 r", nullptr));
        testStartStop->setText(QCoreApplication::translate("ServoForm", "\346\265\213\350\257\225\345\274\200\345\247\213", nullptr));
        pushButtonStartStop->setText(QCoreApplication::translate("ServoForm", "\344\270\212\347\224\265", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ServoForm: public Ui_ServoForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SERVOFORM_H
