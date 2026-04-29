/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QComboBox *comboBoxCtrlProg;
    QLabel *label;
    QPushButton *pushStartStop;
    QLabel *label_2;
    QComboBox *comboBoxEni;
    QSplitter *splitter;
    QListWidget *listWidget;
    QTabWidget *tabWidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1036, 529);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        comboBoxCtrlProg = new QComboBox(centralwidget);
        comboBoxCtrlProg->addItem(QString());
        comboBoxCtrlProg->addItem(QString());
        comboBoxCtrlProg->setObjectName(QString::fromUtf8("comboBoxCtrlProg"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(comboBoxCtrlProg->sizePolicy().hasHeightForWidth());
        comboBoxCtrlProg->setSizePolicy(sizePolicy);
        comboBoxCtrlProg->setEditable(true);

        gridLayout->addWidget(comboBoxCtrlProg, 0, 2, 1, 1);

        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMaximumSize(QSize(16777215, 16777215));

        gridLayout->addWidget(label, 0, 1, 1, 1);

        pushStartStop = new QPushButton(centralwidget);
        pushStartStop->setObjectName(QString::fromUtf8("pushStartStop"));

        gridLayout->addWidget(pushStartStop, 0, 0, 1, 1);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 1, 1, 1);

        comboBoxEni = new QComboBox(centralwidget);
        comboBoxEni->addItem(QString());
        comboBoxEni->setObjectName(QString::fromUtf8("comboBoxEni"));
        comboBoxEni->setEditable(true);

        gridLayout->addWidget(comboBoxEni, 1, 2, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        splitter = new QSplitter(centralwidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(splitter->sizePolicy().hasHeightForWidth());
        splitter->setSizePolicy(sizePolicy1);
        splitter->setOrientation(Qt::Horizontal);
        listWidget = new QListWidget(splitter);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        splitter->addWidget(listWidget);
        tabWidget = new QTabWidget(splitter);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(1);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy2);
        splitter->addWidget(tabWidget);

        gridLayout_2->addWidget(splitter, 1, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1036, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "ServoCtrlWindow", nullptr));
        comboBoxCtrlProg->setItemText(0, QCoreApplication::translate("MainWindow", "/home/niic/ethercat-cpp/build/bin/ethercat-c-exipc-demo", nullptr));
        comboBoxCtrlProg->setItemText(1, QCoreApplication::translate("MainWindow", "/home/niic/ethercat-cpp/build/bin/ethercat-cxx-exipc-demo", nullptr));

        label->setText(QCoreApplication::translate("MainWindow", "\344\274\272\346\234\215\346\216\247\345\210\266\347\250\213\345\272\217\357\274\232", nullptr));
        pushStartStop->setText(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "ENI\351\205\215\347\275\256\346\226\207\344\273\266:", nullptr));
        comboBoxEni->setItemText(0, QCoreApplication::translate("MainWindow", "/home/niic/NIIC_ENI_M0_EP3.xml", nullptr));

    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
