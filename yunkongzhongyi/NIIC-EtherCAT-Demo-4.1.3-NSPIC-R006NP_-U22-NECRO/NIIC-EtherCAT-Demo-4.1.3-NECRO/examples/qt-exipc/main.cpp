#include "mainwindow.h"

#include <QApplication>
#include <QLocale>
#include <QDebug>
#include <QFontDatabase>
#include <QTextCodec>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    
    int fontId = QFontDatabase::addApplicationFont("/usr/lib64/fonts/wenquanyi.ttf");
    QStringList fontIDs = QFontDatabase::applicationFontFamilies(fontId);
    if (!fontIDs.isEmpty()) {
        QFont font(fontIDs.first());
        QApplication::setFont(font);
    }
    else {
        qDebug()<<"Failed to load font.";
    }

    w.show();
    return a.exec();
}
