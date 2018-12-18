/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *winProc1;
    QLabel *winFinalRes;
    QLabel *winProc2;
    QLabel *winProc3;
    QPushButton *btnLoadVideo;
    QPushButton *btnInitProc;
    QLabel *lblMsg;
    QGroupBox *groupBox;
    QRadioButton *rbCircle;
    QRadioButton *rbRing;
    QLineEdit *numRows;
    QLabel *label;
    QLabel *label_2;
    QLineEdit *numCols;
    QLabel *winProc5;
    QLabel *subtitle3;
    QLabel *subtitle2;
    QLabel *subtitle1;
    QLabel *subtitle5;
    QLabel *mainTitle;
    QLabel *label_5;
    QLabel *label_6;
    QGroupBox *groupBox_2;
    QLineEdit *leTFrames;
    QLabel *label_3;
    QLabel *label_4;
    QLineEdit *leAnalized;
    QLineEdit *leAccu;
    QLineEdit *timeExec;
    QLabel *label_7;
    QLabel *label_8;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1510, 687);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        winProc1 = new QLabel(centralWidget);
        winProc1->setObjectName(QStringLiteral("winProc1"));
        winProc1->setGeometry(QRect(300, 100, 261, 231));
        winProc1->setStyleSheet(QStringLiteral("border: 1px solid gray;"));
        winProc1->setScaledContents(true);
        winFinalRes = new QLabel(centralWidget);
        winFinalRes->setObjectName(QStringLiteral("winFinalRes"));
        winFinalRes->setGeometry(QRect(870, 100, 611, 391));
        winFinalRes->setStyleSheet(QStringLiteral("border: 1px solid gray;"));
        winFinalRes->setScaledContents(true);
        winProc2 = new QLabel(centralWidget);
        winProc2->setObjectName(QStringLiteral("winProc2"));
        winProc2->setGeometry(QRect(580, 100, 261, 231));
        winProc2->setStyleSheet(QStringLiteral("border: 1px solid gray;"));
        winProc2->setScaledContents(true);
        winProc3 = new QLabel(centralWidget);
        winProc3->setObjectName(QStringLiteral("winProc3"));
        winProc3->setGeometry(QRect(300, 380, 261, 231));
        winProc3->setStyleSheet(QStringLiteral("border: 1px solid gray;"));
        winProc3->setScaledContents(true);
        btnLoadVideo = new QPushButton(centralWidget);
        btnLoadVideo->setObjectName(QStringLiteral("btnLoadVideo"));
        btnLoadVideo->setGeometry(QRect(40, 390, 99, 27));
        btnInitProc = new QPushButton(centralWidget);
        btnInitProc->setObjectName(QStringLiteral("btnInitProc"));
        btnInitProc->setEnabled(false);
        btnInitProc->setGeometry(QRect(150, 390, 99, 27));
        lblMsg = new QLabel(centralWidget);
        lblMsg->setObjectName(QStringLiteral("lblMsg"));
        lblMsg->setGeometry(QRect(1010, 790, 351, 31));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(30, 150, 201, 191));
        QFont font;
        font.setPointSize(10);
        groupBox->setFont(font);
        groupBox->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        rbCircle = new QRadioButton(groupBox);
        rbCircle->setObjectName(QStringLiteral("rbCircle"));
        rbCircle->setGeometry(QRect(10, 30, 121, 20));
        rbCircle->setFont(font);
        rbCircle->setChecked(true);
        rbRing = new QRadioButton(groupBox);
        rbRing->setObjectName(QStringLiteral("rbRing"));
        rbRing->setGeometry(QRect(10, 50, 121, 20));
        rbRing->setFont(font);
        numRows = new QLineEdit(groupBox);
        numRows->setObjectName(QStringLiteral("numRows"));
        numRows->setGeometry(QRect(120, 120, 61, 21));
        numRows->setFont(font);
        numRows->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        numRows->setReadOnly(true);
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 120, 101, 21));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 90, 101, 21));
        label_2->setFont(font1);
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        numCols = new QLineEdit(groupBox);
        numCols->setObjectName(QStringLiteral("numCols"));
        numCols->setGeometry(QRect(120, 90, 61, 21));
        numCols->setFont(font);
        numCols->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        numCols->setReadOnly(true);
        winProc5 = new QLabel(centralWidget);
        winProc5->setObjectName(QStringLiteral("winProc5"));
        winProc5->setGeometry(QRect(580, 380, 261, 231));
        winProc5->setStyleSheet(QStringLiteral("border: 1px solid gray;"));
        winProc5->setScaledContents(true);
        subtitle3 = new QLabel(centralWidget);
        subtitle3->setObjectName(QStringLiteral("subtitle3"));
        subtitle3->setGeometry(QRect(300, 360, 261, 17));
        subtitle3->setFont(font1);
        subtitle3->setAlignment(Qt::AlignCenter);
        subtitle2 = new QLabel(centralWidget);
        subtitle2->setObjectName(QStringLiteral("subtitle2"));
        subtitle2->setGeometry(QRect(580, 80, 261, 17));
        subtitle2->setFont(font1);
        subtitle2->setAlignment(Qt::AlignCenter);
        subtitle1 = new QLabel(centralWidget);
        subtitle1->setObjectName(QStringLiteral("subtitle1"));
        subtitle1->setGeometry(QRect(300, 80, 261, 17));
        subtitle1->setFont(font1);
        subtitle1->setAlignment(Qt::AlignCenter);
        subtitle5 = new QLabel(centralWidget);
        subtitle5->setObjectName(QStringLiteral("subtitle5"));
        subtitle5->setGeometry(QRect(580, 360, 261, 17));
        subtitle5->setFont(font1);
        subtitle5->setAlignment(Qt::AlignCenter);
        mainTitle = new QLabel(centralWidget);
        mainTitle->setObjectName(QStringLiteral("mainTitle"));
        mainTitle->setGeometry(QRect(880, 50, 261, 17));
        QFont font2;
        font2.setPointSize(12);
        font2.setBold(true);
        font2.setWeight(75);
        mainTitle->setFont(font2);
        mainTitle->setAlignment(Qt::AlignCenter);
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(40, 40, 231, 16));
        label_5->setBaseSize(QSize(5, 0));
        QFont font3;
        font3.setFamily(QStringLiteral("Loma"));
        font3.setBold(true);
        font3.setWeight(75);
        label_5->setFont(font3);
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(40, 60, 491, 31));
        label_6->setBaseSize(QSize(5, 0));
        label_6->setFont(font3);
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 440, 231, 171));
        leTFrames = new QLineEdit(groupBox_2);
        leTFrames->setObjectName(QStringLiteral("leTFrames"));
        leTFrames->setGeometry(QRect(122, 30, 91, 23));
        leTFrames->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        leTFrames->setReadOnly(true);
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 30, 101, 21));
        QFont font4;
        font4.setBold(true);
        font4.setWeight(75);
        label_3->setFont(font4);
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(20, 60, 81, 20));
        label_4->setFont(font4);
        leAnalized = new QLineEdit(groupBox_2);
        leAnalized->setObjectName(QStringLiteral("leAnalized"));
        leAnalized->setGeometry(QRect(120, 60, 91, 23));
        leAnalized->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        leAnalized->setReadOnly(true);
        leAccu = new QLineEdit(groupBox_2);
        leAccu->setObjectName(QStringLiteral("leAccu"));
        leAccu->setGeometry(QRect(120, 90, 91, 23));
        leAccu->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        leAccu->setReadOnly(true);
        timeExec = new QLineEdit(groupBox_2);
        timeExec->setObjectName(QStringLiteral("timeExec"));
        timeExec->setGeometry(QRect(120, 130, 91, 31));
        QFont font5;
        font5.setPointSize(11);
        font5.setBold(true);
        font5.setWeight(75);
        timeExec->setFont(font5);
        timeExec->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        timeExec->setReadOnly(true);
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(18, 90, 71, 20));
        label_7->setFont(font4);
        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(20, 130, 91, 31));
        label_8->setFont(font5);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1510, 20));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Camera Calibration", Q_NULLPTR));
        winProc1->setText(QString());
        winFinalRes->setText(QString());
        winProc2->setText(QString());
        winProc3->setText(QString());
        btnLoadVideo->setText(QApplication::translate("MainWindow", "Load Video", Q_NULLPTR));
        btnInitProc->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        lblMsg->setText(QString());
        groupBox->setTitle(QApplication::translate("MainWindow", "Patr\303\263n", Q_NULLPTR));
        rbCircle->setText(QApplication::translate("MainWindow", "Patr\303\263n circular", Q_NULLPTR));
        rbRing->setText(QApplication::translate("MainWindow", "Patr\303\263n anillo", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "N\302\260 filas:", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "N\302\260 columnas:", Q_NULLPTR));
        winProc5->setText(QString());
        subtitle3->setText(QString());
        subtitle2->setText(QString());
        subtitle1->setText(QString());
        subtitle5->setText(QString());
        mainTitle->setText(QApplication::translate("MainWindow", "RESULTADO FINAL", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "CALIBRATION OF CAMERA", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "COMPUTER SCIENCIE UNIVERSITY CATOLICA SAN PABLO", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", " TIME and ACCURACY A", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Total frames :", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "Analizados :", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "Accuracy :", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "TIME AVG: ", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
