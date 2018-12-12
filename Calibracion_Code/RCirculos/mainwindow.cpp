#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    calibrator = new CameraCalibrator();
    calibrator->setVisualizer(this);
    on_rbCircle_clicked();
    on_rbCalibNone_clicked();
    on_rbFrmManual_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnLoadVideo_clicked()
{
    QString pathImage = QFileDialog::getOpenFileName(this, tr("Search video"), "", tr("Video Files (*.avi *.mp4 *.wmv)"));

    if(!calibrator->loadVideo(pathImage.toStdString())) {
        ui->lblMsg->setText("Video is not found");
        ui->btnInitProc->setEnabled(false);
    }
    else {
        ui->lblMsg->setText("");
        ui->btnInitProc->setEnabled(true);
    }
}

void MainWindow::on_btnInitProc_clicked()
{
    ui->btnInitProc->setEnabled(false);

    calibrator->setSizePattern(ui->numRows->text().toInt(), ui->numCols->text().toInt());
    calibrator->setCurrentCalibrator(currCalibrator);
    //calibrator->setCalcDistance(ui->withDistance->isChecked());
    /*if(!ui->rbCalibNone->isChecked()) {
        calibrator->setTypeFrameSelector(currFrameSelector);
        calibrator->setShowUndistort(ui->withUndistort->isChecked());
        calibrator->setSaveCamParams(ui->saveCamParams->isChecked());
        calibrator->setFixAspectRatio(ui->fixAspectRatio->isChecked());
        calibrator->setFixPrincipalPoint(ui->fixPrincipalPoint->isChecked());
        calibrator->setZeroTangentDist(ui->zeroTangentDist->isChecked());
        calibrator->setDistanceKeypoints(ui->centersDistance->text().toFloat());
        calibrator->setNumFramesToCalibration(ui->calNumFrames->text().toUInt());
        calibrator->setPathOutput(ui->calOutputFile->text().toStdString());
    }*/
    //else {
        calibrator->clearCalibrationInputs();
   // }
   // calibrator->setPathCameraParams(ui->fileDistance->text().toStdString());
    calibrator->initProcessing(pattSelected);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    calibrator->setActived(false);
}

void MainWindow::visualizeMsg(std::string msg)
{
    ui->lblMsg->setText(QString::fromStdString(msg));
}

void MainWindow::visualizeValue(string label, double value)
{
    ui->lblMsg->setText(QString::fromStdString(label) + " = " + QString::number(value));
}

void MainWindow::visualizeImage(int id, QImage img, std::string title)
{
    QPixmap image = QPixmap::fromImage(img);
    switch (id) {
    case PROCFIN:
        ui->winFinalRes->setPixmap(image);
        if(title == "") title = "RESULTADO FINAL";
        ui->mainTitle->setText(QString::fromStdString(title));
        break;
    case PROC1:
        ui->winProc1->setPixmap(image);
        ui->subtitle1->setText(QString::fromStdString(title));
        break;
    case PROC2:
        ui->winProc2->setPixmap(image);
        ui->subtitle2->setText(QString::fromStdString(title));
        break;
    case PROC3:
        ui->winProc3->setPixmap(image);
        ui->subtitle3->setText(QString::fromStdString(title));
        break;
    case PROC5:
        ui->winProc5->setPixmap(image);
        ui->subtitle5->setText(QString::fromStdString(title));
        break;

    }
}

void MainWindow::cleanImage(int id)
{
    switch (id) {
    case PROCFIN:
        ui->winFinalRes->clear();
        break;
    case PROC1:
        ui->winProc1->clear();
        ui->subtitle1->setText("");
        break;
    case PROC2:
        ui->winProc2->clear();
        ui->subtitle2->setText("");
        break;
    case PROC3:
        ui->winProc3->clear();
        ui->subtitle3->setText("");
        break;
    case PROC5:
        ui->winProc5->clear();
        ui->subtitle5->setText("");
        break;
    }
}

void MainWindow::on_rbCircle_clicked()
{
    pattSelected = PATT_CIRCLE;
    ui->numRows->setText("4");
    ui->numCols->setText("11");
    //ui->centersDistance->setText("24.8");
    //ui->centersDistance->setText("12.4");
}

void MainWindow::on_rbRing_clicked()
{
    pattSelected = PATT_RING;
    ui->numRows->setText("4");
    ui->numCols->setText("5");
    //ui->centersDistance->setText("50");
}

void MainWindow::on_withDistance_toggled(bool checked)
{
    //ui->btnLoadFileDist->setEnabled(checked);
    //ui->fileDistance->setEnabled(checked);
    /*if(checked)
        ui->rbCalibNone->setChecked(true);
    if(!checked)
        ui->fileDistance->setText("");*/
}

void MainWindow::on_btnLoadFileDist_clicked()
{
    QString pathFile = QFileDialog::getOpenFileName(this, tr("Search files"), "", tr("Text Files (*.xml)"));
    //ui->fileDistance->setText(pathFile);
}

void MainWindow::activateCalibrationParams(bool status)
{
    /*ui->calNumFrames->setEnabled(status);
    ui->calOutputFile->setEnabled(status);
    ui->fixAspectRatio->setEnabled(status);
    ui->fixPrincipalPoint->setEnabled(status);
    ui->zeroTangentDist->setEnabled(status);
    ui->withUndistort->setEnabled(status);
    ui->saveCamParams->setEnabled(status);
    ui->framesOpts->setEnabled(status);*/
}

void MainWindow::on_rbCalibNone_clicked()
{
    currCalibrator = CALIB_NONE;
    activateCalibrationParams(false);
}

void MainWindow::on_rbCalibOpencv_clicked()
{
    currCalibrator = CALIB_OPENCV;
    activateCalibrationParams(true);
    //ui->withDistance->setChecked(false);
}

void MainWindow::on_rbCalibAnkur_clicked()
{
    currCalibrator = CALIB_ANKUR;
    activateCalibrationParams(true);
    //ui->withDistance->setChecked(false);
}

void MainWindow::on_rbFrmManual_clicked()
{
    currFrameSelector = FRAMESEL_MANUAL;
}

void MainWindow::on_rbFrmIntervals_clicked()
{
    currFrameSelector = FRAMESEL_INTERVAL;
}

void MainWindow::on_rbFrmRansac_clicked()
{
    currFrameSelector = FRAMESEL_RANSAC;
}
